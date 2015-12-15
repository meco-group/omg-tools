import os
from os import system
from casadi import SXFunction, NlpSolver, ExternalFunction
import time


def gen_code_nlp(solver, directory, options,
                 compileme=True, opt='', sx_cast=True):
    # opt = '', '-O3', '-Os'

    if not os.path.isdir(directory):
        os.makedirs(directory)

    nlp = solver.nlp()
    grad_f = nlp.gradient('x', 'f')
    jac_g = nlp.jacobian('x', 'g')
    hess_lag = solver.hessLag()

    functions = {'nlp': nlp, 'grad_f': grad_f,
                 'jac_g': jac_g, 'hess_lag': hess_lag}

    if sx_cast:  # for much faster execution
        functions = {name: SXFunction(fun) for name, fun in functions.items()}

    src = {name: directory+'/'+name+'.c' for name in functions.keys()}
    obj = {name: directory+'/'+name+'.so' for name in functions.keys()}

    cwd = os.getcwd()
    for name, fun in functions.items():
        fun.generate(name)
        os.rename(cwd+'/'+name+'.c', cwd+'/'+directory+'/'+name+'.c')

    if compileme:
        print 'Compiling...',
        t1 = time.time()
        for name, fun in functions.items():
            print name + ' -',
            system('gcc -fPIC -shared -std=c99' +
                   opt + ' ' + src[name] + ' -o ' + obj[name])
        t2 = time.time()
        compile_time = t2 - t1
        print 'Compile time = %5g s' % (compile_time)

    solver = get_nlp_solver(directory, options)

    if compileme:
        return solver, compile_time
    else:
        return solver


def get_nlp_solver(directory, options):
    names = ['nlp', 'grad_f', 'jac_g', 'hess_lag']
    obj = {name: directory+'/'+name+'.so' for name in names}
    functions = {name: ExternalFunction(name, './'+ob)
                 for name, ob in obj.items()}

    options.update({'grad_f': functions['grad_f'],
                    'jac_g': functions['jac_g'],
                    'hess_lag': functions['hess_lag']})

    solver = NlpSolver('solver', 'ipopt', functions['nlp'], options)

    return solver


def gen_code_function(function, directory, name,
                      compileme=True, opt='', sx_cast=True):

    if not os.path.isdir(directory):
        os.makedirs(directory)

    if sx_cast:
        function = SXFunction(function)

    src = directory+'/'+name+'.c'
    obj = directory+'/'+name+'.so'

    cwd = os.getcwd()
    function.generate(name)
    os.rename(cwd+'/'+name+'.c', cwd+'/'+directory+'/'+name+'.c')

    if compileme:
        print 'Compiling function '+name+'...',
        t1 = time.time()
        system('gcc -fPIC -shared -std=c99' + opt + ' ' + src + ' -o ' + obj)
        t2 = time.time()
        compile_time = t2 - t1
        print 'Compile time = %5g s' % (compile_time)

    function = get_function(directory, name)

    if compileme:
        return function, compile_time
    else:
        return function


def get_function(directory, name):
    obj = directory+'/'+name+'.so'
    function = ExternalFunction(name, './'+obj)
    return function
