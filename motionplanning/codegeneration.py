import os
from os import system
from casadi import NL_X, NL_F, NL_G, NL_NUM_OUT
from casadi import SXFunction, NlpSolver, ExternalFunction
import time


def gen_code_nlp(solver, directory, **kwargs):
    # opt = ', '-O3', '-Os'

    compileme = kwargs['compileme'] if 'compileme' in kwargs else True
    opt = kwargs['opt'] if 'opt' in kwargs else ''
    sx_cast = kwargs['sx_cast'] if 'sx_cast' in kwargs else True

    if not os.path.isdir(directory):
        os.makedirs(directory)

    nlp = solver.nlp()
    if 0:
        grad_f = solver.gradF()
        jac_g = solver.jacG()
        hess_lag = solver.hessLag()
    else:
        grad_f = nlp.gradient(NL_X, NL_F)
        grad_f.init()
        jac_g = nlp.jacobian(NL_X, NL_G)
        jac_g.init()
        grad_lag = nlp.derivative(0, 1)
        hess_lag = grad_lag.jacobian(NL_X, NL_NUM_OUT+NL_X, False, True)
        hess_lag.init()

    if sx_cast:
        nlp = SXFunction(nlp)
        grad_f = SXFunction(grad_f)
        jac_g = SXFunction(jac_g)
        hess_lag = SXFunction(hess_lag)

    nlp.init()
    grad_f.init()
    jac_g.init()
    hess_lag.init()

    src_nlp = directory+'/'+'nlp.c'
    src_grad_f = directory+'/'+'grad_f.c'
    src_jac_g = directory+'/'+'jac_g.c'
    src_hess_lag = directory+'/'+'hess_lag.c'

    src = [src_nlp, src_grad_f, src_jac_g, src_hess_lag]
    names = ['nlp', 'grad_f', 'jac_g', 'hess_lag']

    obj_nlp = directory+'/'+'nlp.so'
    obj_grad_f = directory+'/'+'grad_f.so'
    obj_jac_g = directory+'/'+'jac_g.so'
    obj_hess_lag = directory+'/'+'hess_lag.so'

    obj = [obj_nlp, obj_grad_f, obj_jac_g, obj_hess_lag]

    nlp.generateCode(src_nlp)
    grad_f.generateCode(src_grad_f)
    jac_g.generateCode(src_jac_g)
    hess_lag.generateCode(src_hess_lag)

    if compileme:
        print 'Compiling...',
        t1 = time.time()
        for i in range(4):
            print names[i], '-',
            system('gcc -fPIC -shared -std=c99' +
                   opt + ' ' + src[i] + ' -o ' + obj[i])
        t2 = time.time()
        compile_time = t2 - t1
        print 'Compile time = ', (compile_time)*1e3, ' ms'

    nlp = ExternalFunction('./'+obj_nlp)
    grad_f = ExternalFunction('./'+obj_grad_f)
    jac_g = ExternalFunction('./'+obj_jac_g)
    hess_lag = ExternalFunction('./'+obj_hess_lag)

    solver = NlpSolver('ipopt', nlp)
    solver.setOption('grad_f', grad_f)
    solver.setOption('jac_g', jac_g)
    solver.setOption('hess_lag', hess_lag)
    if compileme:
        return solver, compile_time
    else:
        return solver


def get_nlp_solver(directory):
    obj_nlp = directory+'/'+'nlp.so'
    obj_grad_f = directory+'/'+'grad_f.so'
    obj_jac_g = directory+'/'+'jac_g.so'
    obj_hess_lag = directory+'/'+'hess_lag.so'

    nlp = ExternalFunction('./'+obj_nlp)
    grad_f = ExternalFunction('./'+obj_grad_f)
    jac_g = ExternalFunction('./'+obj_jac_g)
    hess_lag = ExternalFunction('./'+obj_hess_lag)

    solver = NlpSolver('ipopt', nlp)
    solver.setOption('grad_f', grad_f)
    solver.setOption('jac_g', jac_g)
    solver.setOption('hess_lag', hess_lag)
    return solver


def gen_code_function(function, directory, name, **kwargs):

    compileme = kwargs['compileme'] if 'compileme' in kwargs else True
    opt = kwargs['opt'] if 'opt' in kwargs else ''
    sx_cast = kwargs['sx_cast'] if 'sx_cast' in kwargs else False

    if not os.path.isdir(directory):
        os.makedirs(directory)

    if sx_cast:
        function = SXFunction(function)
    function.init()

    src = directory+'/'+name+'.c'
    obj = directory+'/'+name+'.so'

    function.generateCode(src)

    if compileme:
        print 'Compiling function...',
        t1 = time.time()
        system('gcc -fPIC -shared -std=c99' + opt + ' ' + src + ' -o ' + obj)
        t2 = time.time()
        compile_time = t2 - t1
        print 'Compile time = ', compile_time*1e3, ' ms'

    function = ExternalFunction('./'+obj)
    function.init()
    if compileme:
        return function, compile_time
    else:
        return function


def get_function(directory, name):
    obj = directory+'/'+name+'.so'
    function = ExternalFunction('./'+obj)
    function.init()
    return function
