import os
import shutil
import re
from casadi import SXFunction, MXFunction, NlpSolver, nlpIn, nlpOut


# Problem
def export_point2point(self):
    # create data
    data = {}
    data['tol'] = self.options['solver']['tol']
    data['linear_solver'] = self.options['solver']['linear_solver']

    data['spline_type'] = 'typedef struct spline {\n'
    data['spline_type'] += '\tstd::vector<double> knots;\n'
    data['spline_type'] += '\tint degree;\n'
    # create function bodies
    fun = {}
    # create parameter dict
    par = {}
    for l in range(len(self.vehicles)):
        par['y0_'+str(l)] = 'prediction.y'
        par['yTp_'+str(l)] = 'target.y'
        par['dy0_'+str(l)] = 'prediction.dy'
        par['dyT_'+str(l)] = 'target.dy'
    if 'T' in self._parameters:
        par['T'] = 'horizonTime'
        par['t'] = 'fmod(currentTime,'+str(self.knot_time)+')'
    else:

        fun['samplesplines'] = ''
        fun['samplesplines'] += '\tvector<double> time(this->time);\n'
        fun['samplesplines'] += '\tfor(int k=0;k<time.size();k++){\n'
        fun['samplesplines'] += ('\t\ttime[k] += fmod(currentTime,' +
                                 str(self.knot_time)+');\n')
        fun['samplesplines'] += '\t}\n'
        data['spline_type'] += '\tstd::vector<std::vector<double>> transform;\n'
        data['spline_type'] += '}   spline_t;'
    elif(self.__class__.__name__ == 'FreeTPoint2point'):
        par['t'] = '0.0'
        fun['samplesplines'] = ''
        data['spline_type'] += '}   spline_t;'
    return data, fun, par


# Environment
def export_environment(self):
    # create data
    data = {}
    data['n_dim'] = self.n_dim
    data['n_obs'] = self.No
    # create function bodies
    fun = {}
    # create parameter dict
    par = {}
    for obs in self.obstacles:
        par['x_'+str(obs)] = 'obstacles['+str(obs.index)+'].position'
        par['v_'+str(obs)] = 'obstacles['+str(obs.index)+'].velocity'
        par['a_'+str(obs)] = 'obstacles['+str(obs.index)+'].acceleration'
    return data, fun, par


# Vehicle
def export_vehicle(self):
    # create data
    data = {}
    data['n_y'] = self.n_y
    data['n_dy'] = self.n_dy
    data['n_der_dep'] = self.n_der_dep
    # create function bodies
    fun = {}
    a, bdy = self._get_function_expression(self._signals_expr['dstate'],
                                           [self._state, self._input],
                                           _translate_expression_cpp)
    fun['dstate'] = '{'+bdy[1:-1]+'}'
    a, bdy = self._get_function_expression(self._signals_expr['input'],
                                           self._y, _translate_expression_cpp)
    for i in range(self.n_y):
        for j in range(self.order+1):
            bdy = bdy.replace('[%d,%d]' % (i, j), '[%d][%d]' % (i, j))
    fun['input'] = '{'+bdy[1:-1]+'}'
    a, bdy = self._get_function_expression(self._signals_expr['y'],
                                           [self._state, self._input],
                                           _translate_expression_cpp)

    bdy = '{{'+bdy[2:-2]+'}}'
    bdy = bdy.replace(' ', '')
    n_zeros = self.order-self.n_der_dep
    bdy = bdy.replace('],[', ','+','.join(['0' for k in range(n_zeros)])+'},{')
    bdy = bdy.replace('}}', ','+','.join(['0' for k in range(n_zeros)])+'}}')
    fun['y'] = bdy
    # create parameter dict
    par = {}
    return data, fun, par


def _translate_expression_cpp(expression):
    # remove '\n'
    expression = expression.translate(None, '\n')
    # sq() means power of 2
    while expression.find('sq(') >= 0:
        splt = expression.split('sq(', 1)
        a = splt[1]
        br_in_between = len(a.split(')')[0].split('(')) - 1
        splt2 = a.split(')', br_in_between+1)
        expression = (splt[0]+'pow('+')'.join(splt2[:br_in_between+1])
                      + ','+str(2)+')' + splt2[-1])
    # find @i
    cnt = 1
    while expression.find('@'+str(cnt)) >= 0:
        value = expression.split('@'+str(cnt)+'=')[1].split(',')[0]
        expression = re.sub(r'@%d\b' % cnt, value, expression)
        expression = ','.join(expression.split(',')[1:])
        cnt += 1
    return expression


# Father
def export_father(self):
    data, par, fun = {}, {}, {}
    for label, child in self.children.items():
        d, f, p = child._export(child)
        data.update(d)
        par.update(p)
        fun.update(f)
    data['n_var'] = self._var_struct.size
    data['n_par'] = self._par_struct.size
    data['n_con'] = self._con_struct.size
    return data, fun, par


class Export:

    def __init__(self, problem, filefamily, options):
        self.set_default_options()
        self.set_options(options)
        self.filefamily = filefamily
        self.father = problem.father
        self.problem = problem
        if len(problem.vehicles) > 1:
            raise ValueError(('Only export for single vehicle ' +
                              'problems is supported.'))
        self.vehicle = problem.vehicles[0]
        self.gen_export_functions()
        self.export()

    def gen_export_functions(self):
        self.vehicle._export = export_vehicle
        self.problem.environment._export = export_environment
        self.problem._export = export_point2point
        self.father._export = export_father

    def set_default_options(self):
        self.options = {}
        self.options['directory'] = os.path.join(os.getcwd(), 'export/')
        self.options['casadi_optiflags'] = ''  # '', '-O3', '-Os'
        self.options['remote_casadi_dir'] = None

    def set_options(self, options):
        self.options.update(options)
        if not os.path.isdir(self.options['directory']):
            os.makedirs(self.options['directory'])

    def perform_checks(self):
        if not self.options['remote_casadi_dir']:
            raise ValueError('Set the path of the casadi install directory '
                             'on the remote system via the option '
                             '"remote_casadi_dir".')

    def export(self):
        self.perform_checks()
        print 'Exporting ...',
        # copy files
        destination = self.options['directory']
        source = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                              'export/'+self.filefamily)
        self.copy_files(source, destination)
        # get_make_options
        make_opt = self.get_make_options()
        # create nlp src files
        make_opt['casadi_src'] = self.create_nlp_src(destination)
        # retrieve info from children
        data, fun, par = self.father._export(self.father)
        # generate function bodies
        bodies = {}
        bodies['initVariables'] = self.gen_initVariables()
        bodies['setParameters'] = self.gen_setParameters(par)
        bodies['updateBounds'], data['lbg'], data['ubg'] = self.gen_updateBounds()
        bodies['updateModel'] = self.gen_updateModel(fun['dstate'])
        bodies['getInput'] = self.gen_getInput(fun['input'])
        bodies['getY'] = self.gen_getY(fun['y'])
        bodies['sampleSplines'] = self.gen_sampleSplines(fun['samplesplines'])
        bodies['initSplines'], data['spline_info'] = self.gen_initSplines()
        bodies['transformSplines'] = self.gen_transformSplines()
        # fill templates
        self.fill_template(bodies, os.path.join(destination+'src/',
                                             'MotionPlanning.cpp'))
        self.fill_template(data, os.path.join(destination+'src/',
                                              'MotionPlanning_def.hpp'))
        self.fill_template(make_opt, os.path.join(destination, 'Makefile'))
        print 'done.'

    def copy_files(self, source, destination):
        # copy src files
        src_files = ['MotionPlanning.cpp', 'MotionPlanning.hpp',
                     'MotionPlanning_def.hpp']
        src_dir = os.path.join(destination, 'src/')
        if not os.path.isdir(src_dir):
            os.makedirs(src_dir)
        for f in src_files:
            shutil.copy(os.path.join(source, f),
                        os.path.join(src_dir, f))
        # copy Makefile
        shutil.copy(os.path.join(source, 'Makefile'),
                    os.path.join(destination, 'Makefile'))

    def get_make_options(self):
        make_opt = {}
        make_opt['casadidir'] = self.options['remote_casadi_dir']
        make_opt['casadi_optiflags'] = self.options['casadi_optiflags']
        return make_opt

    def create_nlp_src(self, destination):
        obj = self.father.problem_description['obj']
        con = self.father.problem_description['con']
        var = self.father.problem_description['var']
        par = self.father.problem_description['par']
        opt = self.father.problem_description['opt']
        nlp = MXFunction('nlp', nlpIn(x=var, p=par), nlpOut(f=obj, g=con))
        solver = NlpSolver('solver', 'ipopt', nlp, opt['solver'])
        grad_f, jac_g = nlp.gradient('x', 'f'), nlp.jacobian('x', 'g')
        hess_lag = solver.hessLag()
        functions = {'nlp': nlp, 'grad_f': grad_f,
                     'jac_g': jac_g, 'hess_lag': hess_lag}
        # sx cast for much faster execution
        functions = {name: SXFunction(fun) for name, fun in functions.items()}
        cwd = os.getcwd()
        src_files = ''
        for name, fun in functions.items():
            fun.generate(name)
            shutil.move(cwd+'/'+name+'.c',
                        os.path.join(destination+'src/', name+'.c'))
            src_files += name+'.c' + ' '
        return src_files

    def fill_template(self, data, files):
        if not isinstance(files, list):
            files = [files]
        for file in files:
            with open(file, 'r+') as f:
                if f is not None:
                    body = f.read()
                    for key, item in data.items():
                        body = body.replace('@'+key+'@', str(item))
                    f.seek(0)
                    f.truncate()
                    f.write(body)
                else:
                    raise ValueError('File '+os.path.join(self.src_dir, file) +
                                     ' does not exist!')

    def gen_setParameters(self, par_cpp):
        body, cnt = '', 0
        for label, child in self.father.children.items():
            for name, par in child._parameters.items():
                if par.size() > 1:
                    body += '\tfor (int i=0; i<'+str(par.size())+'; i++){\n'
                    body += ('\t\tparameters['+str(cnt)+'+i] = ' +
                             par_cpp[name]+'[i];\n')
                    body += '\t}\n'
                else:
                    body += '\tparameters['+str(cnt)+'] = '+par_cpp[name]+';\n'
                cnt += par.size()
        return body

    def gen_initVariables(self):
        body, cnt = '', 0
        for label, child in self.father.children.items():
            for name, var in child._variables.items():
                if name == 'y':
                    deg = child.degree
                    internal = var.shape[0]-2*deg
                    body += '\tfor (int i=0; i<'+str(var.shape[1])+'; i++){\n'
                    body += '\t\tfor (int j=0; j<'+str(deg)+'; j++){\n'
                    body += ('\t\t\tvariables['+str(cnt)+'+i*' +
                             str(var.shape[0])+'+j] = state.y[i];\n')
                    body += ('\t\t\tvariables['+str(cnt+internal+deg) +
                             '+i*'+str(var.shape[0])+'+j] = target.y[i];\n')
                    body += '\t\t}\n'
                    body += '\t\tfor (int j=0; j<'+str(internal)+'; j++){\n'
                    body += ('\t\t\tvariables['+str(cnt+deg)+'+i*' +
                             str(var.shape[0])+'+j] = state.y[i]+j*' +
                             '(target.y[i] - state.y[i])/(' +
                             str(internal-1)+');\n')
                    body += '\t\t}\n'
                    body += '\t}\n'
                cnt += var.size()
        return body

    def gen_updateBounds(self):
        body, cnt = '', 0
        lb, ub = '{', '{'
        shutdowns = {'greater': {}, 'lesser': {}, 'equal': {}}
        for label, child in self.father.children.items():
            for name, con in child._constraints.items():
                if child._add_label(name) in self.father._constraint_shutdown:
                    shutdown = self.father._constraint_shutdown[child._add_label(name)]
                    for rel, bnd in shutdown.items():
                        if bnd not in shutdowns[rel].items():
                            shutdowns[rel][bnd] = []
                        shutdowns[rel][bnd].extend(
                            [cnt+i for i in range(con[0].size())])
                if con[0].size() > 1:
                    for l in range(con[0].size()):
                        lb += str(con[1][l])+','
                        ub += str(con[2][l])+','
                else:
                    lb += str(con[1])+','
                    ub += str(con[2])+','
                cnt += con[0].size()
        lb = lb[:-1]+'}'
        ub = ub[:-1]+'}'

        _lbg, _ubg = self.father._lb.cat, self.father._ub.cat
        for rel, item in shutdowns.items():
            if rel == 'greater':
                op = '>'
            elif rel == 'lesser':
                op = '<'
            elif rel == 'equal':
                op = '=='
            for bnd, indices in item.items():
                body += '\tif(currentTime '+op+' '+str(bnd)+'){\n'
                for ind in indices:
                    body += '\t\tlbg['+str(ind)+'] = -inf;\n'
                    body += '\t\tubg['+str(ind)+'] = +inf;\n'
                body += '\t}else{\n'
                for ind in indices:
                    body += '\t\tlbg['+str(ind)+'] = '+str(_lbg[ind])+';\n'
                    body += '\t\tubg['+str(ind)+'] = '+str(_ubg[ind])+';\n'
                body += '\t}\n'

        return body, lb, ub

    def gen_updateModel(self, fun):
        body = '\tvector<double> _dstate ' + fun + ';\n'
        body += '\tdstate = _dstate;\n'
        return body

    def gen_getInput(self, fun):
        body = '\tvector<double> _input ' + fun + ';\n'
        body += '\tinput = _input;\n'
        return body

    def gen_getY(self, fun):
        body = '\tvector<vector<double>> _y ' + fun + ';\n'
        body += '\ty = _y;\n'
        return body

    def gen_initSplines(self):
        body, spline_info = '', ''

        # specific info for y splines
        derT = '{'
        for d in range(1, self.vehicle.n_der+1):
            B, P = self.vehicle.basis.derivative(d)
            P = P.toarray()
            derT += '{'
            for i in range(P.shape[0]):
                derT += '{'+','.join([str(p) for p in P[i, :].tolist()])+'},'
            derT = derT[:-1]+'},'
        derT = derT[:-1]+'}'
        spline_info += '#define derT_def ' + derT +'\n'

        # this is also for other splines
        for label, child in self.father.children.items():
            for name in child._splines_prim:
                spl = child._splines_prim[name]
                basis = spl['basis']
                spline_info += '#define len_'+name+' '+str(len(basis))+'\n'
                spline_info += '#define '+name+'_degree '+str(basis.degree)+'\n'
                spline_info += ('#define '+name+'_knots '+'{' +
                                ','.join([str(k) for k in basis.knots.tolist()])+'}\n')
                if spl['init'] is not None:
                    tf = '{'
                    for k in range(spl['init'].shape[0]):
                        tf += '{'+','.join([str(t) for t in spl['init'][k].tolist()])+'},'
                    tf = tf[:-1]+'}'
                    spline_info += '#define '+name+'_tf '+tf+'\n'

                body += ('\tspline_t spline_'+name+' = {'+name +
                         '_knots,'+name+'_degree,'+name+'_tf};\n')
                body += '\tsplines["'+name+'"] = spline_' + name + ';\n'
        return body, spline_info
    def gen_sampleSplines(self, fun):
        return fun

    def gen_transformSplines(self):
        body, cnt = '', 0
        if self.problem.__class__.__name__ == 'FixedTPoint2point':
            body += ('\tif(((currentTime > 0) and (fmod(currentTime, ' +
                     str(self.problem.knot_time)+') == 0.0))){\n')
            body += ('\t\tvector<double> spline_tf(' +
                     str(len(self.vehicle.basis))+');\n')
            for label, child in self.father.children.items():
                for name, var in child._variables.items():
                    if name in child._splines_prim:
                        tf = 'splines["'+name+'"].transform'
                        body += ('\t\tfor(int k=0; k<' +
                                 str(var.shape[1])+'; k++){\n')
                        body += ('\t\t\tfor(int i=0; i<' +
                                 str(var.shape[0])+'; i++){\n')
                        body += ('\t\t\t\tfor(int j=0; j<' +
                                 str(var.shape[0])+'; j++){\n')
                        body += ('\t\t\t\t\tspline_tf[i] = '+tf +
                                 '[i][j]*variables['+str(cnt)+'+k*' +
                                 str(var.shape[0])+'+j];\n')
                        body += '\t\t\t\t}\n'
                        body += '\t\t\t}\n'
                        body += ('\t\t\tfor(int i=0; i<'+str(var.shape[0]) +
                                 '; i++){\n')
                        body += ('\t\t\t\tvariables['+str(cnt)+'+k*' +
                                 str(var.shape[0])+'+i] = spline_tf[i];\n')
                        body += '\t\t\t}\n'
                        body += '\t\t}\n'
                    cnt += var.size()
            body += '\t}'
        elif self.problem.__class__.__name__ == 'FreeTPoint2point':
            raise Warning('Initialization for free time problem ' +
                          'not implemented (yet?).')
        return body
