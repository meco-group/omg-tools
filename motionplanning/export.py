import os
import shutil
import re
from casadi import SXFunction, MXFunction, NlpSolver, nlpIn, nlpOut


# Problem
def export_point2point(self):
    info = {}
    # create defines
    defines = {}
    defines['TOL'] = self.options['solver']['tol']
    defines['LINEAR_SOLVER'] = '"'+self.options['solver']['linear_solver']+'"'
    # create types
    types = {}
    types['spline_t'] = 'struct spline {\n'
    types['spline_t'] += '\tstd::vector<double> knots;\n'
    types['spline_t'] += '\tint degree;\n'
    # create function bodies
    fun = {}
    fun['setParameters'] = {}
    for l in range(len(self.vehicles)):
        fun['setParameters']['y0_'+str(l)] = 'y0'
        fun['setParameters']['yTp_'+str(l)] = 'yT'
        fun['setParameters']['dy0_'+str(l)] = 'dy0'
        fun['setParameters']['dyT_'+str(l)] = 'dyT'
    if(self.__class__.__name__ == 'FixedTPoint2point'):
        fun['setParameters']['T'] = 'horizon_time'
        fun['setParameters']['t'] = ('fmod(round(current_time*1000.)/1000.,' +
                                     'horizon_time/' +
                                     str(self.vehicles[0].knot_intervals)+')')
        fun['sampleSplines'] = ''
        fun['sampleSplines'] += '\tvector<double> time(this->time);\n'
        fun['sampleSplines'] += '\tfor(int k=0;k<time.size();k++){\n'
        fun['sampleSplines'] += ('\t\ttime[k] += ' +
                                 'fmod(round(current_time*1000.)/1000.,' +
                                 'horizon_time/' +
                                 str(self.vehicles[0].knot_intervals)+');\n')
        fun['sampleSplines'] += '\t}\n'
        types['spline_t'] += '\tstd::vector<std::vector<double>> transform;\n'
        types['spline_t'] += '}'

    elif(self.__class__.__name__ == 'FreeTPoint2point'):
        fun['setParameters']['t'] = '0.0'
        fun['sampleSplines'] = ''
        types['spline_t'] += '}'
    else:
        raise ValueError('Problems of type ' + self.__class__.__name__ +
                         ' are not supported.')
    info.update({'defines': defines, 'types': types, 'functions': fun})
    return info


# Environment
def export_environment(self):
    info = {}
    # create defines
    defines = {}
    defines['N_DIM'] = self.n_dim
    defines['N_OBS'] = self.No
    # create function bodies
    fun = {}
    fun['setParameters'] = {}
    for obs in self.obstacles:
        fun['setParameters']['x_'+str(obs)] = 'obstacles['+str(obs.index)+'].position'
        fun['setParameters']['v_'+str(obs)] = 'obstacles['+str(obs.index)+'].velocity'
        fun['setParameters']['a_'+str(obs)] = 'obstacles['+str(obs.index)+'].acceleration'
    info.update({'defines': defines, 'functions': fun})
    return info


# Vehicle
def export_vehicle(self):
    info = {}
    # create defines
    defines = {}
    defines['N_Y'] = self.n_y
    defines['N_DER'] = self.n_der
    defines['ORDER'] = self.order
    defines['N_ST'] = self.n_st
    defines['N_IN'] = self.n_in
    derT = '{'  # spline derivative matrices
    for d in range(1, self.n_der+1):
        B, P = self.basis.derivative(d)
        P = P.toarray()
        derT += '{'
        for i in range(P.shape[0]):
            derT += '{'+','.join([str(p) for p in P[i, :].tolist()])+'},'
        derT = derT[:-1]+'},'
    derT = derT[:-1]+'}'
    defines['DERT_DEF'] = derT
    # create function bodies
    fun = {}
    a, bdy = self._get_function_expression(self._signals_expr['dstate'],
                                           [self._state, self._input],
                                           _translate_expression_cpp)
    fun['updateModel'] = '{'+bdy[1:-1]+'}'
    a, bdy = self._get_function_expression(self._signals_expr['state'],
                                           self._y, _translate_expression_cpp)
    for i in range(self.n_y):
        for j in range(self.order+1):
            bdy = bdy.replace('[%d,%d]' % (i, j), '[%d][%d]' % (i, j))
    fun['getState'] = '{'+bdy[1:-1]+'}'
    a, bdy = self._get_function_expression(self._signals_expr['input'],
                                           self._y, _translate_expression_cpp)
    for i in range(self.n_y):
        for j in range(self.order+1):
            bdy = bdy.replace('[%d,%d]' % (i, j), '[%d][%d]' % (i, j))
    fun['getInput'] = '{'+bdy[1:-1]+'}'
    a, bdy = self._get_function_expression(self._signals_expr['y'],
                                           [self._state, self._input],
                                           _translate_expression_cpp)
    bdy = '{{'+bdy[2:-2]+'}}'
    bdy = bdy.replace(' ', '')
    n_zeros = self.n_der-self.order
    bdy = bdy.replace('],[', ','+','.join(['0' for k in range(n_zeros)])+'},{')
    bdy = bdy.replace('}}', ','+','.join(['0' for k in range(n_zeros)])+'}}')
    fun['getY'] = bdy
    info.update({'defines': defines, 'functions': fun})
    return info


def _translate_expression_cpp(expression):
    # remove '\n'
    expression = expression.translate(None, '\n')
    # sq() means power of 2
    while expression.find('sq(') >= 0:
        splt = expression.split('sq(', 1)
        a = splt[1]
        br_in_between = len(a.split(')')[0].split('(')) - 1
        splt2 = a.split(')', br_in_between+1)
        expression = (splt[0]+'pow('+')'.join(splt2[:br_in_between+1]) +
                      ','+str(2)+')' + splt2[-1])
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
    # create defines
    defines = {}
    defines.update({'N_VAR': self._var_struct.size})
    defines.update({'N_PAR': self._par_struct.size})
    defines.update({'N_CON': self._con_struct.size})
    # spline info
    for label, child in self.children.items():
        for name in child._splines_prim:
            spl = child._splines_prim[name]
            basis = spl['basis']
            knots = '{'+','.join([str(k) for k in basis.knots.tolist()])+'}'
            defines.update({('%s_LENGTH') % name.upper(): str(len(basis))})
            defines.update({('%s_DEGREE') % name.upper(): str(basis.degree)})
            defines.update({('%s_KNOTS')  % name.upper(): knots})
            if spl['init'] is not None:
                tf = '{'
                for k in range(spl['init'].shape[0]):
                    tf += '{'+','.join([str(t) for t in spl['init'][k].tolist()])+'},'
                tf = tf[:-1]+'}'
                defines.update({('%s_TF') % name.upper(): tf})
    # lbg & ubg
    lb, ub, cnt = '{', '{', 0
    for label, child in self.children.items():
        for name, con in child._constraints.items():
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
    defines.update({'LBG_DEF': lb, 'UBG_DEF': ub})
    # create info
    info = {'defines': defines}
    for label, child in self.children.items():
        _merge_dict(info, child._export(child))
    return info


def _merge_dict(dict1, dict2):
    for key, item in dict2.items():
        if isinstance(item, dict):
            if key not in dict1:
                dict1[key] = {}
            _merge_dict(dict1[key], item)
        else:
            dict1[key] = item


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
        self.create_export_functions()
        self.export()

    def set_default_options(self):
        self.options = {}
        self.options['directory'] = os.path.join(os.getcwd(), 'export/')
        self.options['casadi_optiflags'] = ''  # '', '-O3', '-Os'
        self.options['casadidir'] = ''
        self.options['casadiobj'] = '.'
        self.options['sourcefiles'] = ''
        self.options['executable'] = 'MotionPlanning'

    def set_options(self, options):
        self.options.update(options)
        if not os.path.isdir(self.options['directory']):
            os.makedirs(self.options['directory'])

    def export():
        raise NotImplementedError('Please implement this method!')


class ExportCpp(Export):

    def create_export_functions(self):
        self.vehicle._export = export_vehicle
        self.problem.environment._export = export_environment
        self.problem._export = export_point2point
        self.father._export = export_father

    def perform_checks(self):
        if (self.vehicle.options['boundary_smoothness']['internal'] !=
            self.vehicle.order):
            raise ValueError('Only internal boundary smoothness equal to ' +
                             'vehicle\'s order is supported.')

    def export(self):
        self.perform_checks()
        print 'Exporting ...',
        # copy files
        source = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                              'export/'+self.filefamily)
        self.copy_files(source, self.options['directory'])
        # retrieve info from children
        info = self.father._export(self.father)
        data = {}
        # create nlp src files and get make options
        data.update(self.create_nlp_src(self.options['directory']))
        data.update(self.get_make_options())
        # generate defines
        data.update(self.create_defines(info))
        # generate types
        data.update(self.create_types(info))
        # generate function bodies
        data.update(self.create_functions(info))
        # write info to source files
        self.write_files(data)
        print 'done.'
        print 'Check out instructions.txt for build and usage instructions.'

    def copy_files(self, source, destination):
        src_files = ['MotionPlanning.cpp', 'MotionPlanning.hpp', 'example.cpp']
        src_dir = os.path.join(destination, 'src/')
        if not os.path.isdir(src_dir):
            os.makedirs(src_dir)
        for f in src_files:
            shutil.copy(os.path.join(source, f),
                        os.path.join(src_dir, f))
        root_files = ['Makefile', 'instructions.txt']
        for f in root_files:
            shutil.copy(os.path.join(source, f),
                        os.path.join(destination, f))

    def write_files(self, info):
        files = ['src/MotionPlanning.cpp', 'src/MotionPlanning.hpp',
                 'Makefile']
        files = [os.path.join(self.options['directory'], f) for f in files]
        self.fill_template(info, files)

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

    def get_make_options(self):
        make_opt = {}
        for key, option in self.options.items():
            make_opt[key] = option
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
        return {'casadi_src': src_files}

    def create_defines(self, info):
        if 'defines' in info:
            code = ''
            for name, define in info['defines'].items():
                code += '#define ' + str(name) + ' ' + str(define) + '\n'
            return {'defines': code}
        else:
            return {}

    def create_types(self, info):
        if 'types' in info:
            code = ''
            for name, type in info['types'].items():
                code += 'typedef ' + type + name + ';\n'
            return {'types': code}
        else:
            return {}

    def create_functions(self, info):
        code = {}
        fun = info['functions']
        code.update(self._create_initVariables())
        code.update(self._create_setParameters(fun['setParameters']))
        code.update(self._create_updateBounds())
        code.update(self._create_updateModel(fun['updateModel']))
        code.update(self._create_getState(fun['getState']))
        code.update(self._create_getInput(fun['getInput']))
        code.update(self._create_getY(fun['getY']))
        code.update(self._create_interpreteVariables())
        code.update(self._create_sampleSplines(fun['sampleSplines']))
        code.update(self._create_initSplines())
        code.update(self._create_transformSplines())
        return code

    def _create_setParameters(self, parameters):
        code, cnt = '', 0
        for label, child in self.father.children.items():
            for name, par in child._parameters.items():
                if par.size() > 1:
                    code += '\tfor (int i=0; i<'+str(par.size())+'; i++){\n'
                    code += ('\t\tparameters['+str(cnt)+'+i] = ' +
                             parameters[name]+'[i];\n')
                    code += '\t}\n'
                else:
                    code += '\tparameters['+str(cnt)+'] = '+parameters[name]+';\n'
                cnt += par.size()
        return {'setParameters': code}

    def _create_initVariables(self):
        code, cnt = '', 0
        for label, child in self.father.children.items():
            for name, var in child._variables.items():
                if name == 'y':
                    deg = child.degree
                    internal = var.shape[0]-2*deg
                    code += '\tfor (int i=0; i<'+str(var.shape[1])+'; i++){\n'
                    code += '\t\tfor (int j=0; j<'+str(deg)+'; j++){\n'
                    code += ('\t\t\tvariables['+str(cnt)+'+i*' +
                             str(var.shape[0])+'+j] = y0[i][0];\n')
                    code += ('\t\t\tvariables['+str(cnt+internal+deg) +
                             '+i*'+str(var.shape[0])+'+j] = yT[i][0];\n')
                    code += '\t\t}\n'
                    code += '\t\tfor (int j=0; j<'+str(internal)+'; j++){\n'
                    code += ('\t\t\tvariables['+str(cnt+deg)+'+i*' +
                             str(var.shape[0])+'+j] = y0[i][0]+j*' +
                             '(yT[i][0] - y0[i][0])/(' +
                             str(internal-1)+');\n')
                    code += '\t\t}\n'
                    code += '\t}\n'
                cnt += var.size()
        return {'initVariables': code}

    def _create_updateBounds(self):
        code, cnt = '', 0
        _lbg, _ubg = self.father._lb.cat, self.father._ub.cat
        for label, child in self.father.children.items():
            for name, con in child._constraints.items():
                if child._add_label(name) in self.father._constraint_shutdown:
                    shutdown = self.father._constraint_shutdown[child._add_label(name)]
                    cond = shutdown.replace('t', 'current_time')
                    code += '\tif(' + cond + '){\n'
                    for i in range(con[0].size()):
                        code += '\t\tlbg['+str(cnt+i)+'] = -INF;\n'
                        code += '\t\tubg['+str(cnt+i)+'] = +INF;\n'
                    code += '\t}else{\n'
                    for i in range(con[0].size()):
                        code += ('\t\tlbg['+str(cnt+i)+'] = ' +
                                 str(_lbg[cnt+i]) + ';\n')
                        code += ('\t\tubg['+str(cnt+i)+'] = ' +
                                 str(_ubg[cnt+i]) + ';\n')
                    code += '\t}\n'
                cnt += con[0].size()
        return {'updateBounds': code}

    def _create_updateModel(self, fun):
        code = '\tvector<double> _dstate ' + fun + ';\n'
        code += '\tdstate = _dstate;\n'
        return {'updateModel': code}

    def _create_getState(self, fun):
        code = '\tvector<double> _state ' + fun + ';\n'
        code += '\tstate = _state;\n'
        return {'getState': code}

    def _create_getInput(self, fun):
        code = '\tvector<double> _input ' + fun + ';\n'
        code += '\tinput = _input;\n'
        return {'getInput': code}

    def _create_getY(self, fun):
        code = '\tvector<vector<double>> _y ' + fun + ';\n'
        code += '\ty = _y;\n'
        return {'getY': code}

    def _create_initSplines(self):
        code = ''
        for label, child in self.father.children.items():
            for name in child._splines_prim:
                code += ('\tspline_t spline_'+name+' = {'+name.upper() +
                         '_KNOTS,'+name.upper()+'_DEGREE,'+name.upper()+'_TF};\n')
                code += '\tsplines["'+name+'"] = spline_' + name + ';\n'
        return {'initSplines': code}

    def _create_interpreteVariables(self):
        code, cnt = '', 0
        for label, child in self.father.children.items():
            for name, var in child._variables.items():
                if name == 'y':
                    code += '\tfor (int i=0; i<'+str(var.shape[1])+'; i++){\n'
                    code += '\t\tfor (int j=0; j<'+str(var.shape[0])+'; j++){\n'
                    code += ('\t\t\ty_coeffs[i][j] = variables['+str(cnt) +
                             '+i*'+str(var.shape[0])+'+j];\n')
                    code += '\t\t}\n'
                    code += '\t}\n'
                if name == 'T':
                    code += '\thorizon_time = variables['+str(cnt)+'];\n';
                cnt += var.size()
        return {'interpreteVariables': code}

    def _create_sampleSplines(self, fun):
        return {'sampleSplines': fun}

    def _create_transformSplines(self):
        code, cnt = '', 0
        if self.problem.__class__.__name__ == 'FixedTPoint2point':
            code += ('\tif(((current_time > 0) and ' +
                     'fabs(fmod(round(current_time*1000.)/1000., ' +
                     'horizon_time/' + str(self.vehicle.knot_intervals)+')) <1.e-6)){\n')
            code += ('\t\tvector<double> spline_tf(' +
                     str(len(self.vehicle.basis))+');\n')
            for label, child in self.father.children.items():
                for name, var in child._variables.items():
                    if name in child._splines_prim:
                        tf = 'splines["'+name+'"].transform'
                        code += ('\t\tfor(int k=0; k<' +
                                 str(var.shape[1])+'; k++){\n')
                        code += ('\t\t\tfor(int i=0; i<' +
                                 str(var.shape[0])+'; i++){\n')
                        code += '\t\t\tspline_tf[i] = 0.0;\n'
                        code += ('\t\t\t\tfor(int j=0; j<' +
                                 str(var.shape[0])+'; j++){\n')
                        code += ('\t\t\t\t\tspline_tf[i] += '+tf +
                                 '[i][j]*variables['+str(cnt)+'+k*' +
                                 str(var.shape[0])+'+j];\n')
                        code += '\t\t\t\t}\n'
                        code += '\t\t\t}\n'
                        code += ('\t\t\tfor(int i=0; i<'+str(var.shape[0]) +
                                 '; i++){\n')
                        code += ('\t\t\t\tvariables['+str(cnt)+'+k*' +
                                 str(var.shape[0])+'+i] = spline_tf[i];\n')
                        code += '\t\t\t}\n'
                        code += '\t\t}\n'
                    cnt += var.size()
            code += '\t}'
        elif self.problem.__class__.__name__ == 'FreeTPoint2point':
            raise Warning('Initialization for free time problem ' +
                          'not implemented (yet?).')
        return {'transformSplines': code}
