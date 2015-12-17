from optilayer import OptiFather
from problem import Problem
from distributedproblem import DistributedProblem
from casadi import symvar, mul, SX, MX, SXFunction, MXFunction
from casadi import vertcat, horzcat, jacobian, solve
from casadi.tools import struct, struct_symMX, entry
from spline_extra import shift_knot1_fwd_mx, shift_knot1_bwd_mx
from copy import deepcopy
import numpy as np
import time


def _create_struct_from_dict(dictionary):
    entries = []
    for key, data in dictionary.items():
        if isinstance(data, dict):
            stru = _create_struct_from_dict(data)
            entries.append(entry(str(key), struct=stru))
        else:
            entries.append(entry(key, shape=len(data)))
    return struct(entries)


class ADMM(Problem):

    def __init__(self, index, vehicle, problem, environment, options={}):
        Problem.__init__(self, vehicle, environment, options, label='admm')
        self.problem = problem

        self.group = {child.label: child for child in [
            vehicle, problem, environment, self]}
        for child in self.group.values():
            child.index = index


    # ========================================================================
    # ADMM options
    # ========================================================================

    def set_default_options(self):
        Problem.set_default_options(self)
        self.options['admm'] = {'rho': 0.1}

    # ========================================================================
    # Create problem
    # ========================================================================

    def init(self):
        self.q_i_struct = _create_struct_from_dict(self.q_i)
        self.q_ij_struct = _create_struct_from_dict(self.q_ij)
        self.q_ji_struct = _create_struct_from_dict(self.q_ji)

        self.var_admm = {}
        for key in ['x_i', 'z_i', 'z_i_p', 'l_i']:
            self.var_admm[key] = self.q_i_struct(0)
        for key in ['x_j', 'z_ij', 'z_ij_p', 'l_ij']:
            self.var_admm[key] = self.q_ij_struct(0)
        for key in ['z_ji', 'l_ji']:
            self.var_admm[key] = self.q_ji_struct(0)

        self.construct_upd_x()
        self.construct_upd_z()
        self.construct_upd_l()

    def construct_upd_x(self):
        # define parameters
        z_i = self.define_parameter('z_i', self.q_i_struct.shape[0])
        z_ji = self.define_parameter('z_ji', self.q_ji_struct.shape[0])
        l_i = self.define_parameter('l_i', self.q_i_struct.shape[0])
        l_ji = self.define_parameter('l_ji', self.q_ji_struct.shape[0])
        rho = self.define_parameter('rho')
        # put them in the struct format
        z_i = self.q_i_struct(z_i)
        z_ji = self.q_ji_struct(z_ji)
        l_i = self.q_i_struct(l_i)
        l_ji = self.q_ji_struct(l_ji)
        # get time info
        t = self.define_symbol('t')
        T = self.define_symbol('T')
        t0 = t/T
        # get (part of) variables
        x_i = self._get_x_variables()
        # transform spline variables: only consider future piece of spline
        tf = shift_knot1_fwd_mx
        self._transform_spline([x_i, z_i, l_i], tf, t0, self.q_i)
        self._transform_spline([z_ji, l_ji], tf, t0, self.q_ji)
        # construct objective
        obj = 0.
        for child, q_i in self.q_i.items():
            for name in q_i.keys():
                x = x_i[child.label][name]
                z = z_i[child.label, name]
                l = l_i[child.label, name]
                obj += mul(l.T, x-z) + 0.5*rho*mul((x-z).T, (x-z))
                for nghb in self.q_ji.keys():
                    z = z_ji[str(nghb), child.label, name]
                    l = l_ji[str(nghb), child.label, name]
                    obj += mul(l.T, x-z) + 0.5*rho*mul((x-z).T, (x-z))
        self.define_objective(obj)
        # construct problem
        self.father = OptiFather(self.group.values())
        options = deepcopy(self.options)
        options['codegen']['buildname'] = (self.options['codegen']['buildname']
                                           + '/admm'+str(self.index)+'/updx')
        prob, compile_time = self.father.construct_problem(options)
        self.problem_upd_x = prob
        self.init_var_admm()

    def construct_upd_z(self):
        # check if we have linear equality constraints
        lineq, A, b = self._check_for_lineq()
        if not lineq:
            raise ValueError(('Awtch, only linear equality interaction '
                              'constraints are allowed for now...'))
        x_i = struct_symMX(self.q_i_struct)
        x_j = struct_symMX(self.q_ji_struct)
        l_i = struct_symMX(self.q_i_struct)
        l_ij = struct_symMX(self.q_ij_struct)
        t = MX.sym('t')
        T = MX.sym('T')
        rho = MX.sym('rho')
        t0 = t/T
        # transform spline variables: only consider future piece of spline
        tf = shift_knot1_fwd_mx
        self._transform_spline([x_i, l_i], tf, t0, self.q_i)
        self._transform_spline(l_ij, tf, t0, self.q_ij)
        self._transform_spline(x_j, tf, t0, self.q_ji)
        # Build KKT system
        E = rho*MX.eye(A.shape[1])
        l, x = vertcat([l_i.cat, l_ij.cat]), vertcat([x_i.cat, x_j.cat])
        f = -(l + rho*x)
        G = vertcat([horzcat([E, A.T]),
                     horzcat([A, MX.zeros(A.shape[0], A.shape[0])])])
        h = vertcat([-f, b])
        z = solve(G, h)
        l_qi = self.q_i_struct.shape[0]
        l_qij = self.q_ij_struct.shape[0]
        z_i_new = self.q_i_struct(z[:l_qi])
        z_ij_new = self.q_ij_struct(z[l_qi:l_qi+l_qij])
        # transform back
        tf = shift_knot1_bwd_mx
        self._transform_spline(z_i_new, tf, t0, self.q_i)
        self._transform_spline(z_ij_new, tf, t0, self.q_ij)
        # create problem
        inp = [x_i, l_i, l_ij, x_j, t, T, rho]
        out = [z_i_new, z_ij_new]
        fun = MXFunction('upd_z', inp, out)
        options = deepcopy(self.options)
        options['codegen']['buildname'] = (self.options['codegen']['buildname']
                                           + '/admm'+str(self.index)+'/updz')
        prob, compile_time = self.father.compile_function(
            fun, 'upd_z', options)
        self.problem_upd_z = prob
        self.update_z(0.0)

    def construct_upd_l(self):
        # create parameters
        x_i = struct_symMX(self.q_i_struct)
        z_i = struct_symMX(self.q_i_struct)
        z_ij = struct_symMX(self.q_ij_struct)
        l_i = struct_symMX(self.q_i_struct)
        l_ij = struct_symMX(self.q_ij_struct)
        x_j = struct_symMX(self.q_ji_struct)
        t = MX.sym('t')
        T = MX.sym('T')
        rho = MX.sym('rho')
        t0 = t/T
        # transform spline variables: only consider future piece of spline
        tf = shift_knot1_fwd_mx
        self._transform_spline([x_i, z_i, l_i], tf, t0, self.q_i)
        self._transform_spline([z_ij, l_ij], tf, t0, self.q_ij)
        self._transform_spline(x_j, tf, t0, self.q_ji)
        # update lambda
        l_i_new = self.q_i_struct(l_i.cat + rho*(x_i.cat - z_i.cat))
        l_ij_new = self.q_ij_struct(l_ij.cat + rho*(x_j.cat - z_ij.cat))
        # transform back
        tf = shift_knot1_bwd_mx
        self._transform_spline(l_i_new, tf, t0, self.q_i)
        self._transform_spline(l_ij_new, tf, t0, self.q_ij)
        # create problem
        inp = [x_i, z_i, z_ij, l_i, l_ij, x_j, t, T, rho]
        out = [l_i_new, l_ij_new]
        fun = MXFunction('upd_l', inp, out)
        options = deepcopy(self.options)
        options['codegen']['buildname'] = (self.options['codegen']['buildname']
                                           + '/admm'+str(self.index)+'/updl')
        prob, compile_time = self.father.compile_function(
            fun, 'upd_l', options)
        self.problem_upd_l = prob

    # ========================================================================
    # Auxiliary methods
    # ========================================================================

    def _get_x_variables(self, **kwargs):
        sol = kwargs['solution'] if 'solution' in kwargs else False
        x = self.q_i_struct(0) if sol else {}
        for child, q_i in self.q_i.items():
            if not sol:
                x[child.label] = {}
            for name, ind in q_i.items():
                var = child.get_variable(name, spline=False, **kwargs)
                if sol:
                    x[child.label, name] = var.ravel()[ind]
                else:
                    x[child.label][name] = var[ind]
        return x

    def _transform_spline(self, var, tf, t0, dic):
        if isinstance(var, list):
            for v in var:
                self._transform_spline(v, tf, t0, dic)
        elif isinstance(dic.keys()[0], ADMM):
            for key in dic.keys():
                if isinstance(var, struct):
                    self._transform_spline(
                        var.prefix[str(key)], tf, t0, dic[key])
                elif isinstance(var, dict):
                    self._transform_spline(var[str(key)], tf, t0, dic[key])
        else:
            for child, q_i in dic.items():
                for name, ind in q_i.items():
                    if name in child._splines:
                        basis = child._splines[name]
                        v1 = SX.sym('v', len(basis))
                        t = SX.sym('t')
                        v2 = tf(v1, basis.knots, basis.degree, t)
                        fun = SXFunction('tf', [v1, t], [v2])
                        for l in range(child._variables[name].shape[1]):
                            sl_min = l*len(basis)
                            sl_max = (l+1)*len(basis)
                            if set(range(sl_min, sl_max)) <= set(ind):
                                sl = slice(sl_min, sl_max)
                                if isinstance(var, dict):
                                    v = var[child.label][name][sl]
                                    # v = tf(v, basis.knots, basis.degree, t0)
                                    v = fun([v, t0])[0]
                                    var[child.label][name][sl] = v
                                else:  # struct or prefix
                                    v = var[child.label, name][sl]
                                    # v = tf(v, basis.knots, basis.degree, t0)
                                    v = fun([v, t0])[0]
                                    var[child.label, name][sl] = v

    def _check_for_lineq(self):
        g = []
        for con in self.constraints:
            lb, ub = con[1], con[2]
            g = vertcat([g, con[0] - lb])
            if not isinstance(lb, np.ndarray):
                lb, ub = [lb], [ub]
            for k in range(len(lb)):
                if lb[k] != ub[k]:
                    return False, None
        sym, jac = [], []
        for child, q_i in self.q_i.items():
            for name, ind in q_i.items():
                var = child.get_variable(name, spline=False)
                jj = jacobian(g, var)
                jac = horzcat([jac, jj[:, ind]])
                sym.append(var)
        for nghb in self.q_ij.keys():
            for child, q_ij in self.q_ij[nghb].items():
                for name, ind in q_ij.items():
                    var = child.get_variable(name, spline=False)
                    jj = jacobian(g, var)
                    jac = horzcat([jac, jj[:, ind]])
                    sym.append(var)
        if len(symvar(jac)) != 0:
            return False, None
        A_fun = MXFunction('A', sym, [jac])
        b_fun = MXFunction('b', sym, [-g])
        sym_num = [np.zeros(s.shape) for s in sym]
        A = A_fun(sym_num)[0].toArray()
        b = b_fun(sym_num)[0].toArray()
        return True, A, b

    # ========================================================================
    # Methods related to solving the problem
    # ========================================================================

    def init_var_admm(self):
        for nghb, q_ji in self.q_ji.items():
            for child, q in q_ji.items():
                for name, ind in q.items():
                    var = self.father._var_result[child.label, name][ind]
                    self.var_admm['z_ji'][str(nghb), child.label, name] = var
        for child, q in self.q_i.items():
            for name, ind in q.items():
                var = self.father._var_result[child.label, name][ind]
                self.var_admm['z_i'][child.label, name] = var

    def set_parameters(self, current_time):
        parameters = {}
        parameters['z_i'] = self.var_admm['z_i'].cat
        parameters['z_ji'] = self.var_admm['z_ji'].cat
        parameters['l_i'] = self.var_admm['l_i'].cat
        parameters['l_ji'] = self.var_admm['l_ji'].cat
        parameters['rho'] = self.options['admm']['rho']
        return parameters

    def update_x(self, current_time):
        # set initial guess, parameters, lb & ub
        var = self.father.get_variables()
        par = self.father.set_parameters(current_time)
        lb, ub = self.father.update_bounds(current_time)
        # solve!
        t0 = time.time()
        self.problem_upd_x({'x0': var, 'p': par, 'lbg': lb, 'ubg': ub})
        t1 = time.time()
        t_upd = t1-t0
        self.father.set_variables(self.problem_upd_x.getOutput('x'))
        self.var_admm['x_i'] = self._get_x_variables(solution=True)
        stats = self.problem_upd_x.getStats()
        if stats.get('return_status') != 'Solve_Succeeded':
            print 'upd_x %d: %s' % (self.index, stats.get('return_status'))
        return t_upd

    def update_z(self, current_time):
        # save previous result
        self.var_admm['z_i_p'] = self.var_admm['z_i']
        self.var_admm['z_ij_p'] = self.var_admm['z_ij']
        # set inputs
        x_i = self.var_admm['x_i']
        l_i = self.var_admm['l_i']
        l_ij = self.var_admm['l_ij']
        x_j = self.var_admm['x_j']
        t = np.round(current_time, 6) % self.problem.knot_time
        T = self.problem.vehicles[0].options['horizon_time']
        rho = self.options['admm']['rho']
        inp = [x_i, l_i, l_ij, x_j, t, T, rho]
        out = self.problem_upd_z(inp)
        self.var_admm['z_i'] = self.q_i_struct(out[0])
        self.var_admm['z_ij'] = self.q_ij_struct(out[1])

    def update_l(self, current_time):
        # set inputs
        x_i = self.var_admm['x_i']
        z_i = self.var_admm['z_i']
        z_ij = self.var_admm['z_ij']
        l_i = self.var_admm['l_i']
        l_ij = self.var_admm['l_ij']
        x_j = self.var_admm['x_j']
        t = np.round(current_time, 6) % self.problem.knot_time
        T = self.problem.vehicles[0].options['horizon_time']
        rho = self.options['admm']['rho']
        inp = [x_i, z_i, z_ij, l_i, l_ij, x_j, t, T, rho]
        out = self.problem_upd_l(inp)
        self.var_admm['l_i'] = self.q_i_struct(out[0])
        self.var_admm['l_ij'] = self.q_ij_struct(out[1])

    def communicate(self):
        for nghb in self.q_ji.keys():
            z_ji = nghb.var_admm['z_ij'].prefix(str(self))
            l_ji = nghb.var_admm['l_ij'].prefix(str(self))
            x_j = nghb.var_admm['x_i'].prefix(str(self))
            self.var_admm['z_ji'][str(nghb)] = z_ji
            self.var_admm['l_ji'][str(nghb)] = l_ji
            self.var_admm['x_j'][str(nghb)] = x_j

    def init_step(self):
        self.problem.init_step()

    def get_residuals(self):
        x_i = self.var_admm['x_i'].cat
        z_i = self.var_admm['z_i'].cat
        x_j = self.var_admm['x_j'].cat
        z_ij = self.var_admm['z_ij'].cat
        z_i_p = self.var_admm['z_i_p'].cat
        z_ij_p = self.var_admm['z_ij_p'].cat
        rho = self.options['admm']['rho']

        pr = np.norm(x_i-z_i)**2 + np.norm(x_j-z_ij)**2
        dr = rho*(np.norm(z_i-z_i_p)**2 + np.norm(z_ij-z_ij_p)**2)
        return pr, dr


class ADMMProblem(DistributedProblem):

    def __init__(self, problems, options):
        DistributedProblem.__init__(self, problems, ADMM, options)

    # ========================================================================
    # ADMM options
    # ========================================================================

    def set_default_options(self):
        Problem.set_default_options(self)
        self.options['admm'] = {'max_iter': 1, 'rho': 0.1}

    # ========================================================================
    # Perform ADMM sequence
    # ========================================================================

    def solve(self, current_time):
        it0 = self.iteration
        while (self.iteration - it0) < self.options['admm']['max_iter']:
            t_upd_x, t_upd_z, t_upd_l, t_res = 0., 0., 0., 0.
            p_res, d_res = 0., 0.
            for updater in self.updaters:
                self.init_step()
            for updater in self.updaters:
                t = updater.update_x(current_time)
                t_upd_x = max(t_upd_x, t)
            for updater in self.updaters:
                updater.communicate()
            for updater in self.updaters:
                t1 = updater.update_z(current_time)
                t2 = updater.update_l(current_time)
                t3, pr, dr = updater.get_residuals()
                t_upd_z = max(t_upd_z, t1)
                t_upd_l = max(t_upd_l, t2)
                t_res = max(t_res, t3)
                p_res += pr**2
                d_res += dr**2
            p_res, d_res = np.sqrt(p_res), np.sqrt(d_res)
            for updater in self.updaters:
                updater.communicate()
            if self.options['verbose'] >= 1:
                self.iteration += 1
                if ((self.iteration - 1) % 20 == 0):
                    print('----|----------------|------------|------------|'
                          '------------|------------|------------')
                    print('%3s | %14s | %10s | %10s | %10s | %10s | %10s ' %
                          ('It', 'time', 'prim res', 'dual res',
                           't upd_x', 't upd_z', 't upd_l', 't_res'))
                    print('----|----------------|------------|------------|'
                          '------------|------------|------------')
                print('%3d | %.1e | %.4e | %.4e | %.4e | %.4e | %.4e ' %
                      (self.iteration, current_time, p_res, d_res, t_upd_x,
                       t_upd_z, t_upd_l, t_res))
