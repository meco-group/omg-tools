from optilayer import OptiFather
from problem import Problem
from distributedproblem import DistributedProblem
from casadi import symvar, mul, SX, MX, DMatrix, MXFunction, reshape
from casadi import vertcat, horzcat, jacobian, solve, substitute
from casadi.tools import struct, struct_symMX, struct_symSX, entry, structure
from spline_extra import shift_knot1_fwd_mx, shift_knot1_bwd_mx, shift_over_knot
import numpy as np
import numpy.linalg as la
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
        tf = lambda cfs, knots, deg: shift_knot1_fwd_mx(cfs, knots, deg, t0)
        self._transform_spline([x_i, z_i, l_i], tf, self.q_i)
        self._transform_spline([z_ji, l_ji], tf, self.q_ji)
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
        prob, compile_time = self.father.construct_problem(self.options)
        self.problem_upd_x = prob
        self.init_var_admm()

    def construct_upd_z(self):
        # check if we have linear equality constraints
        self._lineq_updz, A, b = self._check_for_lineq()
        if not self._lineq_updz:
            self._construct_upd_z_nlp()
        x_i = struct_symSX(self.q_i_struct)
        x_j = struct_symSX(self.q_ij_struct)
        l_i = struct_symSX(self.q_i_struct)
        l_ij = struct_symSX(self.q_ij_struct)
        t = SX.sym('t')
        T = SX.sym('T')
        rho = SX.sym('rho')
        inp = [x_i.cat, l_i.cat, l_ij.cat, x_j.cat, t, T, rho]
        t0 = t/T
        # put symbols in SX structs (necessary for transformation)
        x_i = self.q_i_struct(x_i)
        x_j = self.q_ij_struct(x_j)
        l_i = self.q_i_struct(l_i)
        l_ij = self.q_ij_struct(l_ij)
        # transform spline variables: only consider future piece of spline
        tf = lambda cfs, knots, deg: shift_knot1_fwd_mx(cfs, knots, deg, t0)
        self._transform_spline([x_i, l_i], tf, self.q_i)
        self._transform_spline([x_j, l_ij], tf, self.q_ij)
        # Build KKT system
        E = rho*SX.eye(A.shape[1])
        l, x = vertcat([l_i.cat, l_ij.cat]), vertcat([x_i.cat, x_j.cat])
        f = -(l + rho*x)
        G = vertcat([horzcat([E, A.T]),
                     horzcat([A, SX.zeros(A.shape[0], A.shape[0])])])
        h = vertcat([-f, b])
        z = solve(G, h)
        l_qi = self.q_i_struct.shape[0]
        l_qij = self.q_ij_struct.shape[0]
        z_i_new = self.q_i_struct(z[:l_qi])
        z_ij_new = self.q_ij_struct(z[l_qi:l_qi+l_qij])
        # transform back
        tf = lambda cfs, knots, deg: shift_knot1_bwd_mx(cfs, knots, deg, t0)
        self._transform_spline(z_i_new, tf, self.q_i)
        self._transform_spline(z_ij_new, tf, self.q_ij)
        out = [z_i_new.cat, z_ij_new.cat]
        # create problem
        prob, compile_time = self.father.create_function(
            'upd_z', inp, out, self.options)
        self.problem_upd_z = prob

    def _construct_upd_z_nlp(self):
        # construct variables
        self._var_struct_updz = struct([entry('z_i', struct=self.q_i_struct),
                                        entry('z_ij', struct=self.q_ij_struct)])
        var = struct_symMX(self._var_struct_updz)
        z_i = self.q_i_struct(var['z_i'])
        z_ij = self.q_ij_struct(var['z_ij'])
        # construct parameters
        self._par_struct_updz = struct([entry('x_i', struct=self.q_i_struct),
                                        entry('x_j', struct=self.q_ij_struct),
                                        entry('l_i', struct=self.q_i_struct),
                                        entry('l_ij', struct=self.q_ij_struct),
                                        entry('t'), entry('T'), entry('rho')])
        par = struct_symMX(self._par_struct_updz)
        x_i, x_j = self.q_i_struct(par['x_i']), self.q_ij_struct(par['x_j'])
        l_i, l_ij = self.q_i_struct(par['l_i']), self.q_ij_struct(par['l_ij'])
        t, T, rho = par['t'], par['T'], par['rho']
        t0 = t/T
        # transform spline variables: only consider future piece of spline
        tf = lambda cfs, knots, deg: shift_knot1_fwd_mx(cfs, knots, deg, t0)
        self._transform_spline([x_i, z_i, l_i], tf, self.q_i)
        self._transform_spline([x_j, z_ij, l_ij], tf, self.q_ij)
        # construct constraints
        constraints, lb, ub = [], [], []
        for con in self.constraints:
            c = con[0]
            for sym in symvar(c):
                for label, child in self.group.items():
                    if sym.getName() in child.symbol_dict:
                        name = child.symbol_dict[sym.getName()][1]
                        v = z_i[label, name]
                        ind = self.q_i[child][name]
                        sym2 = MX.zeros(sym.size())
                        sym2[ind] = v
                        sym2 = reshape(sym2, sym.shape)
                        c = substitute(c, sym, sym2)
                        break
                for nghb in self.q_ij.keys():
                    for label, child in nghb.group.items():
                        if sym.getName() in child.symbol_dict:
                            name = child.symbol_dict[sym.getName()][1]
                            v = z_ij[nghb.label, label, name]
                            ind = self.q_ij[nghb][child][name]
                            sym2 = MX.zeros(sym.size())
                            sym2[ind] = v
                            sym2 = reshape(sym2, sym.shape)
                            c = substitute(c, sym, sym2)
                            break
            constraints.append(c)
            lb.append(con[1])
            ub.append(con[2])
        self.lb_updz, self.ub_updz = lb, ub
        # construct objective
        obj = 0.
        for child, q_i in self.q_i.items():
            for name in q_i.keys():
                x = x_i[child.label, name]
                z = z_i[child.label, name]
                l = l_i[child.label, name]
                obj += mul(l.T, x-z) + 0.5*rho*mul((x-z).T, (x-z))
        for nghb in self.q_ij.keys():
            for child, q_ij in self.q_ij[nghb].items():
                for name in q_ij.keys():
                    x = x_j[str(nghb), child.label, name]
                    z = z_ij[str(nghb), child.label, name]
                    l = l_ij[str(nghb), child.label, name]
                    obj += mul(l.T, x-z) + 0.5*rho*mul((x-z).T, (x-z))
        # construct problem
        prob, compile_time = self.father.create_nlp(var, par, obj,
                                                    constraints, self.options)
        self.problem_upd_z = prob

    def construct_upd_l(self):
        # create parameters
        x_i = struct_symSX(self.q_i_struct)
        z_i = struct_symSX(self.q_i_struct)
        z_ij = struct_symSX(self.q_ij_struct)
        l_i = struct_symSX(self.q_i_struct)
        l_ij = struct_symSX(self.q_ij_struct)
        x_j = struct_symSX(self.q_ij_struct)
        t = SX.sym('t')
        T = SX.sym('T')
        rho = SX.sym('rho')
        t0 = t/T
        inp = [x_i, z_i, z_ij, l_i, l_ij, x_j, t, T, rho]
        # put symbols in SX structs (necessary for transformation)
        x_i = self.q_i_struct(x_i)
        z_i = self.q_i_struct(z_i)
        z_ij = self.q_ij_struct(z_ij)
        l_i = self.q_i_struct(l_i)
        l_ij = self.q_ij_struct(l_ij)
        x_j = self.q_ij_struct(x_j)
        # transform spline variables: only consider future piece of spline
        tf = lambda cfs, knots, deg: shift_knot1_fwd_mx(cfs, knots, deg, t0)
        self._transform_spline([x_i, z_i, l_i], tf, self.q_i)
        self._transform_spline([x_j, z_ij, l_ij], tf, self.q_ij)
        # update lambda
        l_i_new = self.q_i_struct(l_i.cat + rho*(x_i.cat - z_i.cat))
        l_ij_new = self.q_ij_struct(l_ij.cat + rho*(x_j.cat - z_ij.cat))
        # transform back
        tf = lambda cfs, knots, deg: shift_knot1_bwd_mx(cfs, knots, deg, t0)
        # tf = shift_knot1_bwd_mx
        self._transform_spline(l_i_new, tf, self.q_i)
        self._transform_spline(l_ij_new, tf, self.q_ij)
        out = [l_i_new, l_ij_new]
        # create problem
        prob, compile_time = self.father.create_function(
            'upd_l', inp, out, self.options)
        self.problem_upd_l = prob

    # ========================================================================
    # Auxiliary methods
    # ========================================================================

    def _struct2dict(self, var, dic):
        if isinstance(var, list):
            return [self._struct2dict(v, dic) for v in var]
        elif isinstance(dic.keys()[0], ADMM):
            ret = {}
            for nghb in dic.keys():
                ret[nghb.label] = {}
                for child, q in dic[nghb].items():
                    ret[nghb.label][child.label] = {}
                    for name in q.keys():
                        ret[nghb.label][child.label][name] = var[
                            nghb.label, child.label, name]
            return ret
        else:
            ret = {}
            for child, q in dic.items():
                ret[child.label] = {}
                for name in q.keys():
                    ret[child.label][name] = var[child.label, name]
            return ret

    def _dict2struct(self, var, stru):
        if isinstance(var, list):
            return [self._dict2struct(v, stru) for v in var]
        elif 'admm' in var.keys()[0]:
            chck = var.values()[0].values()[0].values()[0]
            if isinstance(chck, SX):
                ret = structure.SXStruct(stru)
            elif isinstance(chck, MX):
                ret = structure.MXStruct(stru)
            elif isinstance(chck, DMatrix):
                ret = stru(0)
            for nghb in var.keys():
                for child, q in var[nghb].items():
                    for name in q.keys():
                        ret[nghb, child, name] = var[nghb][child][name]
            return ret
        else:
            chck = var.values()[0].values()[0]
            if isinstance(chck, SX):
                ret = structure.SXStruct(stru)
            elif isinstance(chck, MX):
                ret = structure.MXStruct(stru)
            elif isinstance(chck, DMatrix):
                ret = stru(0)
            for child, q in var.items():
                for name in q.keys():
                    ret[child, name] = var[child][name]
            return ret

    def _get_x_variables(self, **kwargs):
        sol = kwargs['solution'] if 'solution' in kwargs else False
        x = self.q_i_struct(0) if sol else {}
        for child, q_i in self.q_i.items():
            if not sol:
                x[child.label] = {}
            for name, ind in q_i.items():
                var = child.get_variable(name, spline=False, **kwargs)
                if sol:
                    x[child.label, name] = var.T.ravel()[ind]
                else:
                    x[child.label][name] = var[ind]
        return x

    def _transform_spline(self, var, tf, dic):
        if isinstance(var, list):
            return [self._transform_spline(v, tf, dic) for v in var]
        elif isinstance(var, struct):
            var = self._struct2dict(var, dic)
            var = self._transform_spline(var, tf, dic)
            return self._dict2struct(var, _create_struct_from_dict(dic))
        elif isinstance(dic.keys()[0], ADMM):
            ret = {}
            for nghb in dic.keys():
                ret[nghb.label] = self._transform_spline(
                    var[nghb.label], tf, dic[nghb])
            return ret
        else:
            for child, q_i in dic.items():
                for name, ind in q_i.items():
                    if name in child._splines:
                        basis = child._splines[name]
                        for l in range(child._variables[name].shape[1]):
                            sl_min = l*len(basis)
                            sl_max = (l+1)*len(basis)
                            if set(range(sl_min, sl_max)) <= set(ind):
                                sl = slice(sl_min, sl_max)
                                v = var[child.label][name][sl]
                                v = tf(v, basis.knots, basis.degree)
                                var[child.label][name][sl] = v
            return var

    def _check_for_lineq(self):
        g = []
        for con in self.constraints:
            lb, ub = con[1], con[2]
            g = vertcat([g, con[0] - lb])
            if not isinstance(lb, np.ndarray):
                lb, ub = [lb], [ub]
            for k in range(len(lb)):
                if lb[k] != ub[k]:
                    return False, None, None
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
            return False, None, None
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
        self.current_time = current_time
        self.problem.current_time = current_time
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
        if (stats.get('return_status') != 'Solve_Succeeded'):
            print 'upd_x %d: %s' % (self.index, stats.get('return_status'))
        return t_upd

    def update_z(self, current_time):
        # save previous result
        t0 = time.time()
        self.var_admm['z_i_p'] = self.var_admm['z_i']
        self.var_admm['z_ij_p'] = self.var_admm['z_ij']
        current_time = np.round(current_time, 6) % self.problem.knot_time
        horizon_time = self.problem.vehicles[0].options['horizon_time']
        rho = self.options['admm']['rho']
        if self._lineq_updz:
            # set inputs
            x_i = self.var_admm['x_i']
            l_i = self.var_admm['l_i']
            l_ij = self.var_admm['l_ij']
            x_j = self.var_admm['x_j']
            t = current_time
            T = horizon_time
            inp = [x_i, l_i, l_ij, x_j, t, T, rho]
            out = self.problem_upd_z(inp)
            z_i, z_ij = out[0], out[1]
        else:
            # set parameters
            par = self._par_struct_updz(0)
            par['x_i'] = self.var_admm['x_i']
            par['l_i'] = self.var_admm['l_i']
            par['l_ij'] = self.var_admm['l_ij']
            par['x_j'] = self.var_admm['x_j']
            par['t'] = current_time
            par['T'] = horizon_time
            par['rho'] = rho
            lb, ub = self.lb_updz, self.ub_updz
            self.problem_upd_z({'p': par, 'lbg': lb, 'ubg': ub})
            out = self.problem_upd_z.getOutput('x')
            out = self._var_struct_updz(out)
            z_i, z_ij = out['z_i'], out['z_ij']
        self.var_admm['z_i'] = self.q_i_struct(z_i)
        self.var_admm['z_ij'] = self.q_ij_struct(z_ij)
        t1 = time.time()
        return t1-t0

    def update_l(self, current_time):
        t0 = time.time()
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
        t1 = time.time()
        return t1-t0

    def communicate(self):
        for nghb in self.q_ji.keys():
            z_ji = nghb.var_admm['z_ij'].prefix[str(self)]
            l_ji = nghb.var_admm['l_ij'].prefix[str(self)]
            x_j = nghb.var_admm['x_i']
            self.var_admm['z_ji'][str(nghb)] = z_ji
            self.var_admm['l_ji'][str(nghb)] = l_ji
            self.var_admm['x_j'][str(nghb)] = x_j

    def init_step(self, current_time):
        self.problem.init_step(current_time)
        # transform spline variables
        if ((current_time > 0. and
             np.round(current_time, 6) % self.problem.knot_time == 0)):
            tf = lambda cfs, knots, deg: shift_over_knot(cfs, knots, deg, 1)
            for key in ['x_i', 'z_i', 'z_i_p', 'l_i']:
                self.var_admm[key] = self._transform_spline(
                    self.var_admm[key], tf, self.q_i)
            for key in ['x_j', 'z_ij', 'z_ij_p', 'l_ij']:
                self.var_admm[key] = self._transform_spline(
                    self.var_admm[key], tf, self.q_ij)
            for key in ['z_ji', 'l_ji']:
                self.var_admm[key] = self._transform_spline(
                    self.var_admm[key], tf, self.q_ji)

    def get_residuals(self):
        t0 = time.time()
        x_i = self.var_admm['x_i'].cat
        z_i = self.var_admm['z_i'].cat
        x_j = self.var_admm['x_j'].cat
        z_ij = self.var_admm['z_ij'].cat
        z_i_p = self.var_admm['z_i_p'].cat
        z_ij_p = self.var_admm['z_ij_p'].cat
        rho = self.options['admm']['rho']

        pr = la.norm(x_i-z_i)**2 + la.norm(x_j-z_ij)**2
        dr = rho*(la.norm(z_i-z_i_p)**2 + la.norm(z_ij-z_ij_p)**2)
        t1 = time.time()
        return t1-t0, pr, dr


class ADMMProblem(DistributedProblem):

    def __init__(self, problems, options):
        DistributedProblem.__init__(self, problems, ADMM, options)

    # ========================================================================
    # ADMM options
    # ========================================================================

    def set_default_options(self):
        Problem.set_default_options(self)
        self.options['admm'] = {'max_iter': 1, 'rho': 2., 'init': 5}

    def set_options(self, options):
        if 'admm' in options:
            self.options['admm'].update(options.pop('admm'))
        Problem.set_options(self, options)

    # ========================================================================
    # Perform ADMM sequence
    # ========================================================================

    def initialize(self):
        for k in range(self.options['admm']['init']):
            self.solve(0.0)

    def solve(self, current_time):
        self.current_time = current_time
        it0 = self.iteration
        while (self.iteration - it0) < self.options['admm']['max_iter']:
            t_upd_x, t_upd_z, t_upd_l, t_res = 0., 0., 0., 0.
            p_res, d_res = 0., 0.
            for updater in self.updaters:
                updater.init_step(current_time)
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
                    print('----|------|----------|----------|'
                          '----------|----------|----------|----------')
                    print('%3s | %4s | %8s | %8s | %8s | %8s | %8s | %8s ' %
                          ('It', 't', 'prim res', 'dual res',
                           't upd_x', 't upd_z', 't upd_l', 't_res'))
                    print('----|------|----------|----------|'
                          '----------|----------|----------|----------')
                print('%3d | %4.1f | %.2e | %.2e | %.2e | %.2e | %.2e | %.2e ' %
                      (self.iteration, current_time, p_res, d_res, t_upd_x,
                       t_upd_z, t_upd_l, t_res))
            self.update_times.append(t_upd_x + t_upd_z + t_upd_l + t_res)
