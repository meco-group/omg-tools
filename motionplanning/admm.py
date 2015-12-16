from optilayer import OptiFather
from problem import Problem
from point2point import Point2point
from casadi import symvar, mul, SX, MX, SXFunction, MXFunction, sumRows
from casadi.tools import struct, struct_symMX, entry
from spline_extra import shift_knot1_fwd_mx, shift_knot1_bwd_mx
from copy import deepcopy
import numpy as np
import time


def get_dependency(expression):
    t0 = time.time()
    sym = symvar(expression)
    f = MXFunction('f', sym, [expression])
    dep = {}
    for index, sym in enumerate(sym):
        J = f.jacSparsity(index, 0)
        dep[sym] = sorted(sumRows(J).find())
    t1 = time.time()
    print(t1-t0)*1000
    return dep


def create_struct_from_dict(dictionary):
    entries = []
    for key, data in dictionary.items():
        if isinstance(data, dict):
            stru = create_struct_from_dict(data)
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

    def set_default_options(self):
        Problem.set_default_options(self)
        self.options['admm'] = {'rho': 0.1}

    def set_options(self, options):
        Problem.set_options(self, options)
        self.options.update(options)

    def construct_problem(self, q_i, q_ij, q_ji, constraints):
        self.construct_structs(q_i, q_ij, q_ji)
        self.construct_upd_x()
        self.construct_upd_z()
        self.construct_upd_l()

    def construct_structs(self, q_i, q_ij, q_ji):
        self.q_i = q_i
        self.q_ij = q_ij
        self.q_ji = q_ji

        self.q_i_struct = create_struct_from_dict(self.q_i)
        self.q_ij_struct = create_struct_from_dict(self.q_ij)
        self.q_ji_struct = create_struct_from_dict(self.q_ji)

        self.var_admm = {}
        self.var_admm['x_i'] = self.q_i_struct(0)
        self.var_admm['x_j'] = self.q_ij_struct(0)
        self.var_admm['z_i'] = self.q_i_struct(0)
        self.var_admm['z_ij'] = self.q_ij_struct(0)
        self.var_admm['z_ji'] = self.q_ji_struct(0)
        self.var_admm['l_i'] = self.q_i_struct(0)
        self.var_admm['l_ij'] = self.q_ij_struct(0)
        self.var_admm['l_ji'] = self.q_ji_struct(0)

    def get_x_variables(self, **kwargs):
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

    def transform_spline(self, var, tf, t0, dic):
        if isinstance(var, list):
            for v in var:
                self.transform_spline(v, tf, t0, dic)
        elif isinstance(dic.keys()[0], ADMM):
            for key in dic.keys():
                if isinstance(var, struct):
                    self.transform_spline(
                        var.prefix[str(key)], tf, t0, dic[key])
                elif isinstance(var, dict):
                    self.transform_spline(var[str(key)], tf, t0, dic[key])
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
        x_i = self.get_x_variables()
        # transform spline variables: only consider future piece of spline
        tf = shift_knot1_fwd_mx
        self.transform_spline([x_i, z_i, l_i], tf, t0, self.q_i)
        self.transform_spline([z_ji, l_ji], tf, t0, self.q_ji)
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

    def construct_upd_z(self):
        pass

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
        self.transform_spline([x_i, z_i, l_i], tf, t0, self.q_i)
        self.transform_spline([z_ij, l_ij], tf, t0, self.q_ij)
        self.transform_spline(x_j, tf, t0, self.q_ji)
        # update lambda
        l_i_new = self.q_i_struct(l_i.cat + rho*(x_i.cat - z_i.cat))
        l_ij_new = self.q_ij_struct(l_ij.cat + rho*(x_j.cat - z_ij.cat))
        # transform back
        tf = shift_knot1_bwd_mx
        self.transform_spline(l_i_new, tf, t0, self.q_i)
        self.transform_spline(l_ij_new, tf, t0, self.q_ij)
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
        self.update_l(0.0)

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
        self.var_admm['x_i'] = self.get_x_variables(solution=True)
        stats = self.problem_upd_x.getStats()
        if stats.get('return_status') != 'Solve_Succeeded':
            print 'upd_x %d: %s' % (self.index, stats.get('return_status'))
        return t_upd

    def update_z(self, current_time):
        pass

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
        pass


class DistributedProblem(Problem):

    def __init__(self, fleet, environment, options):
        Problem.__init__(self, fleet, environment, options)

    def construct_subproblems(self, problems):
        self.problems = problems
        if len(problems) != len(self.vehicles):
            raise ValueError('Number of vehicles and problems do not match!')
        self.updaters = []
        for index, vehicle in enumerate(self.vehicles):
            admm = ADMM(index, vehicle, problems[index],
                        self.environment.copy(), self.options)
            self.updaters.append(admm)
        q_i, q_ij, q_ji, con = self.interprete_constraints(self.updaters)

        for ind, upd in enumerate(self.updaters):
            upd.construct_problem(q_i[ind], q_ij[ind], q_ji[ind], con[ind])

    def solve(self, current_time):
        it0 = self.iteration
        while (self.iteration - it0) < self.max_iterations:
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

    def compose_dictionary(self):
        children = self.vehicles
        children.extend(self.problems)
        children.append(self.environment)
        symbol_dict = {}
        for child in children:
            symbol_dict.update(child.symbol_dict)
        return symbol_dict

    def interprete_constraints(self, updaters):
        q_i = [{} for updater in updaters]
        q_ij = [{} for updater in updaters]
        q_ji = [{} for updater in updaters]
        con = [[] for updater in updaters]

        symbol_dict = self.compose_dictionary()

        for constraint in self._constraints.values():
            dep = {}
            for sym, indices in get_dependency(constraint[0]).items():
                child, name = symbol_dict[sym.getName()]
                if child not in dep:
                    dep[child] = {}
                dep[child][name] = indices
            for child, dic in dep.items():
                # q_i: structure of local variables i
                index = child.index
                con[index].append(constraint)
                if child not in q_i[index]:
                    q_i[index][child] = {}
                for name, indices in dic.items():
                    if name not in q_i[index][child]:
                        q_i[index][child][name] = []
                    for i in indices:
                        if i not in q_i[index][child][name]:
                            q_i[index][child][name].append(i)
                    q_i[index][child][name].sort()
                # q_ij: structure of remote variables j seen from local i
                for ch, dic_ch in dep.items():
                    if not (child == ch):
                        upd = updaters[ch.index]
                        if upd not in q_ij[index]:
                            q_ij[index][upd] = {}
                        if ch not in q_ij[index][upd]:
                            q_ij[index][upd][ch] = {}
                        for name, indices in dic_ch.items():
                            if name not in q_ij[index][upd][ch]:
                                q_ij[index][upd][ch][name] = []
                            for i in indices:
                                if i not in q_ij[index][upd][ch][name]:
                                    q_ij[index][upd][ch][name].append(i)
                            q_ij[index][upd][ch][name].sort()
        # q_ji: structure of local variables i seen from remote j
        for upd1 in updaters:
            for upd2 in q_ij[upd1.index].keys():
                q_ji[upd2.index][upd1] = q_ij[upd1.index][upd2]

        return q_i, q_ij, q_ji, con


class FormationPoint2point(DistributedProblem):

    def __init__(self, fleet, environment, options={}):
        DistributedProblem.__init__(self, fleet, environment, options)

        problems = [Point2point(vehicle, environment, options)
                    for vehicle in self.vehicles]
        y = [vehicle.get_variable('y') for vehicle in self.vehicles]

        # self.define_constraint(
        #     y[1][0].coeffs[0] - y[0][0].coeffs[0] - 0.5, 0., 0.)
        # self.define_constraint(
        #     y[1][0].coeffs[3:5] - y[0][0].coeffs[5:7], 0., 0.)
        Dx = [0., 0]
        for k in range(self.vehicles[0].n_y):
            self.define_constraint(y[1][k] - y[0][k] - Dx[k], 0., 0.)
            self.define_constraint(y[2][k] - y[1][k] - Dx[k], 0., 0.)
            self.define_constraint(y[3][k] - y[2][k] - Dx[k], 0., 0.)
            self.define_constraint(y[0][k] - y[3][k] - Dx[k], 0., 0.)

        self.construct_subproblems(problems)
