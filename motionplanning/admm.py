from optilayer import OptiChild, OptiFather
from problem import Problem
from point2point import Point2point
from casadi import symvar, vertcat, mul, MXFunction, sumRows
from casadi.tools import struct, struct_MX, entry
from spline_extra import shift_knot1_fwd_mx
from copy import deepcopy
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

    # def set_default_options(self):
    #     self.options = {'rho': 0.1}

    # def set_options(self, options):
    #     self.options.update(options)

    def construct_problem(self, q_i, q_ij, q_ji, constraints):
        self.construct_structs(q_i, q_ij, q_ji)
        self.construct_updX()

    def construct_structs(self, q_i, q_ij, q_ji):
        self.q_i = q_i
        self.q_ij = q_ij
        self.q_ji = q_ji

        self.q_i_struct = create_struct_from_dict(self.q_i)
        self.q_ij_struct = create_struct_from_dict(self.q_ij)
        self.q_ji_struct = create_struct_from_dict(self.q_ji)

    def get_x_variables(self):
        x = {}
        for child, q_i in self.q_i.items():
            x[child.label] = {}
            for name, ind in q_i.items():
                x[child.label][name] = child.get_variable(name, spline=False)[ind]
        return x

    def transform_spline_variables(self, x, z, l, tf, t0):
        for child, q_i in self.q_i.items():
            for name, ind in q_i.items():
                if name in child._splines:
                    basis = child._splines[name]
                    for l in range(child._variables[name].shape[1]):
                        if set(range(l*len(basis), (l+1)*len(basis))) <= set(ind):
                            sl = slice(l*len(basis), (l+1)*len(basis))
                            x[child.label][name][sl] = tf(x[child.label][name][sl], basis.knots, basis.degree, t0)
                            z[child.label, name][sl] = tf(z[child.label, name][sl], basis.knots, basis.degree, t0)
                            l[child.label, name][sl] = tf(l[child.label, name][sl], basis.knots, basis.degree, t0)



    def construct_updX(self):
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
        for child, q_i in self.q_i.items():
            for name, ind in q_i.items():
                if name in child._splines:
                    basis = child._splines[name]
                    for l in range(child._variables[name].shape[1]):
                        if set(range(l*len(basis), (l+1)*len(basis))) <= set(ind):
                            sl = slice(l*len(basis), (l+1)*len(basis))
                            x_i[child.label][name][sl] = shift_knot1_fwd_mx(x_i[child.label][name][sl], basis.knots, basis.degree, t0)
                            z_i[child.label, name][sl] = shift_knot1_fwd_mx(z_i[child.label, name][sl], basis.knots, basis.degree, t0)
                            l_i[child.label, name][sl] = shift_knot1_fwd_mx(l_i[child.label, name][sl], basis.knots, basis.degree, t0)
        for nghb_ind in self.q_ji.keys():
            for child, q_ji in self.q_ji[nghb_ind].items():
                for name, ind in q_ji.items():
                    if name in child._splines:
                        basis = child._splines[name]
                        for l in range(child._variables[name].shape[1]):
                            if set(range(l*len(basis), (l+1)*len(basis))) <= set(ind):
                                sl = slice(l*len(basis), (l+1)*len(basis))
                                z_ji[str(nghb_ind), child.label, name][sl] = shift_knot1_fwd_mx(z_ji[str(nghb_ind), child.label, name][sl], basis.knots, basis.degree, t0)
                                l_ji[str(nghb_ind), child.label, name][sl] = shift_knot1_fwd_mx(l_ji[str(nghb_ind), child.label, name][sl], basis.knots, basis.degree, t0)
        # construct objective
        obj = 0.
        for child, q_i in self.q_i.items():
            for name in q_i.keys():
                x = x_i[child.label][name]
                z = z_i[child.label, name]
                l = l_i[child.label, name]
                obj += mul(l.T, x-z) + 0.5*rho*mul((x-z).T, (x-z))
                for n_i in self.q_ji.keys():
                    z = z_ji[str(n_i), child.label, name]
                    l = l_ji[str(n_i), child.label, name]
                    obj += mul(l.T, x-z) + 0.5*rho*mul((x-z).T, (x-z))
        self.define_objective(obj)
        # construct problem
        self.father = OptiFather(self.group.values())
        options = deepcopy(self.options)
        options['codegen']['buildname'] = self.options['codegen']['buildname'] + '_updx_' + str(self.index)
        self.problem_updX, compile_time = self.father.construct_problem(options)


class DistributedProblem(Problem):

    def __init__(self, fleet, environment, options):
        Problem.__init__(self, fleet, environment, options)

    def construct_subproblems(self, problems):
        self.problems = problems
        if len(problems) != len(self.vehicles):
            raise ValueError('Number of vehicles and problems do not match!')
        self.updaters = {}
        for index, vehicle in enumerate(self.vehicles):
            self.updaters[index] = ADMM(index, vehicle, problems[index],
                                        self.environment.copy(), self.options)
        q_i, q_ij, q_ji, con = self.interprete_constraints(self.updaters)

        for index, updater in self.updaters.items():
            updater.construct_problem(q_i[index], q_ij[index],
                                      q_ji[index], con[index])

    def compose_dictionary(self):
        children = self.vehicles
        children.extend(self.problems)
        children.append(self.environment)
        symbol_dict = {}
        for child in children:
            symbol_dict.update(child.symbol_dict)
        return symbol_dict

    def interprete_constraints(self, groups):
        q_i = [{} for group in groups]
        q_ij = [{} for group in groups]
        q_ji = [{} for group in groups]
        con = [[] for group in groups]

        symbol_dict = self.compose_dictionary()

        for constraint in self._constraints.values():
            dep = {}
            for sym, indices in get_dependency(constraint[0]).items():
                child, name = symbol_dict[sym.getName()]
                if child not in dep:
                    dep[child] = {}
                dep[child][name] = indices
            for child, dic in dep.items():
                # create q_i: structure of local variables
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
                # create q_ij: structure of remote variables j seen from local i
                for ch, dic_ch in dep.items():
                    if not (child == ch):
                        if ch.index not in q_ij[index]:
                            q_ij[index][ch.index] = {}
                        if ch not in q_ij[index][ch.index]:
                            q_ij[index][ch.index][ch] = {}
                        for name, indices in dic_ch.items():
                            if name not in q_ij[index][ch.index][ch]:
                                q_ij[index][ch.index][ch][name] = []
                            for i in indices:
                                if i not in q_ij[index][ch.index][ch][name]:
                                    q_ij[index][ch.index][ch][name].append(i)
                            q_ij[index][ch.index][ch][name].sort()
        # create q_ji: structure of local variables i seen from remote j
        for index_i in range(len(groups)):
            for index_j in q_ij[index_i].keys():
                if index_j not in q_ji:
                    q_ji[index_j] = {}
                q_ji[index_j][index_i] = q_ij[index_i][index_j]

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
        Dx = [0.5, 0]
        for k in range(self.vehicles[0].n_y):
            self.define_constraint(y[1][k] - y[0][k] - Dx[k], 0., 0.)

        self.construct_subproblems(problems)
