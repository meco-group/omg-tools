# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from ..basics.optilayer import OptiFather, create_nlp, create_function
from ..basics.spline_extra import shift_knot1_fwd, shift_knot1_bwd, shift_over_knot
from problem import Problem
from dualmethod import DualUpdater, DualProblem
from casadi import symvar, mtimes, MX, Function, reshape
from casadi import vertcat, horzcat, jacobian, solve, substitute
from casadi.tools import struct, struct_symMX, entry
import numpy as np
import numpy.linalg as la
import warnings
import time


class ADMM(DualUpdater):

    def __init__(self, index, vehicle, problem, environment, distr_problem,
                 options=None):
        DualUpdater.__init__(self, index, vehicle, problem, environment,
                             distr_problem, 'admm', options)

    # ========================================================================
    # Create problem
    # ========================================================================

    def init(self, problems=None):
        if problems is None:
            problems = {'upd_x': None, 'upd_z': None, 'lineq_updz': True, 'upd_l': None}
        DualUpdater.init(self)
        self.var_admm = {}
        for key in ['x_i', 'z_i', 'z_i_p', 'l_i', 'l_i_p']:
            self.var_admm[key] = self.q_i_struct(0)
        for key in ['x_j', 'z_ij', 'z_ij_p', 'l_ij', 'l_ij_p']:
            self.var_admm[key] = self.q_ij_struct(0)
        for key in ['z_ji', 'l_ji']:
            self.var_admm[key] = self.q_ji_struct(0)
        self.construct_upd_x(problems['upd_x'])
        self.construct_upd_z(problems['upd_z'], problems['lineq_updz'])
        self.construct_upd_l(problems['upd_l'])
        return {'upd_x': self.problem_upd_x, 'upd_z': self.problem_upd_z,
                'lineq_updz': self._lineq_updz, 'upd_l': self.problem_upd_l}

    def construct_upd_x(self, problem=None):
        # construct optifather & give reference to problem
        self.father_updx = OptiFather(self.group.values())
        self.problem.father = self.father_updx
        # define parameters
        z_i = self.define_parameter('z_i', self.q_i_struct.shape[0])
        z_ji = self.define_parameter('z_ji', self.q_ji_struct.shape[0])
        l_i = self.define_parameter('l_i', self.q_i_struct.shape[0])
        l_ji = self.define_parameter('l_ji', self.q_ji_struct.shape[0])
        rho = self.define_parameter('rho')
        if problem is None:
            # put z and l variables in the struct format
            z_i = self.q_i_struct(z_i)
            z_ji = self.q_ji_struct(z_ji)
            l_i = self.q_i_struct(l_i)
            l_ji = self.q_ji_struct(l_ji)
            # get time info
            t = self.define_symbol('t')
            T = self.define_symbol('T')
            t0 = t/T
            # get (part of) variables
            x_i = self._get_x_variables(symbolic=True)
            # transform spline variables: only consider future piece of spline
            tf = lambda cfs, basis: shift_knot1_fwd(cfs, basis, t0)
            self._transform_spline([x_i, z_i, l_i], tf, self.q_i)
            self._transform_spline([z_ji, l_ji], tf, self.q_ji)
            # construct objective
            obj = 0.
            for child, q_i in self.q_i.items():
                for name in q_i.keys():
                    x = x_i[child.label][name]
                    z = z_i[child.label, name]
                    l = l_i[child.label, name]
                    obj += mtimes(l.T, x-z) + 0.5*rho*mtimes((x-z).T, (x-z))
                    for nghb in self.q_ji.keys():
                        z = z_ji[str(nghb), child.label, name]
                        l = l_ji[str(nghb), child.label, name]
                        obj += mtimes(l.T, x-z) + 0.5*rho*mtimes((x-z).T, (x-z))
            self.define_objective(obj)
            # construct problem
            prob, _ = self.father_updx.construct_problem(
                self.options, str(self._index))
        else:
            prob, _ = self.father_updx.construct_problem(self.options, str(self._index), problem)
        self.problem_upd_x = prob
        self.father_updx.init_transformations(self.problem.init_primal_transform,
            self.problem.init_dual_transform)
        self.init_var_admm()

    def construct_upd_z(self, problem=None, lineq_updz=True):
        if problem is not None:
            self.problem_upd_z = problem
            self._lineq_updz = lineq_updz
            return
        # check if we have linear equality constraints
        self._lineq_updz, A, b = self._check_for_lineq()
        if not self._lineq_updz:
            raise ValueError('For now, only equality constrained QP ' +
                             'z-updates are allowed!')
        x_i = struct_symMX(self.q_i_struct)
        x_j = struct_symMX(self.q_ij_struct)
        l_i = struct_symMX(self.q_i_struct)
        l_ij = struct_symMX(self.q_ij_struct)
        t = MX.sym('t')
        T = MX.sym('T')
        rho = MX.sym('rho')
        par = struct_symMX(self.par_struct)
        inp = [x_i.cat, l_i.cat, l_ij.cat, x_j.cat, t, T, rho, par.cat]
        t0 = t/T
        # put symbols in MX structs (necessary for transformation)
        x_i = self.q_i_struct(x_i)
        x_j = self.q_ij_struct(x_j)
        l_i = self.q_i_struct(l_i)
        l_ij = self.q_ij_struct(l_ij)
        # transform spline variables: only consider future piece of spline
        tf = lambda cfs, basis: shift_knot1_fwd(cfs, basis, t0)
        self._transform_spline([x_i, l_i], tf, self.q_i)
        self._transform_spline([x_j, l_ij], tf, self.q_ij)
        # fill in parameters
        A = A(par.cat)
        b = b(par.cat)
        # build KKT system
        E = mtimes(rho, MX.eye(A.shape[1]))
        l, x = vertcat(l_i.cat, l_ij.cat), vertcat(x_i.cat, x_j.cat)
        f = -(l + rho*x)
        G = vertcat(horzcat(E, A.T),
                    horzcat(A, MX.zeros(A.shape[0], A.shape[0])))
        h = vertcat(-f, b)
        z = solve(G, h)
        l_qi = self.q_i_struct.shape[0]
        l_qij = self.q_ij_struct.shape[0]
        z_i_new = self.q_i_struct(z[:l_qi])
        z_ij_new = self.q_ij_struct(z[l_qi:l_qi+l_qij])
        # transform back
        tf = lambda cfs, basis: shift_knot1_bwd(cfs, basis, t0)
        self._transform_spline(z_i_new, tf, self.q_i)
        self._transform_spline(z_ij_new, tf, self.q_ij)
        out = [z_i_new.cat, z_ij_new.cat]
        # create problem
        prob, _ = create_function('upd_z_'+str(self._index), inp, out, self.options)
        self.problem_upd_z = prob

    # def _construct_upd_z_nlp(self, problem=None):
    #     warnings.warn('Your z update is not an equality constrained QP. ' +
    #                   'You are exploring highly experimental and non-tested ' +
    #                   'code. Good luck!')
    #     # construct variables
    #     self._var_struct_updz = struct([entry('z_i', struct=self.q_i_struct),
    #                                     entry('z_ij', struct=self.q_ij_struct)])
    #     var = struct_symMX(self._var_struct_updz)
    #     z_i = self.q_i_struct(var['z_i'])
    #     z_ij = self.q_ij_struct(var['z_ij'])
    #     # construct parameters
    #     self._par_struct_updz = struct([entry('x_i', struct=self.q_i_struct),
    #                                     entry('x_j', struct=self.q_ij_struct),
    #                                     entry('l_i', struct=self.q_i_struct),
    #                                     entry('l_ij', struct=self.q_ij_struct),
    #                                     entry('t'), entry('T'), entry('rho'),
    #                                     entry('par', struct=self.par_struct)])
    #     par = struct_symMX(self._par_struct_updz)
    #     x_i, x_j = self.q_i_struct(par['x_i']), self.q_ij_struct(par['x_j'])
    #     l_i, l_ij = self.q_i_struct(par['l_i']), self.q_ij_struct(par['l_ij'])
    #     t, T, rho = par['t'], par['T'], par['rho']
    #     t0 = t/T
    #     # transform spline variables: only consider future piece of spline
    #     tf = lambda cfs, basis: shift_knot1_fwd(cfs, basis, t0)
    #     self._transform_spline([x_i, z_i, l_i], tf, self.q_i)
    #     self._transform_spline([x_j, z_ij, l_ij], tf, self.q_ij)
    #     # construct constraints
    #     constraints, lb, ub = [], [], []
    #     for con in self.constraints:
    #         c = con[0]
    #         for sym in symvar(c):
    #             for label, child in self.group.items():
    #                 if sym.name() in child.symbol_dict:
    #                     name = child.symbol_dict[sym.name()][1]
    #                     v = z_i[label, name]
    #                     ind = self.q_i[child][name]
    #                     sym2 = MX.zeros(sym.size())
    #                     sym2[ind] = v
    #                     sym2 = reshape(sym2, sym.shape)
    #                     c = substitute(c, sym, sym2)
    #                     break
    #             for nghb in self.q_ij.keys():
    #                 for label, child in nghb.group.items():
    #                     if sym.name() in child.symbol_dict:
    #                         name = child.symbol_dict[sym.name()][1]
    #                         v = z_ij[nghb.label, label, name]
    #                         ind = self.q_ij[nghb][child][name]
    #                         sym2 = MX.zeros(sym.size())
    #                         sym2[ind] = v
    #                         sym2 = reshape(sym2, sym.shape)
    #                         c = substitute(c, sym, sym2)
    #                         break
    #             for name, s in self.par_i.items():
    #                 if s.name() == sym.name():
    #                     c = substitute(c, sym, par['par', name])
    #         constraints.append(c)
    #         lb.append(con[1])
    #         ub.append(con[2])
    #     self.lb_updz, self.ub_updz = lb, ub
    #     # construct objective
    #     obj = 0.
    #     for child, q_i in self.q_i.items():
    #         for name in q_i.keys():
    #             x = x_i[child.label, name]
    #             z = z_i[child.label, name]
    #             l = l_i[child.label, name]
    #             obj += mtimes(l.T, x-z) + 0.5*rho*mtimes((x-z).T, (x-z))
    #     for nghb in self.q_ij.keys():
    #         for child, q_ij in self.q_ij[nghb].items():
    #             for name in q_ij.keys():
    #                 x = x_j[str(nghb), child.label, name]
    #                 z = z_ij[str(nghb), child.label, name]
    #                 l = l_ij[str(nghb), child.label, name]
    #                 obj += mtimes(l.T, x-z) + 0.5*rho*mtimes((x-z).T, (x-z))
    #     # construct problem
    #     prob, _ = create_nlp(var, par, obj, constraints, self.options, str(self._index), problem)
    #     self.problem_upd_z = prob

    def construct_upd_l(self, problem=None):
        if problem is not None:
            self.problem_upd_l = problem
            return
        # create parameters
        x_i = struct_symMX(self.q_i_struct)
        z_i = struct_symMX(self.q_i_struct)
        z_ij = struct_symMX(self.q_ij_struct)
        l_i = struct_symMX(self.q_i_struct)
        l_ij = struct_symMX(self.q_ij_struct)
        x_j = struct_symMX(self.q_ij_struct)
        t = MX.sym('t')
        T = MX.sym('T')
        rho = MX.sym('rho')
        inp = [x_i, z_i, z_ij, l_i, l_ij, x_j, t, T, rho]
        # update lambda
        l_i_new = self.q_i_struct(l_i.cat + rho*(x_i.cat - z_i.cat))
        l_ij_new = self.q_ij_struct(l_ij.cat + rho*(x_j.cat - z_ij.cat))
        out = [l_i_new, l_ij_new]
        # create problem
        prob, _ = create_function('upd_l_'+str(self._index), inp, out, self.options)
        self.problem_upd_l = prob

    # ========================================================================
    # Auxiliary methods
    # ========================================================================

    def _check_for_lineq(self):
        g = []
        for con in self.constraints:
            lb, ub = con[1], con[2]
            g = vertcat(g, con[0] - lb)
            if not isinstance(lb, np.ndarray):
                lb, ub = [lb], [ub]
            for k, _ in enumerate(lb):
                if lb[k] != ub[k]:
                    return False, None, None
        sym, jac = [], []
        for child, q_i in self.q_i.items():
            for name, ind in q_i.items():
                var = self.distr_problem.father.get_variables(child, name, spline=False, symbolic=True, substitute=False)
                jj = jacobian(g, var)
                jac = horzcat(jac, jj[:, ind])
                sym.append(var)
        for nghb in self.q_ij.keys():
            for child, q_ij in self.q_ij[nghb].items():
                for name, ind in q_ij.items():
                    var = self.distr_problem.father.get_variables(child, name, spline=False, symbolic=True, substitute=False)
                    jj = jacobian(g, var)
                    jac = horzcat(jac, jj[:, ind])
                    sym.append(var)
        for sym in symvar(jac):
            if sym not in self.par_i.values():
                return False, None, None
        par = struct_symMX(self.par_struct)
        A, b = jac, -g
        for s in sym:
            A = substitute(A, s, np.zeros(s.shape))
            b = substitute(b, s, np.zeros(s.shape))
        dep_b = [s.name() for s in symvar(b)]
        dep_A = [s.name() for s in symvar(b)]
        for name, sym in self.par_i.items():
            if sym.name() in dep_b:
                b = substitute(b, sym, par[name])
            if sym.name() in dep_A:
                A = substitute(A, sym, par[name])
        A = Function('A', [par], [A]).expand()
        b = Function('b', [par], [b]).expand()
        return True, A, b

    # ========================================================================
    # Methods related to solving the problem
    # ========================================================================

    def init_var_admm(self):
        for nghb, q_ji in self.q_ji.items():
            for child, q in q_ji.items():
                for name, ind in q.items():
                    var = self.father_updx.get_variables(child, name, spline=False).T.flatten()[ind]
                    self.var_admm['z_ji'][str(nghb), child.label, name] = var
        for child, q in self.q_i.items():
            for name, ind in q.items():
                # var = self.father_updx._var_result[child.label, name][ind]
                var = self.father_updx.get_variables(child, name, spline=False).T.flatten()[ind]
                self.var_admm['x_i'][child.label, name] = var
                self.var_admm['z_i'][child.label, name] = var

    def set_parameters(self, current_time):
        parameters = {}
        parameters['z_i'] = self.var_admm['z_i'].cat
        parameters['z_ji'] = self.var_admm['z_ji'].cat
        parameters['l_i'] = self.var_admm['l_i'].cat
        parameters['l_ji'] = self.var_admm['l_ji'].cat
        parameters['rho'] = self.options['rho']
        return parameters

    def update_x(self, current_time):
        self.current_time = current_time
        self.problem.current_time = current_time
        # set initial guess, parameters, lb & ub
        var = self.father_updx.get_variables()
        par = self.father_updx.set_parameters(current_time)
        lb, ub = self.father_updx.update_bounds(current_time)
        # solve!
        t0 = time.time()
        result = self.problem_upd_x(x0=var, p=par, lbg=lb, ubg=ub)
        t1 = time.time()
        t_upd = t1-t0
        self.father_updx.set_variables(result['x'])
        self.var_admm['x_i'] = self._get_x_variables()
        stats = self.problem_upd_x.stats()
        if (stats['return_status'] != 'Solve_Succeeded'):
            print 'upd_x %d: %s' % (self._index, stats['return_status'])
        return t_upd

    def set_parameters_upd_z(self, current_time):
        parameters = self.par_struct(0)
        global_par = self.distr_problem.set_parameters(current_time)
        for name in self.par_i:
            parameters[name] = global_par[name]
        return parameters

    def update_z(self, current_time):
        # save previous result
        t0 = time.time()
        self.var_admm['z_i_p'] = self.var_admm['z_i']
        self.var_admm['z_ij_p'] = self.var_admm['z_ij']
        current_time = np.round(current_time, 6) % self.problem.knot_time
        horizon_time = self.problem.options['horizon_time']
        rho = self.options['rho']
        if self._lineq_updz:
            # set inputs
            x_i = self.var_admm['x_i']
            l_i = self.var_admm['l_i']
            l_ij = self.var_admm['l_ij']
            x_j = self.var_admm['x_j']
            t = current_time
            T = horizon_time
            par = self.set_parameters_upd_z(current_time)
            out = self.problem_upd_z(x_i, l_i, l_ij, x_j, t, T, rho, par)
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
            par['par'] = self.set_parameters_upd_z(current_time)
            lb, ub = self.lb_updz, self.ub_updz
            result = self.problem_upd_z(p=par, lbg=lb, ubg=ub)
            out = result['x']
            out = self._var_struct_updz(out)
            z_i, z_ij = out['z_i'], out['z_ij']
        self.var_admm['z_i'] = self.q_i_struct(z_i)
        self.var_admm['z_ij'] = self.q_ij_struct(z_ij)
        t1 = time.time()
        return t1-t0

    def update_l(self, current_time):
        # save previous result
        t0 = time.time()
        self.var_admm['l_i_p'] = self.var_admm['l_i']
        self.var_admm['l_ij_p'] = self.var_admm['l_ij']
        # set inputs
        x_i = self.var_admm['x_i']
        z_i = self.var_admm['z_i']
        z_ij = self.var_admm['z_ij']
        l_i = self.var_admm['l_i']
        l_ij = self.var_admm['l_ij']
        x_j = self.var_admm['x_j']
        t = np.round(current_time, 6) % self.problem.knot_time
        T = self.problem.options['horizon_time']
        rho = self.options['rho']
        out = self.problem_upd_l(x_i, z_i, z_ij, l_i, l_ij, x_j, t, T, rho)
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

    def init_step(self, current_time, update_time):
        self.problem.init_step(current_time, update_time)
        # transform spline variables
        if ((current_time > 0. and
             np.round(current_time, 6) % self.problem.knot_time == 0)):
            tf = shift_over_knot
            for key in ['x_i', 'z_i', 'z_i_p', 'l_i', 'l_i_p']:
                self.var_admm[key] = self._transform_spline(
                    self.var_admm[key], tf, self.q_i)
            for key in ['x_j', 'z_ij', 'z_ij_p', 'l_ij', 'l_ij_p']:
                self.var_admm[key] = self._transform_spline(
                    self.var_admm[key], tf, self.q_ij)
            for key in ['z_ji', 'l_ji']:
                self.var_admm[key] = self._transform_spline(
                    self.var_admm[key], tf, self.q_ji)

    def get_residuals(self, current_time):
        t0 = time.time()
        current_time = np.round(current_time, 6) % self.problem.knot_time
        horizon_time = self.problem.options['horizon_time']
        tf = lambda cfs, basis: shift_knot1_fwd(
            cfs, basis, current_time/horizon_time)
        x_i = self._transform_spline(self.var_admm['x_i'], tf, self.q_i).cat
        z_i = self._transform_spline(self.var_admm['z_i'], tf, self.q_i).cat
        z_i_p = self._transform_spline(
            self.var_admm['z_i_p'], tf, self.q_i).cat
        x_j = self._transform_spline(self.var_admm['x_j'], tf, self.q_ij).cat
        z_ij = self._transform_spline(self.var_admm['z_ij'], tf, self.q_ij).cat
        z_ij_p = self._transform_spline(
            self.var_admm['z_ij_p'], tf, self.q_ij).cat
        rho = self.options['rho']
        pr = la.norm(x_i-z_i)**2 + la.norm(x_j-z_ij)**2
        dr = rho*(la.norm(z_i-z_i_p)**2 + la.norm(z_ij-z_ij_p)**2)
        cr = rho*pr + dr
        t1 = time.time()
        return t1-t0, pr, dr, cr

    def accelerate(self, c_res):
        eta = self.options['eta']
        if not hasattr(self, 'c_res_p'):
            self.c_res_p = (1./eta)*c_res
        if not hasattr(self, 'alpha'):
            self.alpha = 1.
        z_i = self.var_admm['z_i'].cat
        z_ij = self.var_admm['z_ij'].cat
        z_i_p = self.var_admm['z_i_p'].cat
        z_ij_p = self.var_admm['z_ij_p'].cat
        l_i = self.var_admm['l_i'].cat
        l_ij = self.var_admm['l_ij'].cat
        l_i_p = self.var_admm['l_i_p'].cat
        l_ij_p = self.var_admm['l_ij_p'].cat
        if self.options['nesterov_reset']:
            if c_res <= eta*self.c_res_p:
                alpha_p = self.alpha
                self.alpha = 0.5*(1. + np.sqrt(1 + 4.*alpha_p**2))
                z_i = z_i + ((alpha_p - 1)/self.alpha)*(z_i - z_i_p)
                z_ij = z_ij + ((alpha_p - 1)/self.alpha)*(z_ij - z_ij_p)
                l_i = l_i + ((alpha_p - 1)/self.alpha)*(l_i - l_i_p)
                l_ij = l_ij + ((alpha_p - 1)/self.alpha)*(l_ij - l_ij_p)
                self.c_res_p = c_res
            else:
                print 'resetting alpha'
                self.alpha = 1.
                z_i = z_i_p
                z_ij = z_ij_p
                l_i = l_i_p
                l_ij = l_ij_p
                self.c_res_p = (1./eta)*self.c_res_p
        else:
            alpha_p = self.alpha
            self.alpha = 0.5*(1. + np.sqrt(1 + 4.*alpha_p**2))
            z_i = z_i + ((alpha_p - 1)/self.alpha)*(z_i - z_i_p)
            z_ij = z_ij + ((alpha_p - 1)/self.alpha)*(z_ij - z_ij_p)
            l_i = l_i + ((alpha_p - 1)/self.alpha)*(l_i - l_i_p)
            l_ij = l_ij + ((alpha_p - 1)/self.alpha)*(l_ij - l_ij_p)
            self.c_res_p = c_res
        self.var_admm['z_i'] = self.q_i_struct(z_i)
        self.var_admm['z_ij'] = self.q_ij_struct(z_ij)
        self.var_admm['l_i'] = self.q_i_struct(l_i)
        self.var_admm['l_ij'] = self.q_ij_struct(l_ij)


class ADMMProblem(DualProblem):

    def __init__(self, fleet, environment, problems, options):
        DualProblem.__init__(
            self, fleet, environment, problems, ADMM, options)
        self.residuals = {'primal': [], 'dual': [], 'combined': []}

    # ========================================================================
    # ADMM options
    # ========================================================================

    def set_default_options(self):
        DualProblem.set_default_options(self)
        self.options.update({'nesterov_acceleration': False, 'eta': 0.999,
                             'nesterov_reset': False})

    def get_stacked_x_var_it(self):
        stacked_x_var = np.zeros((0, 1))
        for updater in self.updaters:
            stacked_x_var = np.vstack((stacked_x_var, updater.var_admm['x_i'].cat))
        return stacked_x_var

    def dual_update(self, current_time, update_time):
        t_upd_x, t_upd_z, t_upd_l, t_res = 0., 0., 0., 0.
        p_res, d_res, c_res = 0., 0., 0.
        for updater in self.updaters:
            updater.init_step(current_time, update_time)
        for updater in self.updaters:
            t = updater.update_x(current_time)
            t_upd_x = max(t_upd_x, t)
        for updater in self.updaters:
            updater.communicate()
        for updater in self.updaters:
            t1 = updater.update_z(current_time)
            t2 = updater.update_l(current_time)
            t3, pr, dr, cr = updater.get_residuals(current_time)
            t_upd_z = max(t_upd_z, t1)
            t_upd_l = max(t_upd_l, t2)
            t_res = max(t_res, t3)
            p_res += pr**2
            d_res += dr**2
            c_res += cr**2
        p_res, d_res, c_res = np.sqrt(
            p_res), np.sqrt(d_res), np.sqrt(c_res)
        if self.options['nesterov_acceleration']:
            for updater in self.updaters:
                updater.accelerate(c_res)
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
        self.residuals['primal'] = np.r_[self.residuals['primal'], p_res]
        self.residuals['dual'] = np.r_[self.residuals['dual'], d_res]
        self.residuals['combined'] = np.r_[
            self.residuals['combined'], c_res]
        self.update_times.append(t_upd_x + t_upd_z + t_upd_l + t_res)

    # ========================================================================
    # Plot related functions
    # ========================================================================

    def init_plot(self, argument, **kwargs):
        if argument == 'residuals':
            if len(self.residuals['primal']) == 0:
                return None
            labels = ['Primal residual (log10)', 'Dual residual (log10)',
                      'Combined residual (log10)']
            info = []
            for k in range(3):
                lines = [{'linestyle': 'None', 'marker': '*',
                          'color': self.colors[k]}]
                info.append([{'labels': ['Iteration', labels[k]],
                              'lines': lines}])
            return info
        else:
            return Problem.init_plot(self, argument, **kwargs)

    def update_plot(self, argument, t, **kwargs):
        if argument == 'residuals':
            if len(self.residuals['primal']) == 0:
                return None
            data = []
            for residual in self.residuals.values():
                lines = []
                if t == -1:
                    n_it = len(residual)
                    iterations = np.linspace(1, n_it, n_it)
                    lines.append([iterations, np.log10(residual)])
                else:
                    ind = (self.options['init_iter'] +
                           t*self.options['max_iter_per_update'])
                    n_it = ind + 1
                    iterations = np.linspace(1, n_it, n_it)
                    lines.append([iterations, np.log10(residual[:ind+1])])
                data.append([lines])
            return data
        else:
            return Problem.update_plot(self, argument, t, **kwargs)
