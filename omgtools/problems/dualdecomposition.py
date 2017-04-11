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

from ..basics.optilayer import OptiFather, create_function
from ..basics.spline import *
from problem import Problem
from dualmethod import DualUpdater, DualProblem
from casadi import symvar, mtimes, MX, reshape, substitute
from casadi.tools import struct_symMX
import numpy as np
import numpy.linalg as la
import time


class DDUpdater(DualUpdater):

    def __init__(self, index, vehicle, problem, environment, distr_problem,
                 options=None):
        DualUpdater.__init__(self, index, vehicle, problem, environment,
                             distr_problem, 'dd', options)

    # ========================================================================
    # Create problem
    # ========================================================================

    def init(self, problems=None):
        if problems is None:
            problems = {'upd_xz': None, 'upd_l': None}
        DualUpdater.init(self)
        self.var_dd = {}
        for key in ['x_i']:
            self.var_dd[key] = self.q_i_struct(0)
        for key in ['x_j', 'z_ij', 'z_ij_p', 'l_ij', 'l_ij_p']:
            self.var_dd[key] = self.q_ij_struct(0)
        for key in ['l_ji']:
            self.var_dd[key] = self.q_ji_struct(0)
        time_buildxz = self.construct_upd_xz(problems['upd_xz'])
        time_buildl = self.construct_upd_l(problems['upd_l'])
        buildtime = time_buildxz + time_buildl
        return {'upd_xz': self.problem_upd_xz, 'upd_l': self.problem_upd_l}, buildtime

    def construct_upd_xz(self, problem=None):
        # construct optifather & give reference to problem
        self.father_updx = OptiFather(self.group.values())
        self.problem.father = self.father_updx
        # define z_ij variables
        init = self.q_ij_struct(0)
        for nghb, q_ij in self.q_ij.items():
            for child, q_j in q_ij.items():
                for name, ind in q_j.items():
                    var = np.array(child._values[name])
                    v = var.T.flatten()[ind]
                    init[nghb.label, child.label, name, ind] = v
        z_ij = self.define_variable(
            'z_ij', self.q_ij_struct.shape[0], value=np.array(init.cat))
        # define parameters
        l_ij = self.define_parameter('l_ij', self.q_ij_struct.shape[0])
        l_ji = self.define_parameter('l_ji', self.q_ji_struct.shape[0])
        # put them in the struct format
        z_ij = self.q_ij_struct(z_ij)
        l_ij = self.q_ij_struct(l_ij)
        l_ji = self.q_ji_struct(l_ji)
        # get (part of) variables
        x_i = self._get_x_variables(symbolic=True)
        # construct local copies of parameters
        par = {}
        for name, s in self.par_i.items():
            par[name] = self.define_parameter(name, s.shape[0], s.shape[1])
        if problem is None:
            # get time info
            t = self.define_symbol('t')
            T = self.define_symbol('T')
            t0 = t/T
            # transform spline variables: only consider future piece of spline
            tf = lambda cfs, basis: spl.Function(basis, cfs).shiftfirstknot_fwd(t0).data()
            self._transform_spline(x_i, tf, self.q_i)
            self._transform_spline([z_ij, l_ij], tf, self.q_ij)
            self._transform_spline(l_ji, tf, self.q_ji)
            # construct objective
            obj = 0.
            for child, q_i in self.q_i.items():
                for name in q_i.keys():
                    x = x_i[child.label][name]
                    for nghb in self.q_ji.keys():
                        l = l_ji[str(nghb), child.label, name]
                        obj += mtimes(l.T, x)
            for nghb, q_j in self.q_ij.items():
                for child in q_j.keys():
                    for name in q_j[child].keys():
                        z = z_ij[str(nghb), child.label, name]
                        l = l_ij[str(nghb), child.label, name]
                        obj -= mtimes(l.T, z)
            self.define_objective(obj)
        # construct constraints
        for con in self.constraints:
            c = con[0]
            for sym in symvar(c):
                for label, child in self.group.items():
                    if sym.name() in child.symbol_dict:
                        name = child.symbol_dict[sym.name()][1]
                        v = x_i[label][name]
                        ind = self.q_i[child][name]
                        sym2 = MX.zeros(sym.size())
                        sym2[ind] = v
                        sym2 = reshape(sym2, sym.shape)
                        c = substitute(c, sym, sym2)
                        break
                for nghb in self.q_ij.keys():
                    for label, child in nghb.group.items():
                        if sym.name() in child.symbol_dict:
                            name = child.symbol_dict[sym.name()][1]
                            v = z_ij[nghb.label, label, name]
                            ind = self.q_ij[nghb][child][name]
                            sym2 = MX.zeros(sym.size())
                            sym2[ind] = v
                            sym2 = reshape(sym2, sym.shape)
                            c = substitute(c, sym, sym2)
                            break
                for name, s in self.par_i.items():
                    if s.name() == sym.name():
                        c = substitute(c, sym, par[name])
            lb, ub = con[1], con[2]
            self.define_constraint(c, lb, ub)
        # construct problem
        prob, buildtime = self.father_updx.construct_problem(
            self.options, str(self._index), problem)
        self.problem_upd_xz = prob
        self.father_updx.init_transformations(self.problem.init_primal_transform,
                                         self.problem.init_dual_transform)
        self.init_var_dd()
        return buildtime

    def construct_upd_l(self, problem=None):
        if problem is not None:
            self.problem_upd_l = problem
            return 0.
        # create parameters
        z_ij = struct_symMX(self.q_ij_struct)
        l_ij = struct_symMX(self.q_ij_struct)
        x_j = struct_symMX(self.q_ij_struct)
        t = MX.sym('t')
        T = MX.sym('T')
        rho = MX.sym('rho')
        inp = [x_j, z_ij, l_ij, t, T, rho]
        # update lambda
        l_ij_new = self.q_ij_struct(l_ij.cat + rho*(x_j.cat - z_ij.cat))
        out = [l_ij_new]
        # create problem
        prob, buildtime = create_function('upd_l_'+str(self._index), inp, out, self.options)
        self.problem_upd_l = prob
        return buildtime

    # ========================================================================
    # Methods related to solving the problem
    # ========================================================================

    def init_var_dd(self):
        for child, q in self.q_i.items():
            for name, ind in q.items():
                var = self.father_updx.get_variables(child, name, spline=False).T.flatten()[ind]
                self.var_dd['x_i'][child.label, name] = var

    def set_parameters(self, current_time):
        parameters = {}
        global_par = self.distr_problem.set_parameters(current_time)
        for name in self.par_i:
            parameters[name] = global_par[name]
        parameters['l_ij'] = self.var_dd['l_ij'].cat
        parameters['l_ji'] = self.var_dd['l_ji'].cat
        return parameters

    def update_xz(self, current_time):
        # set initial guess, parameters, lb & ub
        var = self.father_updx.get_variables()
        par = self.father_updx.set_parameters(current_time)
        lb, ub = self.father_updx.update_bounds(current_time)
        # solve!
        t0 = time.time()
        result = self.problem_upd_xz(x0=var, p=par, lbg=lb, ubg=ub)
        t1 = time.time()
        t_upd = t1-t0
        self.father_updx.set_variables(result['x'])
        self.var_dd['x_i'] = self._get_x_variables()
        z_ij = self.father_updx.get_variables(self, 'z_ij', spline=False)
        self.var_dd['z_ij'] = self.q_ij_struct(z_ij)
        stats = self.problem_upd_xz.stats()
        if (stats['return_status'] != 'Solve_Succeeded'):
            print 'upd_xz %d: %s' % (self._index, stats['return_status'])
        return t_upd

    def update_l(self, current_time):
        # save previous result
        t0 = time.time()
        self.var_dd['l_ij_p'] = self.var_dd['l_ij']
        # set inputs
        x_j = self.var_dd['x_j']
        z_ij = self.var_dd['z_ij']
        l_ij = self.var_dd['l_ij']
        t = np.round(current_time, 6) % self.problem.knot_time
        T = self.problem.options['horizon_time']
        rho = self.options['rho']
        out = self.problem_upd_l(x_j, z_ij, l_ij, t, T, rho)
        self.var_dd['l_ij'] = self.q_ij_struct(out)
        t1 = time.time()
        return t1-t0

    def communicate(self):
        for nghb in self.q_ji.keys():
            l_ji = nghb.var_dd['l_ij'].prefix[str(self)]
            x_j = nghb.var_dd['x_i']
            self.var_dd['l_ji'][str(nghb)] = l_ji
            self.var_dd['x_j'][str(nghb)] = x_j

    def init_step(self, current_time, update_time):
        self.problem.init_step(current_time, update_time)
        # transform spline variables
        if ((current_time > 0. and
             np.round(current_time, 6) % self.problem.knot_time == 0)):
            tf = lambda cfs, basis: basis.shiftoverknot()[1].dot(cfs)
            for key in ['x_i']:
                self.var_dd[key] = self._transform_spline(
                    self.var_dd[key], tf, self.q_i)
            for key in ['x_j', 'z_ij', 'l_ij', 'l_ij_p']:
                self.var_dd[key] = self._transform_spline(
                    self.var_dd[key], tf, self.q_ij)
            for key in ['l_ji']:
                self.var_dd[key] = self._transform_spline(
                    self.var_dd[key], tf, self.q_ji)

    def get_residuals(self, current_time):
        t0 = time.time()
        current_time = np.round(current_time, 6) % self.problem.knot_time
        horizon_time = self.problem.options['horizon_time']
        tf = lambda cfs, basis: spl.Function(basis, cfs).shiftfirstknot_fwd(t0).data()
        x_j = self._transform_spline(self.var_dd['x_j'], tf, self.q_ij).cat
        z_ij = self._transform_spline(self.var_dd['z_ij'], tf, self.q_ij).cat
        pr = la.norm(x_j-z_ij)**2
        t1 = time.time()
        return t1-t0, pr


class DDProblem(DualProblem):

    def __init__(self, fleet, environment, problems, options):
        DualProblem.__init__(
            self, fleet, environment, problems, DDUpdater, options)
        self.residuals = {'primal': []}

    def reinitialize(self):
        for updater in self.updaters:
            updater.problem.reinitialize(father=updater.father_updx)
            updater.init_var_dd()

    def get_stacked_x_var_it(self):
        stacked_x_var = np.zeros((0, 1))
        for updater in self.updaters:
            stacked_x_var = np.vstack((stacked_x_var, updater.var_dd['x_i'].cat))
        return stacked_x_var

    def dual_update(self, current_time, update_time):
        t_upd_xz, t_upd_l, t_res = 0., 0., 0.
        p_res = 0.
        for updater in self.updaters:
            updater.init_step(current_time, update_time)
        for updater in self.updaters:
            t = updater.update_xz(current_time)
            t_upd_xz = max(t_upd_xz, t)
        for updater in self.updaters:
            updater.communicate()
        for updater in self.updaters:
            t1 = updater.update_l(current_time)
            t2, pr = updater.get_residuals(current_time)
            t_upd_l = max(t_upd_l, t1)
            t_res = max(t_res, t2)
            p_res += pr**2
        p_res = np.sqrt(p_res)
        for updater in self.updaters:
            updater.communicate()
        self.iteration += 1
        if self.options['verbose'] >= 2:
            if ((self.iteration - 1) % 20 == 0):
                print(
                    '----|------|----------|----------|----------|----------')
                print('%3s | %4s | %8s | %8s | %8s | %8s ' %
                      ('It', 't', 'prim res', 't upd_xz', 't upd_l', 't_res'))
                print(
                    '----|------|----------|----------|----------|----------')
            print('%3d | %4.1f | %.2e | %.2e | %.2e | %.2e ' %
                  (self.iteration, current_time, p_res, t_upd_xz, t_upd_l, t_res))
        self.residuals['primal'] = np.r_[self.residuals['primal'], p_res]
        self.update_times.append(t_upd_xz + t_upd_l + t_res)

    # ========================================================================
    # Plot related functions
    # ========================================================================

    def init_plot(self, argument, **kwargs):
        if argument == 'residuals':
            if len(self.residuals['primal']) == 0:
                return None
            labels = ['Primal residual (log10)']
            info = []
            for k in range(1):
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
