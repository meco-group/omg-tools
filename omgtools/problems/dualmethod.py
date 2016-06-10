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

from problem import Problem
from distributedproblem import DistributedProblem
from casadi import SX, MX, DM
from casadi.tools import struct, entry, structure
import pickle


def _create_struct_from_dict(dictionary):
    entries = []
    for key, data in dictionary.items():
        if isinstance(data, dict):
            stru = _create_struct_from_dict(data)
            entries.append(entry(str(key), struct=stru))
        else:
            if isinstance(data, list):
                sh = len(data)
            else:
                sh = data.shape
            entries.append(entry(key, shape=sh))
    return struct(entries)


class DualUpdater(Problem):

    def __init__(self, index, vehicle, problem, environment, distr_problem,
                 label, options=None):
        Problem.__init__(self, vehicle, environment, options, label=label)
        self.problem = problem
        self.distr_problem = distr_problem
        self.vehicle = vehicle
        self.environment = environment
        self.group = {child.label: child for child in ([
            vehicle, problem, environment, self] + environment.obstacles)}
        for child in self.group.values():
            child._index = index

    # ========================================================================
    # Dual updater options
    # ========================================================================

    def set_default_options(self):
        Problem.set_default_options(self)
        self.options.update({'rho': 0.1})

    # ========================================================================
    # Create problem
    # ========================================================================

    def init(self):
        self.q_i_struct = _create_struct_from_dict(self.q_i)
        self.q_ij_struct = _create_struct_from_dict(self.q_ij)
        self.q_ji_struct = _create_struct_from_dict(self.q_ji)
        self.par_struct = _create_struct_from_dict(self.par_i)

    # ========================================================================
    # Auxiliary methods
    # ========================================================================

    def _struct2dict(self, var, dic):
        from admm import ADMM
        from dualdecomposition import DDUpdater
        if isinstance(var, list):
            return [self._struct2dict(v, dic) for v in var]
        elif isinstance(dic.keys()[0], (DDUpdater, ADMM)):
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
        elif 'dd' in var.keys()[0] or 'admm' in var.keys()[0]:
            chck = var.values()[0].values()[0].values()[0]
            if isinstance(chck, SX):
                ret = structure.SXStruct(stru)
            elif isinstance(chck, MX):
                ret = structure.MXStruct(stru)
            elif isinstance(chck, DM):
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
            elif isinstance(chck, DM):
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
        from admm import ADMM
        from dualdecomposition import DDUpdater
        if isinstance(var, list):
            return [self._transform_spline(v, tf, dic) for v in var]
        elif isinstance(var, struct):
            var = self._struct2dict(var, dic)
            var = self._transform_spline(var, tf, dic)
            return self._dict2struct(var, _create_struct_from_dict(dic))
        elif isinstance(dic.keys()[0], (DDUpdater, ADMM)):
            ret = {}
            for nghb in dic.keys():
                ret[nghb.label] = self._transform_spline(
                    var[nghb.label], tf, dic[nghb])
            return ret
        else:
            for child, q_i in dic.items():
                for name, ind in q_i.items():
                    if name in child._splines_prim:
                        basis = child._splines_prim[name]['basis']
                        for l in range(child._variables[name].shape[1]):
                            sl_min = l*len(basis)
                            sl_max = (l+1)*len(basis)
                            if set(range(sl_min, sl_max)) <= set(ind):
                                sl = slice(sl_min, sl_max)
                                v = var[child.label][name][sl]
                                v = tf(v, basis)
                                var[child.label][name][sl] = v
            return var

    # ========================================================================
    # Methods related to solving the problem
    # ========================================================================

    def communicate(self):
        raise NotImplementedError('Please implement this method!')

    def get_residuals(self, current_time):
        raise NotImplementedError('Please implement this method!')


class DualProblem(DistributedProblem):

    def __init__(self, fleet, environment, problems, updater_type, options):
        DistributedProblem.__init__(
            self, fleet, environment, problems, updater_type, options)

    # ========================================================================
    # Dual problem options
    # ========================================================================

    def set_default_options(self):
        Problem.set_default_options(self)
        self.options.update({'max_iter': None, 'max_iter_per_update': 1,
                             'rho': 2., 'init_iter': 5, 'save_residuals': None})

    # ========================================================================
    # Perform dual update sequence
    # ========================================================================

    def initialize(self):
        self.objectives = []
        for _ in range(self.options['init_iter']):
            self.solve(0.0, 0.0)

    def solve(self, current_time, update_time):
        self.current_time = current_time
        it0 = self.iteration
        while (self.iteration - it0) < self.options['max_iter_per_update']:
            self.dual_update(current_time, update_time)
            self.objectives.append(self.compute_objective())

    def dual_update(self, current_time, update_time):
        raise NotImplementedError('Please implement this method!')

    def stop_criterium(self, current_time, update_time):
        if self.options['max_iter']:
            if self.iteration > self.options['max_iter']:
                return True
        else:
            return DistributedProblem.stop_criterium(self, current_time, update_time)

    def final(self):
        DistributedProblem.final(self)
        if self.options['save_residuals']:
            pickle.dump(
                self.residuals, open(self.options['save_residuals'], 'wb'))
