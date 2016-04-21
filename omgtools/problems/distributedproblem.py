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
from casadi import symvar, Function, sum1


def get_dependency(expression):
    sym = symvar(expression)
    f = Function('f', sym, [expression])
    dep = {}
    for index, sym in enumerate(sym):
        J = f.sparsity_jac(index, 0)
        dep[sym] = sorted(sum1(J).find())
    return dep


class DistributedProblem(Problem):

    def __init__(self, fleet, environment, problems, updater_type, options):
        self.problems = problems
        self.updater_type = updater_type
        Problem.__init__(self, fleet, environment, options)

    # ========================================================================
    # Create problem
    # ========================================================================

    def init(self):
        self.updaters = []
        for index, vehicle in enumerate(self.vehicles):
            updater = self.updater_type(index, vehicle, self.problems[index],
                                        self.problems[index].environment, self,
                                        self.options)
            self.updaters.append(updater)
        self.interprete_constraints(self.updaters)
        for updater in self.updaters:
            updater.init()

    def interprete_constraints(self, updaters):
        for upd in updaters:
            upd.q_i, upd.q_ij, upd.q_ji = {}, {}, {}
            upd.constraints = []
            upd.par_i = {}

        symbol_dict = self._compose_dictionary()

        for constraint in self._constraints.values():
            dep = {}
            par = {}
            for sym, indices in get_dependency(constraint[0]).items():
                symname = sym.name()
                if symname in symbol_dict:
                    child, name = symbol_dict[sym.name()]
                    if child not in dep:
                        dep[child] = {}
                    dep[child][name] = indices
                elif symname in self.symbol_dict:
                    name = self.symbol_dict[symname][1]
                    if name in self._parameters:
                        par[name] = sym

            for child, dic in dep.items():
                # q_i: structure of local variables i
                upd = updaters[child._index]
                upd.constraints.append(constraint)
                for name, sym in par.items():
                    if name not in upd.par_i:
                        upd.par_i[name] = sym
                if child not in upd.q_i:
                    upd.q_i[child] = {}
                for name, indices in dic.items():
                    if name not in upd.q_i[child]:
                        upd.q_i[child][name] = []
                    for i in indices:
                        if i not in upd.q_i[child][name]:
                            upd.q_i[child][name].append(i)
                    upd.q_i[child][name].sort()
                # q_ij: structure of remote variables j seen from local i
                for ch, dic_ch in dep.items():
                    if not (child == ch):
                        upd2 = updaters[ch._index]
                        if upd2 not in upd.q_ij:
                            upd.q_ij[upd2] = {}
                        if ch not in upd.q_ij[upd2]:
                            upd.q_ij[upd2][ch] = {}
                        for name, indices in dic_ch.items():
                            if name not in upd.q_ij[upd2][ch]:
                                upd.q_ij[upd2][ch][name] = []
                            for i in indices:
                                if i not in upd.q_ij[upd2][ch][name]:
                                    upd.q_ij[upd2][ch][name].append(i)
                            upd.q_ij[upd2][ch][name].sort()
        # q_ji: structure of local variables i seen from remote j
        for upd1 in updaters:
            for upd2 in upd1.q_ij.keys():
                upd2.q_ji[upd1] = upd1.q_ij[upd2]

    def _compose_dictionary(self):
        children = [veh for veh in self.vehicles]
        children.extend(self.problems)
        children.append(self.environment)
        children.extend(self.environment.obstacles)
        symbol_dict = {}
        for child in children:
            symbol_dict.update(child.symbol_dict)
        return symbol_dict

    # ========================================================================
    # Methods required for simulation
    # ========================================================================

    def update(self, current_time, update_time, sample_time):
        for problem in self.problems:
            problem.update(current_time, update_time, sample_time)
        self.environment.update(update_time, sample_time)
        self.fleet.update_plots()
        self.update_plots()

    def stop_criterium(self, current_time, update_time):
        stop = True
        for problem in self.problems:
            stop *= problem.stop_criterium(current_time, update_time)
        return stop

    def final(self):
        obj = self.compute_objective()
        print '\nWe reached our target!'
        print '%-18s %6g' % ('Objective:', obj)
        print '%-18s %6g ms' % ('Max update time:',
                                max(self.update_times)*1000.)
        print '%-18s %6g ms' % ('Av update time:',
                                (sum(self.update_times)*1000. /
                                 len(self.update_times)))

    def compute_objective(self):
        obj = 0.
        for problem in self.problems:
            obj += problem.compute_objective()
        return obj

    # ========================================================================
    # Methods required to override (no general implementation possible)
    # ========================================================================

    def solve(self, current_time, update_time):
        raise NotImplementedError('Please implement this method!')
