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
from casadi import symvar, Function
import collections as col
import numpy as np


def get_dependency(expression):
    sym = symvar(expression)
    f = Function('f', sym, [expression])
    dep = col.OrderedDict()
    for index, sym in enumerate(sym):
        J = f.sparsity_jac(index, 0)
        dep[sym] = sorted(set(J.T.row()))
    return dep


class DistributedProblem(Problem):

    def __init__(self, fleet, environment, problems, updater_type, options):
        self.problems = problems
        self.updater_type = updater_type
        Problem.__init__(self, fleet, environment, options)

    def set_default_options(self):
        Problem.set_default_options(self)
        self.options['separate_build'] = False

    def set_options(self, options):
        Problem.set_options(self, options)
        for problem in self.problems:
            problem.set_options(options)

    # ========================================================================
    # Create problem
    # ========================================================================

    def construct(self):
        for problem in self.problems:
            problem.construct()

    def init(self):
        for problem in self.problems:
            problem.father.reset()
        self.construct()
        self.updaters = []
        for index, vehicle in enumerate(self.vehicles):
            updater = self.updater_type(index, vehicle, self.problems[index],
                                        self.problems[index].environment, self,
                                        self.options)
            self.updaters.append(updater)
        self.interprete_constraints(self.updaters)
        buildtime = []
        if self.options['separate_build']:
            for updater in self.updaters:
                _, bt = updater.init()
                buildtime.append(bt)
        else:
            updaters = self.separate_per_build()
            for veh_type, nghb_nr in updaters.items():
                for nr, upd in nghb_nr.items():
                    if self.options['verbose'] >= 2:
                        print('*Construct problem for type %s with %d neighbor(s):' % (veh_type, nr))
                    problems, bt = upd[0].init()
                    buildtime.append(bt)
                    for u in upd[1:]:
                        u.init(problems)
        return np.mean(buildtime)

    def separate_per_build(self):
        vehicle_types = self.fleet.sort_vehicles()
        updaters = {}
        for veh_type, vehicles in vehicle_types.items():
            nghb_nr = col.OrderedDict()
            for veh in vehicles:
                nr = len(self.fleet.get_neighbors(veh))
                if nr not in nghb_nr:
                    nghb_nr[nr] = []
                nghb_nr[nr].append(veh)
            updaters[veh_type] = {}
            for nr, vehicles in nghb_nr.items():
                updaters[veh_type][nr] = []
                for vehicle in vehicles:
                    updaters[veh_type][nr].append(self.updaters[vehicle._index])
        return updaters

    def interprete_constraints(self, updaters):
        for upd in updaters:
            upd.q_i, upd.q_ij, upd.q_ji = col.OrderedDict(), col.OrderedDict(), col.OrderedDict()
            upd.constraints = []
            upd.par_i = col.OrderedDict()

        symbol_dict = self._compose_dictionary()

        for constraint in self._constraints.values():
            dep = col.OrderedDict()
            par = col.OrderedDict()
            for sym, indices in get_dependency(constraint[0]).items():
                symname = sym.name()
                if symname in symbol_dict:
                    child, name = symbol_dict[sym.name()]
                    if child not in dep:
                        dep[child] = col.OrderedDict()
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
                    upd.q_i[child] = col.OrderedDict()
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
                            upd.q_ij[upd2] = col.OrderedDict()
                        if ch not in upd.q_ij[upd2]:
                            upd.q_ij[upd2][ch] = col.OrderedDict()
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

        # sort dics for logical order of neighbors
        # (not necessary, but easier for debugging)
        for upd in updaters:
            upd.q_ij = self._sort_dict(upd._index, upd.q_ij)
            upd.q_ji = self._sort_dict(upd._index, upd.q_ji)

    def _compose_dictionary(self):
        children = [veh for veh in self.vehicles]
        children.extend(self.problems)
        children.append(self.environment)
        children.extend(self.environment.obstacles)
        symbol_dict = col.OrderedDict()
        for child in children:
            symbol_dict.update(child.symbol_dict)
        return symbol_dict

    def _sort_dict(self, ref, dic):
        return col.OrderedDict(sorted(dic.iteritems(), key=lambda x: x[0]._index if(x[0]._index > ref) else x[0]._index + self.fleet.N))

    # ========================================================================
    # Deploying related functions
    # ========================================================================

    def initialize(self, current_time):
        for problem in self.problems:
            problem.initialize(current_time)

    def store(self, current_time, update_time, sample_time):
        for problem in self.problems:
            problem.store(current_time, update_time, sample_time)

    def predict(self, current_time, predict_time, sample_time, states=None, delay=0):
        if states is None:
            states = [None for k in range(len(self.problems))]
        for problem, state in zip(self.problems, states):
            problem.predict(current_time, predict_time, sample_time, state, delay)

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def simulate(self, current_time, simulation_time, sample_time):
        for problem in self.problems:
            problem.simulate(current_time, simulation_time, sample_time)
        horizon_time = self.problems[0].options['horizon_time']
        if horizon_time < simulation_time:
            simulation_time = horizon_time
        self.environment.simulate(simulation_time, sample_time)
        self.fleet.update_plots()
        self.update_plots()

    def sleep(self, current_time, sleep_time, sample_time):
        for problem in self.problems:
            problem.sleep(current_time, sleep_time, sample_time)
        self.environment.simulate(sleep_time, sample_time)

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
