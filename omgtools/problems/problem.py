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

from ..basics.optilayer import OptiFather, OptiChild
from ..vehicles.fleet import get_fleet_vehicles
from ..simulation.plotlayer import PlotLayer
from itertools import groupby
import time


class Problem(OptiChild, PlotLayer):

    def __init__(self, fleet, environment, options=None, label='problem'):
        options = options or {}
        OptiChild.__init__(self, label)
        PlotLayer.__init__(self)
        self.fleet, self.vehicles = get_fleet_vehicles(fleet)
        self.environment = environment
        self.set_default_options()
        self.set_options(options)
        self.iteration = 0
        self.update_times = []

        # first add children and construct father, this allows making a
        # difference between the simulated and the processed vehicles,
        # e.g. when passing on a trailer + leading vehicle to problem, but
        # only the trailer to the simulator
        children = [vehicle for vehicle in self.vehicles]
        children += [obstacle for obstacle in self.environment.obstacles]
        children += [self, self.environment]
        self.father = OptiFather(children)

    # ========================================================================
    # Problem options
    # ========================================================================

    def set_default_options(self):
        self.options = {'verbose': 2}
        self.options['solver'] = 'ipopt'
        ipopt_options = {'ipopt.tol': 1e-3, 'ipopt.linear_solver': 'mumps',
                         'ipopt.warm_start_init_point': 'yes',
                         'ipopt.print_level': 0, 'print_time': 0}
        self.options['solver_options'] = {'ipopt': ipopt_options}
        self.options['codegen'] = {'build': None, 'flags': '-O0'}

    def set_options(self, options):
        if 'solver_options' in options:
            for key, value in options['solver_options'].items():
                self.options['solver_options'][key].update(value)
        if 'codegen' in options:
            self.options['codegen'].update(options['codegen'])
        for key in options:
            if key not in ['solver_options', 'codegen']:
                self.options[key] = options[key]

    # ========================================================================
    # Create and solve problem
    # ========================================================================

    def init(self):
        children = [vehicle for vehicle in self.vehicles]
        children += [obstacle for obstacle in self.environment.obstacles]
        children += [self, self.environment]
        self.father = OptiFather(children)
        self.problem, _ = self.father.construct_problem(
            self.options)
        self.father.init_transformations(self.init_primal_transform,
                                         self.init_dual_transform)

    def solve(self, current_time, update_time):
        self.init_step(current_time, update_time)
        # set initial guess, parameters, lb & ub
        var = self.father.get_variables()
        par = self.father.set_parameters(current_time)
        lb, ub = self.father.update_bounds(current_time)
        # solve!
        t0 = time.time()
        result = self.problem(x0=var, p=par, lbg=lb, ubg=ub)
        t1 = time.time()
        t_upd = t1-t0
        self.father.set_variables(result['x'])
        stats = self.problem.stats()
        if stats['return_status'] != 'Solve_Succeeded':
            print stats['return_status']
        # print
        if self.options['verbose'] >= 1:
            self.iteration += 1
            if ((self.iteration-1) % 20 == 0):
                print "----|------------|------------"
                print "%3s | %10s | %10s " % ("It", "t upd", "time")
                print "----|------------|------------"
            print "%3d | %.4e | %.4e " % (self.iteration, t_upd, current_time)
        self.update_times.append(t_upd)

    # ========================================================================
    # Plot related functions
    # ========================================================================

    def init_plot(self, argument, **kwargs):
        if argument == 'scene':
            if not hasattr(self.vehicles[0], 'signals'):
                return None
            info = self.environment.init_plot(None, **kwargs)
            labels = kwargs['labels'] if 'labels' in kwargs else [
                '' for _ in range(self.environment.n_dim)]
            n_colors = len(self.colors)
            indices = [int([''.join(g) for _, g in groupby(
                v.label, str.isalpha)][-1]) % n_colors for v in self.vehicles]
            for v in range(len(self.vehicles)):
                info[0][0]['lines'].append(
                    {'color': self.colors_w[indices[v]]})
            for v in range(len(self.vehicles)):
                info[0][0]['lines'].append({'color': self.colors[indices[v]]})
            for v, vehicle in enumerate(self.vehicles):
                for _ in vehicle.draw():
                    info[0][0]['lines'].append(
                        {'color': self.colors[indices[v]]})
            info[0][0]['labels'] = labels
            return info
        else:
            return None

    def update_plot(self, argument, t, **kwargs):
        if argument == 'scene':
            if not hasattr(self.vehicles[0], 'signals'):
                return None
            data = self.environment.update_plot(None, t, **kwargs)
            for vehicle in self.vehicles:
                data[0][0].append(
                    [vehicle.traj_storage['pose'][t][k, :] for k in range(vehicle.n_dim)])
            for vehicle in self.vehicles:
                if t == -1:
                    data[0][0].append(
                        [vehicle.signals['pose'][k, :] for k in range(vehicle.n_dim)])
                else:
                    data[0][0].append(
                        [vehicle.signals['pose'][k, :t+1] for k in range(vehicle.n_dim)])
            for vehicle in self.vehicles:
                for l in vehicle.draw(t):
                    data[0][0].append([l[k, :] for k in range(vehicle.n_dim)])
            return data
        else:
            return None

    # ========================================================================
    # Methods encouraged to override
    # ========================================================================

    def init_step(self, current_time, update_time):
        pass

    def final(self):
        pass

    def initialize(self):
        pass

    def init_primal_transform(self, basis):
        return None

    def init_dual_transform(self, basis):
        return None

    def set_parameters(self, time):
        return {}

    # ========================================================================
    # Methods required to override
    # ========================================================================

    def update(self, current_time, update_time, sample_time):
        raise NotImplementedError('Please implement this method!')

    def stop_criterium(self, current_time, update_time):
        raise NotImplementedError('Please implement this method!')

    def export(self, options=None):
        raise NotImplementedError('Please implement this method!')
