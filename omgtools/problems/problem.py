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
from ..basics.geometry import distance_between_points, point_in_polyhedron
from ..basics.geometry import circle_polyhedron_intersection
from ..basics.geometry import rectangles_overlap
from ..basics.shape import Circle, Polyhedron, Rectangle, Square
from ..basics.shape import Circle, Rectangle
from ..vehicles.fleet import get_fleet_vehicles
from ..execution.plotlayer import PlotLayer
from itertools import groupby
import numpy as np
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
        ipopt_options = {'ipopt.tol': 1e-3,
                         'ipopt.warm_start_init_point': 'yes',
                         'ipopt.print_level': 0, 'print_time': 0,
                         'ipopt.fixed_variable_treatment':'make_constraint'}
        self.options['solver_options'] = {'ipopt': ipopt_options}
        self.options['codegen'] = {'build': None, 'flags': '-O0'}

    def set_options(self, options):
        if 'solver_options' in options:
            for key, value in options['solver_options'].items():
                if key not in self.options['solver_options']:
                    self.options['solver_options'][key] = {}
                self.options['solver_options'][key].update(value)
        if 'codegen' in options:
            self.options['codegen'].update(options['codegen'])
        for key in options:
            if key not in ['solver_options', 'codegen']:
                self.options[key] = options[key]

    # ========================================================================
    # Create problem
    # ========================================================================

    def construct(self):
        self.environment.init()
        for vehicle in self.vehicles:
            vehicle.init()

    def init(self):
        self.father.reset()
        self.construct()
        self.problem, buildtime = self.father.construct_problem(self.options)
        self.father.init_transformations(self.init_primal_transform,
                                         self.init_dual_transform)
        return buildtime

    # ========================================================================
    # Deploying related functions
    # ========================================================================

    def reinitialize(self, father=None):
        if father is None:
            father = self.father
        father.init_variables()
        father.init_parameters()

    def solve(self, current_time, update_time):
        current_time -= self.start_time  # start_time: the point in time where you start solving
        self.init_step(current_time, update_time)  # pass on update_time to make initial guess
        self.update_vehicle_limits()  # used to change the velocity limits
        # set initial guess, parameters, lb & ub
        var = self.father.get_variables()
        dual_var = self.father.get_dual_variables()
        par = self.father.set_parameters(current_time)
        lb, ub = self.father.update_bounds(current_time)
        # solve!
        t0 = time.time()
        # result = self.problem(x0=var, p=par, lbg=lb, ubg=ub)
        result = self.problem(x0=var, lam_g0= dual_var, p=par, lbg=lb, ubg=ub)
        t1 = time.time()
        t_upd = t1-t0
        self.father.set_variables(result['x'])
        self.father.set_dual_variables(result['lam_g'])
        stats = self.problem.stats()
        if stats['return_status'] != 'Solve_Succeeded':
            if stats['return_status'] == 'Maximum_CpuTime_Exceeded':
                if current_time != 0.0:  # first iteration can be slow, neglect time here
                    print 'Maximum solving time exceeded, resetting initial guess'
                    self.reset_init_guess()
                    print stats['return_status']
            else:
                # there was another problem
                print stats['return_status']
        if self.options['verbose'] >= 2:
            self.iteration += 1
            if ((self.iteration-1) % 20 == 0):
                print "----|------------|------------"
                print "%3s | %10s | %10s " % ("It", "t upd", "time")
                print "----|------------|------------"
            print "%3d | %.4e | %.4e " % (self.iteration, t_upd, current_time)
        self.update_times.append(t_upd)

    def predict(self, current_time, predict_time, sample_time, states=None, inputs=None, dinputs=None, delay=0, enforce_states=False, enforce_inputs=False):
        if states is None:
            states = [None for k in range(len(self.vehicles))]
        if inputs is None:
            inputs = [None for k in range(len(self.vehicles))]
        if dinputs is None:
            dinputs = [None for k in range(len(self.vehicles))]
        if len(self.vehicles) == 1:
            if not isinstance(states, list):
                states = [states]
            elif isinstance(states[0], float):
                states = [states]
        if len(self.vehicles) == 1:
            if not isinstance(inputs, list):
                inputs = [inputs]
            elif isinstance(inputs[0], float):
                inputs = [inputs]
        if len(self.vehicles) == 1:
            if not isinstance(dinputs, list):
                dinputs = [dinputs]
            elif isinstance(dinputs[0], float):
                dinputs = [dinputs]
        if current_time == self.start_time:
            enforce_states = True
        for k, vehicle in enumerate(self.vehicles):
            vehicle.predict(current_time, predict_time, sample_time, states[k], inputs[k], dinputs[k], delay, enforce_states, enforce_inputs)

    def reset_init_guess(self, init_guess=None):
            if init_guess is None:  # no user provided initial guess
                init_guess = []
                for k, vehicle in enumerate(self.vehicles):  # build list
                    init_guess.append(vehicle.get_init_spline_value())
            elif not isinstance(init_guess, list):  # guesses must be in a list
                init_guess = [init_guess]

            for k, vehicle in enumerate(self.vehicles):
                if len(init_guess) != vehicle.n_seg:
                    raise ValueError('Each spline segment of the vehicle should receive an initial guess.')
                else:
                    for l in range(vehicle.n_seg):
                        if init_guess[l].shape[1] != vehicle.n_spl:
                            raise ValueError('Each vehicle spline should receive an initial guess.')
                        else:
                            self.father.set_variables(init_guess[l].tolist(),child=vehicle, name='splines_seg'+str(l))

    def update_vehicle_limits(self):
        # check if the vehicle is in a zone that requires reduced speed, if so change its limits
        if hasattr(self.environment, 'danger_zones'):
            if hasattr(self.vehicles[0], 'signals'):
                veh_pos = self.vehicles[0].signals['state'][:,-1]
            else:
                veh_pos = self.vehicles[0].prediction['state']  # first iteration
            # initialize variables to hold new velocity limits
            vxmin, vymin, vxmax, vymax = [], [], [], []
            vmax = []
            in_danger_zone = False
            for zone in self.environment.danger_zones:
                # check distance between current vehicle position and dangerzone
                zone_pos = zone.signals['position'][:,-1]
                if self.environment.shapes_overlap(self.vehicles[0].shapes[0], veh_pos, zone.shape, zone_pos):
                    for vehicle in self.vehicles:
                        # vehicle is supposed to be holonomic
                        # immediately reduce bounds to minimum value
                        # but check if the new limit is more strict than the old one, choose most strict one
                        if self.vehicles[0].options['syslimit'] == 'norm_inf':
                            if not vxmin or zone.bounds['vxmin'] > vxmin:
                                vxmin = zone.bounds['vxmin']
                            if not vymin or zone.bounds['vymin'] > vymin:
                                vymin = zone.bounds['vymin']
                            if not vxmax or zone.bounds['vxmax'] < vxmax:
                                vxmax = zone.bounds['vxmax']
                            if not vymax or zone.bounds['vymax'] < vymax:
                                vymax = zone.bounds['vymax']
                            vel_limits = [vxmin, vymin, vxmax, vymax]
                        else:
                            if not vmax or zone.bounds['vmax'] < vmax:
                                vmax = zone.bounds['vmax']
                            vel_limits = [vmax]
                        vehicle.set_velocities(vel_limits)  # xmin,ymin,xmax,ymax
                        in_danger_zone = True
            if not in_danger_zone:
                # if not in zone, set back to original limits
                # such that limits are re-set if you leave the zone
                for vehicle in self.vehicles:
                    if self.vehicles[0].options['syslimit'] == 'norm_inf':
                        vxmin = vehicle.original_bounds['vxmin']
                        vymin = vehicle.original_bounds['vymin']
                        vxmax = vehicle.original_bounds['vxmax'] 
                        vymax = vehicle.original_bounds['vymax']
                        vel_limits = [vxmin, vymin, vxmax, vymax]
                    else:
                        vmax = vehicle.original_bounds['vmax']
                        vel_limits = [vmax]
                    vehicle.set_velocities(vel_limits)  # xmin,ymin,xmax,ymax
        else:
            # there were no danger zones so keep the original limits
            pass

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def simulate(self, current_time, simulation_time, sample_time):
        for vehicle in self.vehicles:
            vehicle.simulate(simulation_time, sample_time)
        self.environment.simulate(simulation_time, sample_time)
        self.fleet.update_plots()
        self.update_plots()

    def sleep(self, current_time, sleep_time, sample_time):
        # update vehicles
        for vehicle in self.vehicles:
            spline_segments = [self.father.get_variables(
                vehicle, 'splines_seg'+str(k)) for k in range(vehicle.n_seg)]
            spline_values = vehicle.signals['splines'][:, -1]
            spline_values = [self.father.get_variables(
                vehicle, 'splines_seg'+str(k), spline=False)[-1, :] for k in range(vehicle.n_seg)]
            for segment, values in zip(spline_segments, spline_values):
                for spl, value in zip(segment, values):
                    spl.coeffs = value*np.ones(len(spl.basis))
            vehicle.store(current_time, sample_time, spline_segments, sleep_time)
        # no correction for update time!
        Problem.simulate(self, current_time, sleep_time, sample_time)

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
                info[0][0]['lines'] += [{'color': self.colors_w[indices[v]]}]
            for v in range(len(self.vehicles)):
                info[0][0]['lines'] += [{'color': self.colors[indices[v]]}]
            for v, vehicle in enumerate(self.vehicles):
                s, l = vehicle.draw()
                info[0][0]['lines'] += [{'color': self.colors[indices[v]]} for _ in l]
                info[0][0]['surfaces'] += [{'facecolor': self.colors_w[indices[v]],
                    'edgecolor': self.colors[indices[v]], 'linewidth': 1.2} for _ in s]
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
                data[0][0]['lines'] += [vehicle.traj_storage['pose'][t][:3, :]]
            for vehicle in self.vehicles:
                if t == -1:
                    data[0][0]['lines'] += [vehicle.signals['pose'][:3, :]]
                else:
                    data[0][0]['lines'] += [vehicle.signals['pose'][:3, :t+1]]
            for vehicle in self.vehicles:
                surfaces, lines = vehicle.draw(t)
                data[0][0]['surfaces'] += surfaces
                data[0][0]['lines'] += lines
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

    def initialize(self, current_time):
        pass

    def init_primal_transform(self, basis):
        return None

    def init_dual_transform(self, basis):
        return None

    def set_parameters(self, time):
        return {self: {}}

    # ========================================================================
    # Methods required to override
    # ========================================================================

    def update(self, current_time, update_time, sample_time):
        raise NotImplementedError('Please implement this method!')

    def store(self, current_time, update_time, sample_time):
        raise NotImplementedError('Please implement this method!')

    def stop_criterium(self, current_time, update_time):
        raise NotImplementedError('Please implement this method!')

    def export(self, options=None):
        raise NotImplementedError('Please implement this method!')
