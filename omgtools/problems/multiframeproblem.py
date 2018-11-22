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
from point2point import Point2pointProblem
from ..basics.optilayer import OptiFather, OptiChild
from ..vehicles.fleet import get_fleet_vehicles
from ..execution.plotlayer import PlotLayer
from ..basics.spline_extra import definite_integral
from ..basics.spline_extra import shift_spline, evalspline
from ..basics.geometry import compute_rectangle_overlap_center
from casadi import inf
import numpy as np


class MultiFrameProblem(Problem):

    # Problem consisting of multiple frames, connecting multiple spline segments
    def __init__(self, fleet, environment, n_frames, options=None):
        Problem.__init__(self, fleet, environment, options, label='multiframeproblem')
        self.n_frames = n_frames
        if self.n_frames > len(self.environment.room):
            raise RuntimeError('Number of frames you want to consider at once is larger' +
                               'than the amount of rooms provided in environment')
        self.init_time = None
        self.start_time = 0.
        self.objective = 0.

    def set_default_options(self):
        Problem.set_default_options(self)

    def set_options(self, options):
        Problem.set_options(self, options)

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def construct(self):
        self.t = self.define_parameter('t')
        self.motion_times = []  # holds motion time through each frame
        for frame in range(self.n_frames):
            self.motion_times.append(self.define_variable('T'+str(frame), value=10))

        # positivity contraint on motion time
        for motion_time in self.motion_times:
            self.define_constraint(-motion_time, -inf, 0.)

        # collision constraints
        # self.environment.init()
        for vehicle in self.vehicles:
            vehicle.init()
            # create splines with correct amount of segments, i.e. a segment per frame
            total_splines = vehicle.define_splines(n_seg=self.n_frames)
            for frame in range(self.n_frames):
                vehicle.define_trajectory_constraints(total_splines[frame], self.motion_times[frame])
            self.environment.define_collision_constraints(vehicle, total_splines, self.motion_times)
        if len(self.vehicles) > 1 and self.options['inter_vehicle_avoidance']:
            self.environment.define_intervehicle_collision_constraints(self.vehicles)

        # constrain spline segments
        self.define_init_constraints()
        self.define_terminal_constraints()
        self.define_connection_constraints()

        # minimize total motion time
        obj = sum(self.motion_times)
        # add regularization on jerk to avoid nervous solutions
        # if self.n_frames > 1:
        #     for frame in range(self.n_frames):
        #         for s in total_splines[frame]:
        #             dds = s.derivative(3)
        #             obj += definite_integral((0.01*dds)**2, 0., 1.)
        self.define_objective(obj)

    def define_init_constraints(self):
        # place initial constraints only on first spline segment
        for vehicle in self.vehicles:
            init_con = vehicle.get_initial_constraints(vehicle.splines[0], self.motion_times[0])
            for con in init_con:
                spline, condition = con[0], con[1]
                # use dimensionless time for first segment
                self.define_constraint(
                    evalspline(spline, self.t/self.motion_times[0]) - condition, 0., 0.)

    def define_terminal_constraints(self):
        # place final constraints only on last spline segment
        for vehicle in self.vehicles:
            term_con, term_con_der = vehicle.get_terminal_constraints(
                vehicle.splines[-1], horizon_time=self.motion_times[-1])  # select last spline segment
            if ('no_term_con_der' in self.options and self.options['no_term_con_der']):
                term_con_der = []
            for con in (term_con + term_con_der):
                spline, condition = con[0], con[1]
                self.define_constraint(spline(1.) - condition, 0., 0.)

    def define_connection_constraints(self):
        # connect splines over different frames
        # only necessary when n_frames>1
        for j in range(self.n_frames-1):
            for vehicle in self.vehicles:
                for spline1, spline2 in zip(vehicle.splines[j], vehicle.splines[j+1]):
                    for d in range(spline1.basis.degree):
                        # in connection point splines should be equal until derivative of order degree-1
                        # give dimensions by multplication with the motion time
                        self.define_constraint(
                            evalspline(spline1.derivative(d), 1)*self.motion_times[j+1]**d -
                            evalspline(spline2.derivative(d), 0)*self.motion_times[j]**d, 0., 0.)

    def set_parameters(self, current_time):
        parameters = Problem.set_parameters(self, current_time)
        # current time is always 0 for FreeT problem, time axis always resets
        if self.init_time is None:
            parameters[self]['t'] = 0
        else:
            parameters[self]['t'] = self.init_time
        return parameters

    # ========================================================================
    # Deploying related functions
    # ========================================================================

    def initialize(self, current_time):
        self.start_time = current_time

    def reinitialize(self, father=None):
        if father is None:
            father = self.father
        Problem.reinitialize(self)
        for vehicle in self.vehicles:
            # compute initial guess for all spline values
            subgoals = []
            for k in range(self.n_frames-1):
                room1 = self.environment.room[k]
                room2 = self.environment.room[k+1]
                # subgoals is given as initial position, center of overlap of regions and overall goal
                # compute center of overlap region of rooms
                subgoals.append(compute_rectangle_overlap_center(room1['shape'], room1['position'], room2['shape'], room2['position']))
            init = vehicle.get_init_spline_value(subgoals = subgoals)
            for k in range(self.n_frames):
                father.set_variables(init[k], vehicle, 'splines_seg'+str(k))

    def store(self, current_time, update_time, sample_time):
        segment_times = []
        # compute total remaining motion time
        for frame in range(self.n_frames):
            segment_times.append(self.father.get_variables(self, 'T'+str(frame))[0][0])
        horizon_time = sum(segment_times)  # total horizon time
        if self.init_time is None:
            rel_current_time = 0.0
        else:
            rel_current_time = self.init_time
        if horizon_time < sample_time: # otherwise interp1d() crashes
            return
        # update vehicles
        for vehicle in self.vehicles:
            n_samp = int(
                round((horizon_time-rel_current_time)/sample_time, 6)) + 1
            time_axis = np.linspace(rel_current_time, rel_current_time + (n_samp-1)*sample_time, n_samp)
            spline_segments = [self.father.get_variables(vehicle, 'splines_seg'+str(k)) for k in range(vehicle.n_seg)]
            vehicle.store(current_time, sample_time, spline_segments, segment_times, time_axis)

    def reset_init_time(self):
        self.init_time = None

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def simulate(self, current_time, simulation_time, sample_time):
        horizon_time = 0
        # compute total remaining motion time
        for frame in range(self.n_frames):
            horizon_time += self.father.get_variables(self, 'T'+str(frame))[0][0]
        if self.init_time is None:
            rel_current_time = 0.0
        else:
            rel_current_time = self.init_time
        if horizon_time < sample_time: # otherwise interp1d() crashes
            return
        if horizon_time < simulation_time:
            simulation_time = horizon_time
        if horizon_time - rel_current_time < simulation_time:
            simulation_time = horizon_time - rel_current_time
        self.compute_partial_objective(current_time+simulation_time-self.start_time)
        for vehicle in self.vehicles:
            vehicle.simulate(simulation_time, sample_time)
        self.environment.simulate(simulation_time, sample_time)
        self.fleet.update_plots()
        self.update_plots()

    def stop_criterium(self, current_time, update_time):
        T_tot = 0
        # compute total remaining motion time
        for k in range(self.n_frames):
            T_tot += self.father.get_variables(self, 'T'+str(k))[0][0]
        if T_tot < update_time:
            return True
        stop = True
        for vehicle in self.vehicles:
            stop *= vehicle.check_terminal_conditions()
        return stop

    def final(self):
        self.reset_init_time()
        obj = self.compute_objective()
        if self.options['verbose'] >= 1:
            print '\nWe reached our target!'
            print '%-18s %6g' % ('Objective:', obj)
            print '%-18s %6g ms' % ('Max update time:',
                                    max(self.update_times)*1000.)
            print '%-18s %6g ms' % ('Av update time:',
                                    (sum(self.update_times)*1000. /
                                     len(self.update_times)))

    def init_step(self, current_time, update_time):
        if (current_time - self.start_time) > 0:
            # compute total remaining motion time
            T = 0
            for frame in range(self.n_frames):
                T += self.father.get_variables(self, 'T'+str(frame))[0][0]
            # check if almost arrived, if so lower the update time
            if T < 2*update_time:
                update_time = T - update_time
                target_time = T
            else:
                target_time = T - update_time
            # create spline which starts from the position at update_time and goes
            # to goal position at target_time. Approximate/Represent this spline in
            # a new basis with new equidistant knots.

            # shifting spline is only required for first segment (index 0), so seg_shift=[0]
            self.father.transform_primal_splines(
                lambda coeffs, basis: shift_spline(coeffs, update_time/target_time, basis), seg_shift=[0])
            T_0 = self.father.get_variables(self, 'T'+str(0))[0][0]  # remaining motion time for first segment
            self.father.set_variables(T_0-update_time, self, 'T0')  # only change time of first segment

    def compute_partial_objective(self, current_time):
        self.objective = current_time

    def compute_objective(self):
        return self.objective
