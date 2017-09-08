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
from ..basics.spline_extra import shift_spline, evalspline

from casadi import inf
import numpy as np


class GCodeProblem(Problem):
    # Problem containing several blocks of GCode
    # here environment is a local part (given by some GCode blocks) of the
    # total environment (given by the combination of all GCode blocks)
    # the shape of the segments is the same as the one of
    # the rooms in the environment
    def __init__(self, fleet, environment, segments, n_segments, options=None):
        # Todo: environment and segments partly hold the same information, improve?
        Problem.__init__(self, fleet, environment, options, label='gcodeproblem')
        self.segments = segments
        self.n_segments = n_segments
        if self.n_segments > len(self.segments):
            raise RuntimeError('Number of segments is larger than the amount of ' +
                               'GCode segments provided')
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
        self.motion_times = []  # holds motion time for each segment
        for segment in range(self.n_segments):
            self.motion_times.append(self.define_variable('T'+str(segment), value=10))

        # minimize total motion time
        self.define_objective(sum(self.motion_times))

        # positivity contraint on motion time
        for motion_time in self.motion_times:
            self.define_constraint(-motion_time, -inf, 0.)

        # collision constraints
        self.vehicles[0].init()
        # create splines with correct amount of segments, i.e. equal to n_segments
        total_splines = self.vehicles[0].define_splines(n_seg=self.n_segments)
        for idx in range(self.n_segments):
            self.vehicles[0].define_trajectory_constraints(total_splines[idx], self.motion_times[idx])
            # set up room constraints
            self.vehicles[0].define_collision_constraints(self.segments[idx], total_splines[idx], self.motion_times[idx])

        # constrain spline segments
        self.define_init_constraints()
        self.define_terminal_constraints()
        self.define_connection_constraints()

    def define_init_constraints(self):
        # place initial constraints only on first spline segment
        init_con = self.vehicles[0].get_initial_constraints(self.vehicles[0].splines[0], self.motion_times[0])
        for con in init_con:
            spline, condition = con[0], con[1]
            # use dimensionless time for first segment
            self.define_constraint(
                evalspline(spline, self.t/self.motion_times[0]) - condition, 0., 0.)

    def define_terminal_constraints(self):
        # place final constraints only on last spline segment
        term_con, term_con_der = self.vehicles[0].get_terminal_constraints(
            self.vehicles[0].splines[-1])  # select last spline segment
        if ('no_term_con_der' in self.options and self.options['no_term_con_der']):
            term_con_der = []
        for con in (term_con + term_con_der):
            spline, condition = con[0], con[1]
            self.define_constraint(spline(1.) - condition, 0., 0.)

    def define_connection_constraints(self):
        # connect splines over different frames
        # only necessary when n_segments>1
        for j in range(self.n_segments-1):
            for spline1, spline2 in zip(self.vehicles[0].splines[j], self.vehicles[0].splines[j+1]):
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

    def reinitialize(self, father=None):
        if father is None:
            father = self.father
        Problem.reinitialize(self)
        # compute initial guess for all spline values
        subgoals = []
        for k in range(self.n_segments-1):
            segment1 = self.segments[k]
            segment2 = self.segments[k+1]
            # subgoals is given as [initial position, center of overlap of regions and overall goal]
            # compute center of overlap region of the area that is shared by two subsequent segments (after taking tolerance into account)

            # Todo: change this from rooms to segments: e.g. how handle a circle arc?
            # Put subgoal on a line between the two segments? (i.e. a line with correct orientation)
            subgoals.append(compute_rectangle_overlap_center(segment1['border']['shape'], segment1['borer']['position'],
                                                             segment2['border']['shape'], segment2['border']['position']))
        init = vehicle.get_init_spline_value(subgoals = subgoals)
        for k in range(self.n_segments):
            father.set_variables(init[k], self.vehicles[0], 'splines_seg'+str(k))

    def store(self, current_time, update_time, sample_time):
        segment_times = []
        # compute total remaining motion time
        for segment in range(self.n_segments):
            segment_times.append(self.father.get_variables(self, 'T'+str(segment))[0][0])
        horizon_time = sum(segment_times)  # total horizon time
        if self.init_time is None:
            rel_current_time = 0.0
        else:
            rel_current_time = self.init_time
        if horizon_time < sample_time: # otherwise interp1d() crashes
            return
        # update tool
        n_samp = int(
            round((horizon_time-rel_current_time)/sample_time, 6)) + 1
        time_axis = np.linspace(rel_current_time, rel_current_time + (n_samp-1)*sample_time, n_samp)
        spline_segments = [self.father.get_variables(self.vehicles[0], 'splines_seg'+str(k)) for k in range(self.vehicles[0].n_seg)]
        self.vehicles[0].store(current_time, sample_time, spline_segments, segment_times, time_axis)

    def reset_init_time(self):
        self.init_time = None

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def simulate(self, current_time, simulation_time, sample_time):

        # Todo: how simulate and shift one segment at a time?


        horizon_time = 0
        # compute total remaining motion time
        for segment in range(self.n_segments):
            horizon_time += self.father.get_variables(self, 'T'+str(segment))[0][0]
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
        self.vehicles[0].simulate(simulation_time, sample_time)
        # self.environment.simulate(simulation_time, sample_time)
        self.vehicles[0].update_plots()
        self.update_plots()

    def stop_criterium(self, current_time, update_time):
        T_tot = 0
        # compute total remaining motion time
        for k in range(self.n_segments):
            T_tot += self.father.get_variables(self, 'T'+str(k))[0][0]
        if T_tot < update_time:
            return True
        stop = True
        stop *= self.vehicles[0].check_terminal_conditions()
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
            for segment in range(self.n_segments):
                T += self.father.get_variables(self, 'T'+str(segment))[0][0]
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
