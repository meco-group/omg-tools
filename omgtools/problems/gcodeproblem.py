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

from .problem import Problem
from ..basics.spline_extra import shift_spline, evalspline

from casadi import inf
import numpy as np


class GCodeProblem(Problem):
    # Problem containing several blocks of GCode
    # here environment is a local part (given by some GCode blocks) of the
    # total environment (given by the combination of all GCode blocks)
    # the rooms represent the GCode segments
    def __init__(self, fleet, environment, n_segments, options=None, **kwargs):
        Problem.__init__(self, fleet, environment, options, label='gcodeproblem')
        self.n_segments = n_segments  # amount of GCode commands to connect
        if self.n_segments > len(self.environment.room):
            raise RuntimeError('Number of segments to combine is larger than the amount of ' +
                               'GCode segments/rooms provided')
        self.init_time = None
        self.start_time = 0.
        self.objective = 0.

        # an initial guess for the motion time may be passed by the scheduler during problem creation
        if 'motion_time_guess' in kwargs:
            self.motion_time_guess = kwargs['motion_time_guess']

    def set_default_options(self):
        Problem.set_default_options(self)

    def set_options(self, options):
        Problem.set_options(self, options)

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def construct(self):
        self.t = self.define_parameter('t')
        self.motion_times = []  # holds motion time for each room
        for room in range(self.n_segments):
            self.motion_times.append(self.define_variable('T'+str(room), value=10))

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
            if idx == 0:
                self.vehicles[0].define_trajectory_constraints(total_splines[idx], self.motion_times[idx], skip=[1,0])
            elif idx == self.n_segments-1:
                self.vehicles[0].define_trajectory_constraints(total_splines[idx], self.motion_times[idx], skip=[0,1])
            else:
                self.vehicles[0].define_trajectory_constraints(total_splines[idx], self.motion_times[idx], skip=[])
            # set up room constraints
            self.vehicles[0].define_collision_constraints(self.environment.room[idx], total_splines[idx], self.motion_times[idx])

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
        # connect splines over different segments
        # only necessary when n_segments>1

        # in connection point splines should be equal until derivative of order degree-1
        # suppose that all splines have the same degree
        self.continuity = self.vehicles[0].splines[0][0].basis.degree  # save desired continuity
        for j in range(self.n_segments-1):
            for spline1, spline2 in zip(self.vehicles[0].splines[j], self.vehicles[0].splines[j+1]):
                for d in range(self.continuity):
                    # give dimensions by multiplication with the motion time
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
        self.vehicles[0].store(current_time, sample_time, spline_segments, segment_times, time_axis, continuity=self.continuity)

    def reset_init_time(self):
        self.init_time = None

    # ========================================================================
    # Simulation related functions
    # ========================================================================

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
            print('\nWe reached our target!')
            print('%-18s %6g' % ('Objective:', obj))
            print('%-18s %6g ms' % ('Max update time:',
                                    max(self.update_times)*1000.))
            print('%-18s %6g ms' % ('Av update time:',
                                    (sum(self.update_times)*1000. /
                                     len(self.update_times))))

    def init_step(self, current_time, update_time):
        # set guess for motion time, that was passed on by the scheduler
        if hasattr(self, 'motion_time_guess'):
            T_guess = self.motion_time_guess
        else:
            T_guess = []
            for idx in range(self.n_segments):
                T_guess.append(self.father.get_variables(self, 'T'+str(idx))[0][0])
        for idx in range(self.n_segments):
            self.father.set_variables(T_guess[idx], self, 'T'+str(idx))

        # since we are simulating per segment, no further shifting of splines is required
        # if you want to simulate with a receding horizon, add:
        # self.father.transform_primal_splines(
        #         lambda coeffs, basis: shift_spline(coeffs, update_time/T_guess[0], basis))

    def compute_partial_objective(self, current_time):
        self.objective = current_time

    def compute_objective(self):
        return self.objective
