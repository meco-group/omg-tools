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

from problem_HolonomicF import Problem_HolonomicF
from ..basics.spline_extra import definite_integral
from ..basics.spline_extra import shiftoverknot_T, shift_spline, evalspline
from ..export.export_p2p import ExportP2P
from casadi import inf
import numpy as np


class DistributedPayload(object):
    # this class selects between fixed T and free T problem

    def __new__(cls, fleet, environment, options=None, freeT=False):
        if freeT:
            return FreeTDistributedPayload(fleet, environment, options)
        else:
            return FixedTDistributedPayload(fleet, environment, options)


class DistributedPayloadProblem(Problem_HolonomicF):

    def __init__(self, fleet, environment, options):
        Problem_HolonomicF.__init__(self, fleet, environment, options, label='distPyld')
        self.init_time = None
        self.start_time = 0.

    def set_init_time(self, time):
        self.init_time = time

    def reset_init_time(self):
        self.init_time = None

    def construct(self):
        self.T, self.t = self.define_parameter('T'), self.define_parameter('t')
        self.t0 = self.t/self.T
        Problem_HolonomicF.construct(self)
        for vehicle in self.vehicles:
            splines = vehicle.define_splines(n_seg=1)[0]
            vehicle.assign_payload_accn_splines(splines)
            vehicle.define_trajectory_constraints(splines)
            self.environment.define_collision_constraints(vehicle, splines)

    def reinitialize(self, father=None):
        if father is None:
            father = self.father
        Problem_HolonomicF.reinitialize(self)
        for vehicle in self.vehicles:
            init = vehicle.get_init_spline_value()
            father.set_variables(init, vehicle, 'splines0')

    def define_init_constraints(self):
            #As we have a problem with initial velocities, we can rather define them
            #as objective, similar to terminal_constraints
            #We just add (spline[t0]-vx)*(spline[t0]-vx) to the objective
            #con[2] now tells if it is a soft constraint or a hard constraint
            objective = 0.
            for vehicle in self.vehicles:
                init_con = vehicle.get_initial_constraints(vehicle.splines[0])

                for con in init_con:
                    spline, condition, con_type = con[0], con[1], con[2]
                    if con_type == 'hard':
                        self.define_constraint(
                            evalspline(spline, self.t0) - condition, 0., 0.)
                    elif con_type == 'soft':
                        objective+= vehicle.tuning_weights[2]*(evalspline(spline, self.t0)-condition)*(evalspline(spline, self.t0)-condition)
                        #As the value of velocity is low, have to give high weightage
                    else:
                        print 'No initial constraint on force input'
            self.define_objective(objective)


    def stop_criterium(self, current_time, update_time):
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

    def compute_objective(self):
        raise NotImplementedError('Please implement this method!')

    def export(self, options=None):
        options = options or {}
        if not hasattr(self, 'father'):
            self.init()
        ExportP2P(self, options)

    def sleep(self, current_time, sleep_time, sample_time):
        # update vehicles
        for vehicle in self.vehicles:
            spline_segments = [self.father.get_variables(vehicle, 'splines'+str(k)) for k in range(vehicle.n_seg)]
            spline_values = vehicle.signals['splines'][:, -1]
            spline_values = [self.father.get_variables(vehicle, 'splines'+str(k), spline=False)[-1, :] for k in range(vehicle.n_seg)]
            for segment, values in zip(spline_segments, spline_values):
                for spl, value in zip(segment, values):
                    spl.coeffs = value*np.ones(len(spl.basis))
            vehicle.update(current_time, sleep_time, sample_time, spline_segments, sleep_time)
        # update plots
        self.environment.update(sleep_time, sample_time)
        # update plots
        self.update_plots()

class FixedTDistributedPayload(DistributedPayloadProblem):

    def __init__(self, fleet, environment, options):
        DistributedPayloadProblem.__init__(self, fleet, environment, options)
        self.objective = 0.
        if self.vehicles[0].knot_intervals is None:
            raise ValueError('A constant knot interval should be used for ' +
                             'a fixed T point2point problem.')
        self.knot_time = (int(self.options['horizon_time']*1000.) /
                          self.vehicles[0].knot_intervals) / 1000.

    def set_default_options(self):
        DistributedPayloadProblem.set_default_options(self)
        self.options['horizon_time'] = 10.
        self.options['hard_term_con'] = False

    def construct(self):
        DistributedPayloadProblem.construct(self)
        self.define_init_constraints()
        self.define_terminal_constraints()
        self.define_additional_objectives()

    def define_additional_objectives(self):
        objective = 0. #To minimize deviation between approximate and actual accleeration of payload (Dynamics)
        for vehicle in self.vehicles:
            x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx = vehicle.splines[0]

            ax10_actual = x10.derivative(2)
            ay10_actual = y10.derivative(2)

            expr = vehicle.tuning_weights[0]*(ax10_actual-ax10_approx)*(ax10_actual-ax10_approx)
            objective+= definite_integral(expr, self.t0, 1.)
            expr = vehicle.tuning_weights[0]*(ay10_actual-ay10_approx)*(ay10_actual-ay10_approx)
            objective+= definite_integral(expr, self.t0, 1.)
            expr = vehicle.tuning_weights[4]*(ax10_actual*ax10_actual)
            objective+= definite_integral(expr, self.t0, 1.)
            expr = vehicle.tuning_weights[4]*(ay10_actual*ay10_actual)
            objective+= definite_integral(expr, self.t0, 1.)
        self.define_objective(objective)

    def define_terminal_constraints(self):
        objective = 0.
        #Split into soft final position constraints and hard final derivative constraints
        for vehicle in self.vehicles:
            term_con, term_con_der = vehicle.get_terminal_constraints(
                vehicle.splines[0])
            for k, con in enumerate(term_con):
                spline, condition = con[0], con[1]
                #This is the L1 norm spline - This is set to all the variables, not just payload position
                g = self.define_spline_variable(
                    'g'+str(k), 1, basis=spline.basis)[0]
                objective+= vehicle.tuning_weights[1]*definite_integral(g, self.t0, 1.)
                self.define_constraint(spline - condition - g, -inf, 0.)
                self.define_constraint(-spline + condition - g, -inf, 0.)
                if self.options['hard_term_con']:
                    self.define_constraint(spline(1.) - condition, 0., 0.)
            for con in term_con_der:
                spline, condition = con[0], con[1]
                self.define_constraint(spline(1.) - condition, 0., 0.)
        self.define_objective(objective)

    def set_parameters(self, current_time):
        parameters = DistributedPayloadProblem.set_parameters(self, current_time)
        if self.init_time is None:
            parameters[self]['t'] = np.round(current_time, 6) % self.knot_time
        else:
            parameters[self]['t'] = self.init_time
        parameters[self]['T'] = self.options['horizon_time']
        return parameters

    def init_step(self, current_time, update_time):
        # transform spline variables
        if (current_time > 0. and np.round(current_time, 6) % self.knot_time == 0):
            self.father.transform_primal_splines(lambda coeffs, basis, T:
                                                 T.dot(coeffs))
            # self.father.transform_dual_splines(lambda coeffs, basis, T:
            #                                    T.dot(coeffs))

    def init_primal_transform(self, basis):
        return shiftoverknot_T(basis)

    # def init_dual_transform(self, basis):
    #     B = integral_sqbasis(basis)
    #     Binv = np.linalg.solve(B, np.eye(len(basis)))
    #     T = shiftoverknot_T(basis)
    #     return B.dot(T).dot(Binv)

    def initialize(self, current_time):
        self.start_time = current_time


    def update(self, current_time, update_time, sample_time):
        horizon_time = self.options['horizon_time']
        if self.init_time is None:
            # y_coeffs represents coefficients of a spline, for which a part of
            # its time horizon lies in the past. Therefore, we need to pass the
            # current time relatively to the begin of this time horizon. In this
            # way, only the future, relevant, part will be saved/plotted.
            rel_current_time = np.round(current_time-self.start_time, 6) % self.knot_time
        else:
            rel_current_time = self.init_time
        if horizon_time - rel_current_time < update_time:
            update_time = horizon_time - rel_current_time
        self.compute_partial_objective(current_time, update_time)
        # update vehicles
        for vehicle in self.vehicles:
            n_samp = int(
                round((horizon_time-rel_current_time)/sample_time, 3)) + 1
            time_axis = np.linspace(rel_current_time, rel_current_time + (n_samp-1)*sample_time, n_samp)
            spline_segments = [self.father.get_variables(vehicle, 'splines'+str(k)) for k in range(vehicle.n_seg)]
            vehicle.update(
                current_time, update_time, sample_time, spline_segments, horizon_time, time_axis)
        # update environment
        self.environment.update(update_time, sample_time)
        # update plots
        self.update_plots()

    def compute_partial_objective(self, current_time, update_time):
        rel_current_time = np.round(current_time-self.start_time, 6) % self.knot_time
        horizon_time = self.options['horizon_time']
        t0 = rel_current_time/horizon_time
        t1 = t0 + update_time/horizon_time
        part_objective = 0.
        for vehicle in self.vehicles:
            term_con, _ = vehicle.get_terminal_constraints(
                vehicle.splines[0])
            for k in range(len(term_con)):
                g = self.father.get_variables(self, 'g'+str(k))[0]
                part_objective += horizon_time*definite_integral(g, t0, t1)
        self.objective += part_objective

    def compute_objective(self):
        if self.objective == 0:
            obj = 0.
            for vehicle in self.vehicles:
                term_con, _ = vehicle.get_terminal_constraints(
                    vehicle.splines[0])
                for k in range(len(term_con)):
                    g = self.father.get_variables(self, 'g'+str(k))[0]
                    obj += self.options['horizon_time']*g.integral()
            return obj
        return self.objective

    # ========================================================================
    # Deploying related functions
    # ========================================================================
    def store(self, current_time, update_time, sample_time):
        horizon_time = self.options['horizon_time']
        if self.init_time is None:
            # y_coeffs represents coefficients of a spline, for which a part of
            # its time horizon lies in the past. Therefore, we need to pass the
            # current time relatively to the begin of this time horizon. In this
            # way, only the future, relevant, part will be saved/plotted.
            rel_current_time = np.round(current_time-self.start_time, 6) % self.knot_time
        else:
            rel_current_time = self.init_time
        # store trajectories in vehicles
        for vehicle in self.vehicles:
            n_samp = int(
                round((horizon_time-rel_current_time)/sample_time, 6)) + 1
            time_axis = np.linspace(rel_current_time, rel_current_time + (n_samp-1)*sample_time, n_samp)
            spline_segments = [self.father.get_variables(vehicle, 'splines'+str(k)) for k in range(vehicle.n_seg)]
            vehicle.store(current_time, sample_time, spline_segments, horizon_time, time_axis)

class FreeTDistributedPayload(DistributedPayloadProblem):

    def __init__(self, fleet, environment, options):
        DistributedPayloadProblem.__init__(self, fleet, environment, options)
        self.objective = 0.

    def construct(self):
        DistributedPayloadProblem.construct(self)
        T = self.define_variable('T', value=10)
        t = self.define_parameter('t')
        self.t0 = t/T
        self.define_objective(T)
        # positivity contraint on motion time
        self.define_constraint(-T, -inf, 0.)
        self.define_init_constraints()
        self.define_terminal_constraints()

    def define_terminal_constraints(self):
        for vehicle in self.vehicles:
            term_con, term_con_der = vehicle.get_terminal_constraints(
                vehicle.splines[0])
            for con in (term_con + term_con_der):
                spline, condition = con[0], con[1]
                self.define_constraint(spline(1.) - condition, 0., 0.)

    def set_parameters(self, current_time):
        parameters = DistributedPayloadProblem.set_parameters(self, current_time)
        # current time is always 0 for FreeT problem, time axis always resets
        if self.init_time is None:
            parameters[self]['t'] = 0
        else:
            parameters[self]['t'] = self.init_time
        return parameters

    def update(self, current_time, update_time, sample_time):
        self.update_time = update_time
        horizon_time = self.father.get_variables(self, 'T')[0][0]
        if self.init_time is None:
            rel_current_time = 0.0
        else:
            rel_current_time = self.init_time
        if horizon_time < sample_time: #otherwise interp1d() crashes
            return
        if horizon_time < update_time:
            update_time = horizon_time
        if horizon_time - rel_current_time < update_time:
            update_time = horizon_time - rel_current_time
        self.compute_partial_objective(current_time+update_time)
        # update vehicles
        for vehicle in self.vehicles:
            n_samp = int(
                round((horizon_time-rel_current_time)/sample_time, 3)) + 1
            time_axis = np.linspace(rel_current_time, rel_current_time + (n_samp-1)*sample_time, n_samp)
            spline_segments = [self.father.get_variables(vehicle, 'splines'+str(k)) for k in range(vehicle.n_seg)]
            vehicle.update(
                current_time, update_time, sample_time, spline_segments, horizon_time, time_axis)
        # update environment
        self.environment.update(update_time, sample_time)
        # update plots
        self.update_plots()

    def stop_criterium(self, current_time, update_time):
        T = self.father.get_variables(self, 'T')[0][0]
        if T < update_time:
            return True
        return DistributedPayloadProblem.stop_criterium(self, current_time, update_time)

    def init_step(self, current_time, update_time):
        T = self.father.get_variables(self, 'T')[0][0]
        # check if almost arrived, if so lower the update time
        if T < 2*update_time:
            update_time = T - update_time
            target_time = T
        else:
            target_time = T - update_time
        # create spline which starts from the position at update_time and goes
        # to goal position at target_time. Approximate/Represent this spline in
        # a new basis with new equidistant knots.
        self.father.transform_primal_splines(
            lambda coeffs, basis: shift_spline(coeffs, update_time/target_time, basis))
        self.father.set_variables(target_time, self, 'T')

    def compute_partial_objective(self, current_time):
        self.objective = current_time

    def compute_objective(self):
        return self.objective

class FreeEndDistributedPayload(FixedTDistributedPayload):

    def __init__(self, fleet, environment, options, free_ind=None):
        FixedTDistributedPayload.__init__(self, fleet, environment, options)
        self.free_ind = free_ind

    def construct(self):
        if self.free_ind is None:
            self.free_ind = {}
            for vehicle in self.vehicles:
                term_con = vehicle.get_terminal_constraints(vehicle.splines[0])
                self.free_ind[vehicle] = range(len(term_con))
        FixedTPoint2point.construct(self)

    def define_terminal_constraints(self):
        objective = 0.
        for l, vehicle in enumerate(self.vehicles):
            term_con, term_con_der = vehicle.get_terminal_constraints(
                vehicle.splines[0])
            conditions = self.define_variable(
                'conT'+str(l), len(self.free_ind[vehicle]))
            cnt = 0
            for k, con in enumerate(term_con):
                if k in self.free_ind[vehicle]:
                    spline, condition = con[0], conditions[cnt]
                    cnt += 1
                else:
                    spline, condition = con[0], con[1]
                g = self.define_spline_variable(
                    'g'+str(k), 1, basis=spline.basis)[0]
                objective += definite_integral(g, self.t0, 1.)
                self.define_constraint(spline - condition - g, -inf, 0.)
                self.define_constraint(-spline + condition - g, -inf, 0.)
            for con in term_con_der:
                self.define_constraint(spline(1.) - condition, 0., 0.)
        self.define_objective(objective)
