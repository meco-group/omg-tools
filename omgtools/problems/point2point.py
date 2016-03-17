from problem import Problem
from ..basics.spline_extra import definite_integral
from ..basics.spline_extra import shiftoverknot_T, shift_spline, evalspline
from casadi import inf
import numpy as np


class Point2point(object):
    # this class selects between fixed T and free T problem
    def __new__(cls, fleet, environment, options={}, freeT=False):
        if freeT:
            return FreeTPoint2point(fleet, environment, options)
        else:
            return FixedTPoint2point(fleet, environment, options)


class Point2pointProblem(Problem):

    def __init__(self, fleet, environment, options={}):
        Problem.__init__(self, fleet, environment, options, label='p2p')
        for vehicle in self.vehicles:
            splines = vehicle.define_splines(n_seg=1)[0]
            vehicle.define_trajectory_constraints(splines)
            environment.define_collision_constraints(vehicle, splines)

    def stop_criterium(self):
        check = True
        for vehicle in self.vehicles:
            check *= vehicle.check_terminal_conditions()
        return check

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
        raise NotImplementedError('Please implement this method!')


class FixedTPoint2point(Point2pointProblem):

    def __init__(self, fleet, environment, options={}):
        Point2pointProblem.__init__(self, fleet, environment, options)
        self.objective = 0.
        self.knot_time = (int(self.options['horizon_time']*1000.) /
                          self.vehicles[0].options['knot_intervals']) / 1000.
        T, t = self.define_parameter('T'), self.define_parameter('t')
        self.t0 = t/T
        self.define_init_constraints()
        self.define_terminal_constraints()

    def define_init_constraints(self):
        for vehicle in self.vehicles:
            init_con = vehicle.get_initial_constraints(vehicle.splines[0])
            for con in init_con:
                spline, condition = con[0], con[1]
                self.define_constraint(
                    evalspline(spline, self.t0) - condition, 0., 0.)

    def define_terminal_constraints(self):
        objective = 0.
        for vehicle in self.vehicles:
            term_con = vehicle.get_terminal_constraints(vehicle.splines[0])
            for k, con in enumerate(term_con):
                spline, condition = con[0], con[1]
                g = self.define_spline_variable(
                    'g'+str(k), 1, basis=spline.basis)[0]
                objective += definite_integral(g, self.t0, 1.)
                self.define_constraint(spline - condition - g, -inf, 0.)
                self.define_constraint(-spline + condition - g, -inf, 0.)
                for d in range(1, spline.basis.degree+1):
                    self.define_constraint(spline.derivative(d)(1.), 0., 0.)
        self.define_objective(objective)

    def set_default_options(self):
        Point2pointProblem.set_default_options(self)
        self.options['horizon_time'] = 10.

    def set_parameters(self, current_time):
        parameters = Point2pointProblem.set_parameters(self, current_time)
        parameters['t'] = np.round(current_time, 6) % self.knot_time
        parameters['T'] = self.options['horizon_time']
        return parameters

    def init_step(self, current_time):
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

    def update(self, current_time):
        self.compute_partial_objective(current_time)
        update_time = self.options['update_time']
        # update vehicles
        for vehicle in self.vehicles:
            # y_coeffs represents coefficients of a spline, for which a part of
            # its time horizon lies in the past. Therefore, we need to pass the
            # current time relatively to the begin of this time horizon. In this
            # way, only the future, relevant, part will be saved/plotted.
            rel_current_time = np.round(current_time, 6) % self.knot_time
            sample_time = vehicle.options['sample_time']
            horizon_time = self.options['horizon_time']
            n_samp = int(
                round((horizon_time-rel_current_time)/sample_time, 3)) + 1
            time_axis = np.linspace(rel_current_time, horizon_time, n_samp)
            vehicle.update(current_time, update_time, horizon_time, time_axis)
        # update environment
        self.environment.update(update_time)
        return current_time + update_time

    def compute_partial_objective(self, current_time):
        rel_current_time = np.round(current_time, 6) % self.knot_time
        update_time = self.options['update_time']
        horizon_time = self.options['horizon_time']
        t0 = rel_current_time/horizon_time
        t1 = t0 + update_time/horizon_time
        part_objective = 0.
        for vehicle in self.vehicles:
            term_con = vehicle.get_terminal_constraints(vehicle.splines[0])
            for k in range(len(term_con)):
                g = self.get_variable('g'+str(k), solution=True)[0]
                part_objective += horizon_time*definite_integral(g, t0, t1)
        self.objective += part_objective

    def compute_objective(self):
        return self.objective


class FreeTPoint2point(Point2pointProblem):

    def __init__(self, fleet, environment, options={}):
        Point2pointProblem.__init__(self, fleet, environment, options)
        self.objective = 0.
        T = self.define_variable('T', value=10)
        self.define_parameter('t')
        self.define_objective(T)
        # positivity contraint on motion time
        self.define_constraint(-T, -inf, 0.)
        self.define_init_constraints()
        self.define_terminal_constraints()

    def define_init_constraints(self):
        for vehicle in self.vehicles:
            init_con = vehicle.get_initial_constraints(vehicle.splines[0])
            for con in init_con:
                spline, condition = con[0], con[1]
                self.define_constraint(spline(0) - condition, 0., 0.)

    def define_terminal_constraints(self):
        for vehicle in self.vehicles:
            term_con = vehicle.get_terminal_constraints(vehicle.splines[0])
            for con in term_con:
                spline, condition = con[0], con[1]
                self.define_constraint(spline(1.) - condition, 0., 0.)
                for d in range(1, spline.basis.degree+1):
                    self.define_constraint(spline.derivative(d)(1.), 0., 0.)

    def set_parameters(self, current_time):
        parameters = Point2pointProblem.set_parameters(self, current_time)
        # current time is always 0 for FreeT problem, time axis always resets
        parameters['t'] = 0
        return parameters

    def update(self, current_time):
        self.compute_partial_objective(current_time)
        update_time = self.options['update_time']
        # update vehicles
        for vehicle in self.vehicles:
            horizon_time = self.get_variable('T', solution=True)[0][0]
            if horizon_time < update_time:
                update_time = horizon_time
            vehicle.update(current_time, update_time, horizon_time)
        # update environment
        self.environment.update(update_time)
        return current_time + update_time

    def stop_criterium(self):
        T = self.get_variable('T', solution=True)[0][0]
        if T < self.options['update_time']:
            return True
        return Point2pointProblem.stop_criterium(self)

    def init_step(self, current_time):
        T = self.get_variable('T', solution=True)[0][0]
        # check if almost arrived, if so lower the update time
        if T < 2*self.options['update_time']:
            update_time = T - self.options['update_time']
            target_time = T
        else:
            update_time = self.options['update_time']
            target_time = T - update_time
        # create spline which starts from the position at update_time and goes
        # to goal position at target_time. Approximate/Represent this spline in
        # a new basis with new equidistant knots.
        self.father.transform_primal_splines(
            lambda coeffs, basis: shift_spline(coeffs, update_time/target_time, basis))
        self.set_variable('T', target_time)

    def compute_partial_objective(self, current_time):
        self.objective = current_time

    def compute_objective(self):
        return self.objective


class FreeEndPoint2point(FixedTPoint2point):

    def __init__(self, fleet, environment, options, free_ind=None):
        if free_ind is None:
            self.free_ind = {}
            for vehicle in fleet:
                term_con = vehicle.get_terminal_constraints(vehicle.splines[0])
                self.free_ind[vehicle] = range(len(term_con))
        else:
            self.free_ind = free_ind
        FixedTPoint2point.__init__(self, fleet, environment, options)

    def define_terminal_constraints(self):
        objective = 0.
        for l, vehicle in enumerate(self.vehicles):
            term_con = vehicle.get_terminal_constraints(vehicle.splines[0])
            conditions = self.define_variable('conT'+str(l), len(self.free_ind[vehicle]))
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
                self.define_constraint( spline - condition - g, -inf, 0.)
                self.define_constraint(-spline + condition - g, -inf, 0.)
                for d in range(1, spline.basis.degree+1):
                    self.define_constraint(spline.derivative(d)(1.), 0., 0.)
        self.define_objective(objective)
