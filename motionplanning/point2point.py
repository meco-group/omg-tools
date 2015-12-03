from optilayer import OptiLayer
from problem import Problem
from spline_extra import definite_integral, evalspline, shift_over_knot
import numpy as np


class Point2point(Problem):

    def __init__(self, fleet, environment, options={}):
        Problem.__init__(self, fleet, environment, options)

        g = [self.define_spline_variable(
            'g_'+str(vehicle.index), vehicle.n_y, basis=vehicle.basis)
            for vehicle in self.vehicles]
        T = self.define_parameter('T')
        t = self.define_parameter('t')
        t0 = t/T

        y0 = [self.define_parameter(
            'y0_'+str(vehicle.index), vehicle.n_y, vehicle.order+1)
            for vehicle in self.vehicles]
        yT = [self.define_parameter(
            'yT_'+str(vehicle.index), vehicle.n_y, vehicle.order+1)
            for vehicle in self.vehicles]

        y = [vehicle.splines for vehicle in self.vehicles]

        # objective + related constraints
        objective = 0.
        for l, vehicle in enumerate(self.vehicles):
            objective += sum([definite_integral(g_k, t0, 1.) for g_k in g[l]])
            for k in range(vehicle.n_y):
                self.define_constraint(
                    y[l][k] - yT[l][k] - g[l][k], -np.inf, 0.)
                self.define_constraint(-y[l][k] +
                                       yT[l][k] - g[l][k], -np.inf, 0.)
        self.define_objective(objective)

        # initial & terminal constraints
        for l, vehicle in enumerate(self.vehicles):
            bs = vehicle.options['boundary_smoothness']
            for k in range(vehicle.n_y):
                for d in range(max(bs['initial'], bs['internal'])+1):
                    if (d > bs['initial']) and (d <= bs['internal']):
                        shutdown = lambda t: (t == 0.)
                    elif (d > bs['internal']) and (d <= bs['initial']):
                        shutdown = lambda t: (t > 0.)
                    else:
                        shutdown = False
                    if d == 0:
                        self.define_constraint(
                            evalspline(y[l][k], t0) - y0[l][k, d],
                            0., 0., shutdown)
                    else:
                        self.define_constraint(
                            (evalspline(y[l][k].derivative(d), t/T) -
                             (T**d)*y0[l][k, d]), 0., 0.,
                            shutdown, str(d)+str(k))
                for d in range(bs['terminal']+1):
                    if not (d == 0):
                        if d <= vehicle.order:
                            self.define_constraint(
                                y[l][k].derivative(d)(1.) - (T**d)*yT[l][k, d],
                                0., 0.)
                        else:
                            self.define_constraint(
                                (y[l][k].derivative(d))(1.), 0., 0.)
        self.construct_problem()

        self.knot_time = (int(self.vehicles[0].options['horizon_time']*1000.) /
                          self.vehicles[0].knot_intervals) / 1000.

    def get_parameters(self, current_time):
        parameters = {}
        parameters['T'] = self.vehicles[0].options['horizon_time']
        parameters['t'] = np.round(current_time, 6) % self.knot_time
        for vehicle in self.vehicles:
            parameters['y0_'+str(vehicle.index)] = vehicle.prediction['y']
            parameters['yT_'+str(vehicle.index)] = vehicle.yT
        return parameters

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
        objective = 0.
        for vehicle in self.vehicles:
            n_samp = vehicle.path['y'].shape[2]
            err = vehicle.path['y'][:, 0, :] - np.vstack(vehicle.yT[:, 0])
            err_nrm = np.zeros(n_samp)
            for k in range(n_samp):
                err_nrm[k] = np.linalg.norm(err[:, k], 1)
            for k in range(n_samp-1):
                objective += 0.5 * \
                    (err_nrm[k] + err_nrm[k+1])*vehicle.options['sample_time']
        return objective

    def init_step(self):
        # transform spline variables
        if np.round(self.current_time, 6) % self.knot_time == 0:
            OptiLayer.transform_splines(
                lambda coeffs, knots, degree: shift_over_knot(coeffs, knots,
                                                              degree, 1))

    def update(self, current_time, update_time):
        for vehicle in self.vehicles:
            y_coeffs = vehicle.get_variable('y')
            """
            This represents coefficients of spline in a basis, for which a part
            of the corresponding time horizon lies in the past. Therefore,
            we need to pass the current time relatively to the begin of this
            time horizon. In this way, only the future, relevant, part will be
            saved and plotted. Also an adapted time axis of the knots is
            passed: Here the first knot is shifted towards the current time
            point. In the future this approach should dissapear: when symbolic
            knots are possible in the spline toolbox, we can transform the
            spline every iteration (in the init_step method). In this way,
            the current time coincides with the begin of the considered time
            horizon (rel_current_time = 0.).
            """
            rel_current_time = np.round(current_time, 6) % self.knot_time
            time_axis_knots = np.copy(
                vehicle.knots)*vehicle.options['horizon_time']
            time_axis_knots[:vehicle.degree+1] = rel_current_time
            time_axis_knots += current_time - rel_current_time
            vehicle.update(y_coeffs, current_time, update_time,
                           rel_current_time=rel_current_time,
                           time_axis_knots=time_axis_knots)
        self.environment.update(current_time, update_time)

    def stop_criterium(self):
        for vehicle in self.vehicles:
            if (np.linalg.norm(vehicle.trajectory['y'][:, :, 0] - vehicle.yT)
               > 1.e-2):
                return False
        return True
