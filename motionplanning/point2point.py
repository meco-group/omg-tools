from problem import FixedTProblem
from spline_extra import definite_integral, evalspline
import numpy as np


class Point2point(FixedTProblem):

    def __init__(self, fleet, environment, options={}):
        FixedTProblem.__init__(self, fleet, environment, options, label='p2p')

        g = [self.define_spline_variable(
             'g_'+str(l), vehicle.n_y, basis=vehicle.basis)
             for l, vehicle in enumerate(self.vehicles)]
        T = self.define_symbol('T')
        t = self.define_symbol('t')
        t0 = t/T

        y0 = [self.define_parameter(
            'y0_'+str(l), vehicle.n_y, vehicle.order+1)
            for l, vehicle in enumerate(self.vehicles)]
        yT = [self.define_parameter(
            'yT_'+str(l), vehicle.n_y, vehicle.order+1)
            for l, vehicle in enumerate(self.vehicles)]

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

        self.knot_time = (int(self.vehicles[0].options['horizon_time']*1000.) /
                          self.vehicles[0].knot_intervals) / 1000.

    def set_parameters(self, current_time):
        parameters = FixedTProblem.set_parameters(self, current_time)
        for l, vehicle in enumerate(self.vehicles):
            parameters['y0_'+str(l)] = vehicle.prediction['y']
            parameters['yT_'+str(l)] = vehicle.yT
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

    def stop_criterium(self):
        for vehicle in self.vehicles:
            if (np.linalg.norm(vehicle.trajectory['y'][:, :, 0] - vehicle.yT)
               > 1.e-2):
                return False
        return True
