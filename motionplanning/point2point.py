from problem import Problem
from spline import BSplineBasis
from spline_extra import definite_integral, evalspline
from spline_extra import shiftoverknot_T, shift_spline
from casadi import vertcat, fmod, MX, MXFunction
import export
import numpy as np


class Point2point(object):
    # This class looks at the problem type of the created problem and
    # creates the corresponding instance (freeTProblem or fixedTProblem)
    def __new__(cls, fleet, environment, options={}):
        # by default fixed T problem
        if 'freeTProblem' not in options:
            options['freeTProblem'] = False
        if (options['freeTProblem'] is True):
            return FreeTPoint2point(fleet, environment, options)
        else:
            return FixedTPoint2point(fleet, environment, options)


class Point2pointProblem(Problem):

    def __init__(self, fleet, environment, options={}, label='p2p'):
        Problem.__init__(self, fleet, environment, options, label=label)

        T = self.define_symbol('T')
        t = self.define_symbol('t')
        self.t0 = t/T

        # initial state
        y0 = [self.define_parameter('y0_'+str(l), vehicle.n_y)
              for l, vehicle in enumerate(self.vehicles)]
        dy0 = [self.define_parameter(
            'dy0_'+str(l), vehicle.n_der, vehicle.n_y)
               for l, vehicle in enumerate(self.vehicles)]

        # terminal state
        fxd_yT = self.options['fixed_yT']
        # part of goal positions which are parameters
        yT_par = [self.define_parameter(
            'yTp_'+str(l), len(fxd_yT[l]))
                  for l, vehicle in enumerate(self.vehicles)]
        # part of goal positions which are variables
        yT_var = [self.define_variable(
            'yTv_'+str(l), vehicle.n_y-len(fxd_yT[l]))
                  for l, vehicle in enumerate(self.vehicles) if (vehicle.n_y-len(fxd_yT[l]) > 0)]

        # final vel, acc,... depending on vehicle.n_der
        dyT = [self.define_parameter(
            'dyT_'+str(l), vehicle.n_der, vehicle.n_y)
               for l, vehicle in enumerate(self.vehicles)]

        self.yT = []
        for l, veh in enumerate(self.vehicles):
            yT_, cnt_par, cnt_var = [], 0, 0
            for k in range(veh.n_y):
                if k in fxd_yT[l]:
                    yT_.append(yT_par[l][cnt_par])
                    cnt_par += 1
                else:
                    yT_.append(yT_var[l][cnt_var])
                    cnt_var += 1
            # collect parametric and variable parts of goal positions in yT
            self.yT.append(vertcat(yT_))

        # vehicle splines
        self.y = [vehicle.splines for vehicle in self.vehicles]

        # initial & terminal constraints
        for l, vehicle in enumerate(self.vehicles):
            bs = vehicle.options['boundary_smoothness']
            for k in range(vehicle.n_y):
                # d = deg of derivative used to impose smoothness on first
                # knots and internal knots
                for d in range(max(bs['initial'], bs['internal'])+1):
                    if (d > bs['initial']) and (d <= bs['internal']):
                        shutdown={'equal': 0.}
                    elif (d > bs['internal']) and (d <= bs['initial']):
                        shutdown={'greater': 0.}
                    else:
                        shutdown = False
                    if d == 0:
                        self.define_constraint(
                            evalspline(self.y[l][k], self.t0) - y0[l][k],
                            0., 0., shutdown)
                    else:
                        # dy0 = [dx0 dy0 ; ddx0 ddy0] for n_y=2 and n_der=2
                        if d <= vehicle.n_der:
                            self.define_constraint(
                                (evalspline(self.y[l][k].derivative(d), self.t0) -
                                 (T**d)*dy0[l][d-1, k]), 0., 0.,
                                shutdown, str(d)+str(k))
                        else:
                            self.define_constraint(
                                evalspline(self.y[l][k].derivative(d), self.t0),
                                0., 0., shutdown, str(d)+str(k))
                # d = deg of derivative you used on the last knots to impose
                # continuity/smoothness
                for d in range(bs['terminal']+1):
                    if not (d == 0):
                        if d <= vehicle.n_der:
                            self.define_constraint(
                                self.y[l][k].derivative(d)(
                                    1.) - (T**d)*dyT[l][d-1, k],
                                0., 0.)
                        else:
                            self.define_constraint(
                                (self.y[l][k].derivative(d))(1.), 0., 0.)

    def set_default_options(self):
        Problem.set_default_options(self)
        # by default assign all end positions as parameters
        self.options['fixed_yT'] = [range(veh.n_y) for veh in self.vehicles]

    def set_parameters(self, current_time):
        parameters = {}
        for l, vehicle in enumerate(self.vehicles):
            parameters['y0_'+str(l)] = vehicle.prediction['y'][:, 0]
            yT = []
            if len(self.options['fixed_yT'][l]) > 0:
                for ind in self.options['fixed_yT'][l]:
                    yT.append(vehicle.yT[ind, 0])
                parameters['yTp_'+str(l)] = np.array(yT)
            parameters['dy0_'+str(l)] = vehicle.prediction['y'][:, 1:].T
            parameters['dyT_'+str(l)] = vehicle.yT[:, 1:].T
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

    def stop_criterium(self):
        for vehicle in self.vehicles:
            if (np.linalg.norm(vehicle.trajectory['y'][:, :, 0] - vehicle.yT)
                    > 1.e-2):
                return False
        return True

    # ========================================================================
    # Methods encouraged to override (very basic implementation)
    # ========================================================================

    def compute_objective(self):
        return {}


class FixedTPoint2point(Point2pointProblem):

    def __init__(self, fleet, environment, options={}, label='fixedT'):
        Point2pointProblem.__init__(self, fleet, environment, options, label)
        self.knot_time = (int(self.options['horizon_time']*1000.) /
                          self.vehicles[0].knot_intervals) / 1000.
        self.T = self.define_parameter('T')
        self.t = self.define_parameter('t')
        self.t0 = self.t/self.T

        # define slack 'g' to impose 1-norm objective function
        g = [self.define_spline_variable(
            'g_'+str(l), vehicle.n_y, basis=vehicle.basis)
            for l, vehicle in enumerate(self.vehicles)]

        # objective + related constraints
        objective = 0.
        for l, vehicle in enumerate(self.vehicles):
            objective += sum([definite_integral(g_k, self.t0, 1.) for g_k in g[l]])
            for k in range(vehicle.n_y):
                self.define_constraint(
                    self.y[l][k] - self.yT[l][k] - g[l][k], -np.inf, 0.)
                self.define_constraint(-self.y[l][k] +
                                       self.yT[l][k] - g[l][k], -np.inf, 0.)
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

    def update_vehicles(self, current_time, update_time):
        for vehicle in self.vehicles:
            # y_coeffs represents coefficients of a spline, for which a part of
            # its time horizon lies in the past. Therefore, we need to pass the
            # current time relatively to the begin of this time horizon. In this
            # way, only the future, relevant, part will be saved/plotted. Also an
            # adapted time axis of the knots is passed: the first knot is shifted
            # towards the current itme point.
            y_coeffs = vehicle.get_variable('y', solution=True, spline=False)
            rel_current_time = np.round(current_time, 6) % self.knot_time
            horizon_time = self.options['horizon_time']
            time_axis_knots = np.copy(vehicle.knots)*horizon_time
            time_axis_knots[:vehicle.degree+1] = rel_current_time
            time_axis_knots += current_time - rel_current_time
            vehicle.update(y_coeffs, current_time, update_time, horizon_time,
                           rel_current_time=rel_current_time,
                           time_axis_knots=time_axis_knots)

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


class FreeTPoint2point(Point2pointProblem):

    def __init__(self, fleet, environment, options={}, label='freeT'):
        Point2pointProblem.__init__(self, fleet, environment, options, label)

        # unknown motion time
        self.T = self.define_variable('T')
        # current time
        self.define_parameter('t')

        # objective
        objective = self.T
        # Todo: include soft constraints here
        self.define_objective(objective)

        # add terminal constraint on position
        # l = vehicle number
        for l, vehicle in enumerate(self.vehicles):
            # k = select x(=y0), y(=y1),...
            for k in range(vehicle.n_y):
                self.define_constraint(
                    evalspline(self.y[l][k], 1) - self.yT[l][k],
                    0., 0.)

    def set_parameters(self, current_time):
        parameters = Point2pointProblem.set_parameters(self, current_time)
        # current time is always 0 for FreeT problem, time axis always resets
        parameters['t'] = 0
        return parameters

    def init_variables(self):
        self._variables['T'] = self.options['horizon_time']
        return self._variables

    def init_step(self, current_time):
        # transform spline variables to get better initialization
        # use optimal motion time from previous iteration
        Tprev = float(self.father._var_result['p2p0', 'T'])
        # check if almost arrived, if so lower the update time
        if Tprev - self.options['update_time'] < self.options['update_time']:
            update_time = Tprev - self.options['update_time']
            target_time = Tprev
        else:
            update_time = self.options['update_time']
            target_time = Tprev - update_time
        # create spline which starts from the position at update_time and goes
        # to goal position at target_time. Approximate/Represent this spline in
        # a new basis with new equidistant knots.
        self.father.transform_primal_splines(lambda coeffs, basis:
            shift_spline(coeffs, update_time/target_time, basis))

    def compute_objective(self):
        # gives the current value of the objective
        # motion time is known at this point
        objective = self.T
        return objective

    def update_vehicles(self, current_time, update_time):
        for vehicle in self.vehicles:
            # assign numerical value of T
            self.T = float(self.father._var_result['p2p0', 'T'])
            horizon_time = self.T
            # current solution
            y_coeffs = vehicle.get_variable('y', solution=True, spline=False)
            vehicle.update(y_coeffs, current_time, update_time, horizon_time)

    def stop_criterium(self):
        for vehicle in self.vehicles:
            if (self.T < self.options['update_time']):
                return True
            if (np.linalg.norm(vehicle.trajectory['y'][:, :, 0] - vehicle.yT)
                    > 1.e-2):
                return False
        return True
