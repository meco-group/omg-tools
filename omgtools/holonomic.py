from vehicle import Vehicle
from shape import Circle
from spline_extra import sample_splines

import numpy as np


class Holonomic(Vehicle):

    def __init__(self, shape=Circle(0.1), options={}, bounds={}):
        Vehicle.__init__(self, n_spl=2, degree=3, shape=shape, options=options)
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -0.5
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amin = bounds['amin'] if 'amin' in bounds else -1.
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        # time horizon
        self.T = self.define_symbol('T')

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'syslimit': 'norm_inf'})
        self.options.update({'terminal_smoothness': self.degree})

    def define_trajectory_constraints(self, splines):
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        if self.options['syslimit'] is 'norm_2':
            self.define_constraint(
                (dx**2+dy**2) - (self.T**2)*self.vmax**2, -np.inf, 0.)
            self.define_constraint(
                (ddx**2+ddy**2) - (self.T**4)*self.amax**2, -np.inf, 0.)
        elif self.options['syslimit'] is 'norm_inf':
            self.define_constraint(-dx + self.T*self.vmin, -np.inf, 0.)
            self.define_constraint(-dy + self.T*self.vmin, -np.inf, 0.)
            self.define_constraint(dx - self.T*self.vmax, -np.inf, 0.)
            self.define_constraint(dy - self.T*self.vmax, -np.inf, 0.)

            self.define_constraint(-ddx + (self.T**2)*self.amin, -np.inf, 0.)
            self.define_constraint(-ddy + (self.T**2)*self.amin, -np.inf, 0.)
            self.define_constraint(ddx - (self.T**2)*self.amax, -np.inf, 0.)
            self.define_constraint(ddy - (self.T**2)*self.amax, -np.inf, 0.)
        else:
            raise ValueError(
                'Only norm_2 and norm_inf are defined as system limit.')

    def get_initial_constraints(self, splines):
        state0 = self.define_parameter('state0', 2)
        input0 = self.define_parameter('input0', 2)
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        return [(x, state0[0]), (y, state0[1]),
                (dx, self.T*input0[0]), (dy, self.T*input0[1])]

    def get_terminal_constraints(self, splines):
        poseT = self.define_parameter('poseT', 2)
        x, y = splines
        term_con = [(x, poseT[0]), (y, poseT[1])]
        return term_con

    def set_initial_conditions(self, state, input=np.zeros(2)):
        self.prediction['state'] = state
        self.prediction['input'] = input

    def set_terminal_conditions(self, pose):
        self.poseT = pose

    def check_terminal_conditions(self):
        if (np.linalg.norm(self.signals['state'][:, -1] - self.poseT) > 1.e-3 or
                np.linalg.norm(self.signals['input'][:, -1])) > 1.e-3:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = {}
        parameters['state0'] = self.prediction['state']
        parameters['input0'] = self.prediction['input']
        parameters['poseT'] = self.poseT
        return parameters

    def define_collision_constraints(self, hyperplane):
        a, b = hyperplane['a'], hyperplane['b']
        x, y = self.splines
        # run over all shapes... -> in vehicle

    def splines2signals(self, splines, time):
        signals = {}
        x, y = splines[0], splines[1]
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        input = np.c_[sample_splines([dx, dy], time)]
        signals['state'] = np.c_[sample_splines([x, y], time)]
        signals['input'] = input
        signals['position'] = signals['state']
        signals['v_tot'] = np.sqrt(input[0, :]**2 + input[1, :]**2)
        signals['a'] = np.c_[sample_splines([ddx, ddy], time)]
        return signals

    def ode(self, state, input):
        return input

    def draw(self, t=-1):
        return np.c_[self.signals['state'][:, t]] + self.shape.draw()
