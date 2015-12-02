from vehicle import Vehicle
from shape import Circle
from casadi import inf
from scipy.integrate import odeint

import numpy as np


class Holonomic(Vehicle):

    def __init__(self, shape=Circle(0.1), options={}, bounds={}, **kwargs):
        Vehicle.__init__(self, n_y=2, degree=3, shape=shape,
                         options=options, order=2, **kwargs)
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -0.5
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amin = bounds['amin'] if 'amin' in bounds else -1.
        self.amax = bounds['amax'] if 'amax' in bounds else 1.

        # define physical signals
        y = self.get_signal('y')

        y0, dy0, ddy0 = y[0, 0], y[0, 1], y[0, 2]
        y1, dy1, ddy1 = y[1, 0], y[1, 1], y[1, 2]

        self.define_position([y0, y1])
        self.define_input([dy0, dy1])
        self.define_state([y0, y1])
        self.define_signal('a', [ddy0, ddy1])

        # define system constraints
        y0, y1 = self.splines[0], self.splines[1]
        dy0, dy1 = y0.derivative(), y1.derivative()
        ddy0, ddy1 = y0.derivative(2), y1.derivative(2)
        T = self.define_symbol('T')

        self.define_constraint(-dy0 + T*self.vmin, -inf, 0.)
        self.define_constraint(-dy1 + T*self.vmin, -inf, 0.)
        self.define_constraint(dy0 - T*self.vmax, -inf, 0.)
        self.define_constraint(dy1 - T*self.vmax, -inf, 0.)

        self.define_constraint(-ddy0 + (T**2)*self.amin, -inf, 0.)
        self.define_constraint(-ddy1 + (T**2)*self.amin, -inf, 0.)
        self.define_constraint(ddy0 - (T**2)*self.amax, -inf, 0.)
        self.define_constraint(ddy1 - (T**2)*self.amax, -inf, 0.)

    def set_initial_pose(self, position):
        y = np.zeros((self.n_y, self.order+1))
        y[:, 0] = position
        self.set_initial_condition(y)

    def set_terminal_pose(self, position):
        y = np.zeros((self.n_y, self.order+1))
        y[:, 0] = position
        self.set_terminal_condition(y)

    def integrate_model(self, y0, input, sample_time, integration_time):
        n_samp = int(integration_time/sample_time)+1
        y = np.zeros((self.n_y, self.order+1, n_samp))
        state0 = self.get_state(y0).ravel()
        time_axis = np.linspace(0., (n_samp-1)*sample_time, n_samp)
        y[:, 0, :] = odeint(self._model_update, state0, time_axis,
                            args=(input[:, 0, :], sample_time)).T
        y[:, 1, :] = input[:, 0, :n_samp]
        for d in range(2, self.order+1):
            for k in range(self.n_y):
                y[k, d, :] = np.gradient(y[k, d-1, :], sample_time)
        return y

    def _model_update(self, state, time, input, sample_time):
        u = self._get_input_sample(time, input, sample_time)
        return u

    def draw(self, t=-1):
        return self.path['position'][:, :, t] + self.shape.draw()
