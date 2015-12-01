from vehicle import Vehicle
from shape import Circle
from casadi import inf
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
        self.define_input([dy0, dy0])
        self.define_state([y0, y1, dy0, dy1])

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

    def update_model(self, state, input, _y, sample_time):
        _x, _z, _ux, _uz = state[0, :], state[1, :], state[2, :], state[3, :]

        ux, uz = input[0, :], input[1, :]
        x = _x + 0.5*sample_time*(ux + _ux)
        z = _z + 0.5*sample_time*(uz + _uz)

        y = np.zeros((self.n_y, self.order+1))
        y[:, 0] = [x, z]
        y[:, 1] = [ux, uz]
        for k in range(2, self.orde+2):
            y[:, k] = _y[:, k]
        return y

    def draw(self, t=-1):
        return self.path['position'][:, :, t] + self.shape.draw()
