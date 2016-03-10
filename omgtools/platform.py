from vehicle import Vehicle
from shape import Rectangle
from casadi import inf

import numpy as np


class Platform(Vehicle):

    def __init__(self, width=0.7, height=0.1, options={}, bounds={}, **kwargs):
        Vehicle.__init__(self, n_y=1, n_der=2, degree=3, order=1, shape=Rectangle(width, height),
                         options=options, **kwargs)
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -0.8
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.8
        self.amin = bounds['amin'] if 'amin' in bounds else -2.
        self.amax = bounds['amax'] if 'amax' in bounds else 2.

        # define physical signals
        y = self.get_signal('y')

        y, dy, ddy = y[0, 0], y[0, 1], y[0, 2]

        self.define_pose([y, 0.])
        u = self.define_input(dy)
        x = self.define_state(y)
        self.define_signal('a', ddy)
        self.define_y([[x], [u]])
        self.define_dstate([u])

        # define system constraints
        y = self.splines[0]
        dy = y.derivative()
        ddy = y.derivative(2)
        T = self.define_symbol('T')

        self.define_constraint(-dy + T*self.vmin, -inf, 0.)
        self.define_constraint(dy - T*self.vmax, -inf, 0.)

        self.define_constraint(-ddy + (T**2)*self.amin, -inf, 0.)
        self.define_constraint(ddy - (T**2)*self.amax, -inf, 0.)

    def set_initial_pose(self, position):
        y = np.zeros((self.n_y, self.n_der+1))
        y[:, 0] = position
        self.set_initial_condition(y)

    def set_terminal_pose(self, position):
        y = np.zeros((self.n_y, self.n_der+1))
        y[:, 0] = position
        self.set_terminal_condition(y)

    def get_checkpoints(self, y=None):
        if y is None:
            return self.shape.get_checkpoints([self.splines[0], 0.])
        else:
            return self.shape.get_checkpoints([y[0], 0.])

    def draw(self, t=-1):
        return np.vstack((self.path['pose'][:, :, t])) + self.shape.draw()
