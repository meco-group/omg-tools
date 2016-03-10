from vehicle import Vehicle
from shape import Circle
from casadi import inf, sqrt

import numpy as np


class Holonomic(Vehicle):

    def __init__(self, shape=Circle(0.1), options={}, bounds={}, **kwargs):
        Vehicle.__init__(self, n_y=2, n_der=2, degree=3, order=1, shape=shape,
                         options=options, **kwargs)
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -0.5
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amin = bounds['amin'] if 'amin' in bounds else -1.
        self.amax = bounds['amax'] if 'amax' in bounds else 1.

        # define physical signals
        # y was defined in the Vehicle.__init__()
        y = self.get_signal('y')

        y0, dy0, ddy0 = y[0, 0], y[0, 1], y[0, 2]
        y1, dy1, ddy1 = y[1, 0], y[1, 1], y[1, 2]

        self.define_pose([y0, y1])
        u0, u1 = self.define_input([dy0, dy1])
        x0, x1 = self.define_state([y0, y1])
        self.define_signal('a', [ddy0, ddy1])
        self.define_signal('vnorm_two', [sqrt(dy0**2+dy1**2)])
        self.define_y([[x0, x1], [u0, u1]])
        self.define_dstate([u0, u1])

        # define system constraints
        y0, y1 = self.splines[0], self.splines[1]
        dy0, dy1 = y0.derivative(), y1.derivative()
        ddy0, ddy1 = y0.derivative(2), y1.derivative(2)
        T = self.define_symbol('T')

        if self.options['syslimit'] is 'norm_two':
            self.define_constraint((dy0**2+dy1**2) - T**2*self.vmax**2, -inf, 0.)
            self.define_constraint((ddy0**2+ddy1**2) - (T**2)**2*self.amax**2, -inf, 0.)

        elif self.options['syslimit'] is 'norm_inf':
            self.define_constraint(-dy0 + T*self.vmin, -inf, 0.)
            self.define_constraint(-dy1 + T*self.vmin, -inf, 0.)
            self.define_constraint(dy0 - T*self.vmax, -inf, 0.)
            self.define_constraint(dy1 - T*self.vmax, -inf, 0.)

            self.define_constraint(-ddy0 + (T**2)*self.amin, -inf, 0.)
            self.define_constraint(-ddy1 + (T**2)*self.amin, -inf, 0.)
            self.define_constraint(ddy0 - (T**2)*self.amax, -inf, 0.)
            self.define_constraint(ddy1 - (T**2)*self.amax, -inf, 0.)
        else:
            raise ValueError('System limit type not defined! Use set_options({syslimit: norm_inf}).')

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options['syslimit'] = 'norm_inf'

    def set_initial_pose(self, position):
        y = np.zeros((self.n_y, self.n_der+1))
        y[:, 0] = position
        self.set_initial_condition(y)

    def set_terminal_pose(self, position):
        y = np.zeros((self.n_y, self.n_der+1))
        y[:, 0] = position
        self.set_terminal_condition(y)

    def draw(self, t=-1):
        return self.path['pose'][:, :, t] + self.shape.draw()
