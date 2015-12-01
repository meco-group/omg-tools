from vehicle import Vehicle
from shape import Quad
import numpy as np
from casadi import inf, sqrt, arctan2


class Quadrotor(Vehicle):

    def __init__(self, radius=0.2, options={}, bounds={},  **kwargs):
        Vehicle.__init__(self, n_y=2, degree=4, shape=Quad(
            radius), options=options, order=3, **kwargs)
        self.u1min = bounds['u1min'] if 'u1min' in bounds else 1.
        self.u1max = bounds['u1max'] if 'u1max' in bounds else 15.
        self.u2min = bounds['u2min'] if 'u2min' in bounds else -8.
        self.u2max = bounds['u2max'] if 'u2max' in bounds else 8.
        self.g = 9.81

        # define physical signals
        y = self.get_signal('y')

        y0, dy0, ddy0, dddy0 = y[0, 0], y[0, 1], y[0, 2], y[0, 3]
        y1, dy1, ddy1, dddy1 = y[1, 0], y[1, 1], y[1, 2], y[1, 3]

        u1 = sqrt(ddy0**2 + (ddy1+self.g)**2)
        u2 = (dddy0*(ddy1 + self.g) - ddy0*dddy1) / \
            ((ddy1 + self.g)**2 + ddy0**2)
        theta = arctan2(ddy0, (ddy1 + self.g))

        self.define_position([y0, y1])
        self.define_input([u1, u2])
        self.define_state([y0, y1, dy0, dy1, theta, u1, u2])

        # define system constraints
        y0, y1 = self.splines[0], self.splines[1]
        dy0, dy1 = y0.derivative(), y1.derivative()
        ddy0, ddy1 = y0.derivative(2), y1.derivative(2)
        dddy0, dddy1 = y0.derivative(3), y1.derivative(3)
        # this is a symbol which needs to be defined by another OptiLayer
        # object as variable or parameter
        T = self.define_symbol('T')
        g_tf = self.g*(T**2)

        self.define_constraint(-(ddy0**2 + (ddy1+g_tf) ** 2) +
                               (T**4)*self.u1min**2, -inf, 0.)
        self.define_constraint((ddy0**2 + (ddy1+g_tf)**2) -
                               (T**4)*self.u1max**2, -inf, 0.)
        self.define_constraint(-(dddy0*(ddy1+g_tf) - ddy0*dddy1) +
                               (ddy0**2 + (ddy1 + g_tf)**2)*(T*self.u2min),
                               -inf, 0.)
        self.define_constraint((dddy0*(ddy1+g_tf) - ddy0*dddy1) -
                               (ddy0**2 + (ddy1 + g_tf)**2)*(T*self.u2max),
                               -inf, 0.)

    def set_initial_pose(self, position):
        y = np.zeros((self.n_y, self.order+1))
        y[:, 0] = position
        self.set_initial_condition(y)

    def set_terminal_pose(self, position):
        y = np.zeros((self.n_y, self.order+1))
        y[:, 0] = position
        self.set_terminal_condition(y)

    def update_model(self, state, input, _y, sample_time):
        _x, _z, _dx, _dz = state[0, :], state[1, :], state[2, :], state[3, :]
        _th, _u1, _u2 = state[4, :], state[5, :], state[6, :]

        u1, u2 = input[0, :], input[1, :]
        th = _th + 0.5*sample_time*(u2 + _u2)
        dx = _dx + 0.5*sample_time*(u1*np.sin(th) + _u1*np.sin(_th))
        dz = _dz + 0.5*sample_time*(u1*np.cos(th) + _u1*np.cos(_th) - 2*self.g)
        x = _x + 0.5*sample_time*(dx + _dx)
        z = _z + 0.5*sample_time*(dz + _dz)

        y = np.zeros((self.n_y, self.order+1))
        y[:, 0] = [x, z]
        y[:, 1] = [dx, dz]
        y[:, 2] = [u1*np.sin(th), u1*np.cos(th)-self.g]
        y[:, 3] = _y[:, 3]
        return y

    def draw(self, t=-1):
        return (self.path['position'][:, :, t] +
                self.shape.draw(-self.path['state'][4, :, t]))
