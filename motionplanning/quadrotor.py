from vehicle import Vehicle
from shape import Quad
import numpy as np
from casadi import inf, sqrt, arctan2
from scipy.integrate import odeint


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
        self.define_state([y0, y1, theta, dy0, dy1])

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

    def integrate_model(self, y0, input, sample_time, integration_time):
        n_samp = int(integration_time/sample_time)+1
        y = np.zeros((self.n_y, self.order+1, n_samp))
        state0 = self.get_state(y0).ravel()
        time_axis = np.linspace(0., (n_samp-1)*sample_time, n_samp)
        state = odeint(self._model_update, state0, time_axis,
                       args=(input[:, 0, :], sample_time)).T
        y[:, 0, :] = state[:2, :]
        y[:, 1, :] = state[3:, :]
        u = input[:, 0, :n_samp]
        theta = state[2, :]
        y[0, 2, :] = u[0, :]*np.sin(theta)
        y[1, 2, :] = u[0, :]*np.cos(theta) - self.g
        for d in range(3, self.order+1):
            for k in range(self.n_y):
                y[k, d, :] = np.gradient(y[k, d-1, :], sample_time)
        return y

    def _model_update(self, state, time, input, sample_time):
        u = self._get_input_sample(time, input, sample_time)
        x, z, theta, dx, dz = state
        dstate = np.zeros(5)
        dstate[0] = dx
        dstate[1] = dz
        dstate[2] = u[1]
        dstate[3] = u[0]*np.sin(theta)
        dstate[4] = u[0]*np.cos(theta) - self.g
        return dstate

    def draw(self, t=-1):
        return (self.path['position'][:, :, t] +
                self.shape.draw(-self.path['state'][2, :, t]))
