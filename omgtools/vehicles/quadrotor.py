from vehicle import Vehicle
from ..basics.shape import Circle
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np


class Quadrotor(Vehicle):

    def __init__(self, radius=0.2, options={}, bounds={}):
        Vehicle.__init__(
            self, n_spl=2, degree=4, shapes=Circle(radius), options=options)
        self.radius = radius
        self.u1min = bounds['u1min'] if 'u1min' in bounds else 1.
        self.u1max = bounds['u1max'] if 'u1max' in bounds else 15.
        self.u2min = bounds['u2min'] if 'u2min' in bounds else -8.
        self.u2max = bounds['u2max'] if 'u2max' in bounds else 8.
        self.g = 9.81
        # time horizon
        self.T = self.define_symbol('T')

    def define_trajectory_constraints(self, splines):
        x, y = splines
        ddx, ddy = x.derivative(2), y.derivative(2)
        dddx, dddy = x.derivative(3), y.derivative(3)
        g_tf = self.g*(self.T**2)
        self.define_constraint(-(ddx**2 + (ddy+g_tf)**2) +
                               (self.T**4)*self.u1min**2, -inf, 0.)
        self.define_constraint(
            (ddx**2 + (ddy+g_tf)**2) - (self.T**4)*self.u1max**2, -inf, 0.)
        self.define_constraint(-(dddx*(ddy+g_tf) - ddx*dddy) +
                               (ddx**2 + (ddy + g_tf)**2)*(self.T*self.u2min), -inf, 0.)
        self.define_constraint(
            (dddx*(ddy+g_tf) - ddx*dddy) - (ddx**2 + (ddy + g_tf)**2)*(self.T*self.u2max), -inf, 0.)

    def get_initial_constraints(self, splines):
        spl0 = self.define_parameter('spl0', 2)
        dspl0 = self.define_parameter('dspl0', 2)
        ddspl0 = self.define_parameter('ddspl0', 2)
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        return [(x, spl0[0]), (y, spl0[1]),
                (dx, self.T*dspl0[0]), (dy, self.T*dspl0[1]),
                (ddx, (self.T**2)*ddspl0[0]), (ddy, (self.T**2)*ddspl0[1])]

    def get_terminal_constraints(self, splines):
        position = self.define_parameter('positionT', 2)
        x, y = splines
        return [(x, position[0]), (y, position[1])]

    def set_initial_conditions(self, position):
        self.prediction['state'] = np.r_[position, np.zeros(3)].T
        self.prediction['dspl'] = np.zeros(2)
        self.prediction['ddspl'] = np.zeros(2)

    def set_terminal_conditions(self, position):
        self.positionT = position

    def get_init_spline_value(self):
        init_value = np.zeros((len(self.basis), 2))
        pos0 = self.prediction['state'][:2]
        posT = self.positionT
        for k in range(2):
            init_value[:, k] = np.r_[pos0[k]*np.ones(self.degree), np.linspace(
                pos0[k], posT[k], len(self.basis) - 2*self.degree), posT[k]*np.ones(self.degree)]
        return init_value

    def check_terminal_conditions(self):
        if (np.linalg.norm(self.signals['position'][:, -1] - self.positionT) > 1.e-2 or
                np.linalg.norm(self.signals['dspl'][:, -1])) > 1.e-2:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = {}
        parameters['spl0'] = self.prediction['state'][:2]
        parameters['dspl0'] = self.prediction['dspl']
        parameters['ddspl0'] = self.prediction['ddspl']
        parameters['positionT'] = self.positionT
        return parameters

    def define_collision_constraints(self, hyperplanes, room_lim, splines):
        x, y = splines[0], splines[1]
        self.define_collision_constraints_2d(hyperplanes, room_lim, [x, y])

    def splines2signals(self, splines, time):
        signals = {}
        x, y = splines[0], splines[1]
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        dddx, dddy = x.derivative(3), y.derivative(3)

        x_s, y_s = sample_splines([x, y], time)
        dx_s, dy_s = sample_splines([dx, dy], time)
        ddx_s, ddy_s = sample_splines([ddx, ddy], time)
        dddx_s, dddy_s = sample_splines([dddx, dddy], time)

        theta = np.arctan2(ddx_s, ddy_s + self.g)
        u1 = np.sqrt(ddx_s**2 + (ddy_s + self.g)**2)
        u2 = (dddx_s*(ddy_s + self.g) - ddx_s*dddy_s) / \
            ((ddy_s + self.g)**2 + ddx_s**2)
        signals['state'] = np.c_[x_s, y_s, dx_s, dy_s, theta].T
        signals['input'] = np.c_[u1, u2].T
        signals['position'] = np.c_[x_s, y_s].T
        signals['dspl'] = np.c_[dx_s, dy_s].T
        signals['ddspl'] = np.c_[ddx_s, ddy_s].T
        return signals

    def ode(self, state, input):
        theta = state[4]
        u1, u2 = input[0], input[1]
        return np.r_[state[2:4], u1*np.sin(theta), u1*np.cos(theta)-self.g, u2].T

    def draw(self, t=-1):
        theta = -self.signals['state'][4, t]
        cth, sth = np.cos(theta), np.sin(theta)
        rot = np.array([[cth, -sth], [sth, cth]])
        r = self.radius
        h, rw = 0.2*r, (1./3.)*r
        plt_x = [r, r-2*rw, r-rw, r-rw, -r+rw, -r+rw, -r, -r+2*rw]
        plt_y = [h, h, h, 0, 0, h, h, h]
        return [np.c_[self.signals['position'][:, t]] + rot.dot(np.vstack((plt_x, plt_y)))]
