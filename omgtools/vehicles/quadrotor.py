# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from vehicle import Vehicle
from ..basics.shape import Circle
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np


class Quadrotor(Vehicle):

    def __init__(self, radius=0.2, options=None, bounds=None):
        bounds = bounds or {}
        Vehicle.__init__(
            self, n_spl=2, degree=4, shapes=Circle(radius), options=options)
        self.radius = radius
        self.u1min = bounds['u1min'] if 'u1min' in bounds else 1.
        self.u1max = bounds['u1max'] if 'u1max' in bounds else 15.
        self.u2min = bounds['u2min'] if 'u2min' in bounds else -8.
        self.u2max = bounds['u2max'] if 'u2max' in bounds else 8.
        self.g = 9.81

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options['stop_tol'] = 1.e-2


    def init(self):
        pass

    def define_trajectory_constraints(self, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
        x, y = splines
        ddx, ddy = x.derivative(2), y.derivative(2)
        dddx, dddy = x.derivative(3), y.derivative(3)
        g_tf = self.g*(horizon_time**2)
        self.define_constraint(-(ddx**2 + (ddy+g_tf)**2) +
                               (horizon_time**4)*self.u1min**2, -inf, 0.)
        self.define_constraint(
            (ddx**2 + (ddy+g_tf)**2) - (horizon_time**4)*self.u1max**2, -inf, 0.)
        self.define_constraint(-(dddx*(ddy+g_tf) - ddx*dddy) +
                               (ddx**2 + (ddy + g_tf)**2)*(horizon_time*self.u2min), -inf, 0.)
        self.define_constraint(
            (dddx*(ddy+g_tf) - ddx*dddy) - (ddx**2 + (ddy + g_tf)**2)*(horizon_time*self.u2max), -inf, 0.)

    def get_initial_constraints(self, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
        spl0 = self.define_parameter('spl0', 2)
        dspl0 = self.define_parameter('dspl0', 2)
        ddspl0 = self.define_parameter('ddspl0', 2)
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        return [(x, spl0[0]), (y, spl0[1]),
                (dx, horizon_time*dspl0[0]), (dy, horizon_time*dspl0[1]),
                (ddx, (horizon_time**2)*ddspl0[0]), (ddy, (horizon_time**2)*ddspl0[1])]

    def get_terminal_constraints(self, splines):
        position = self.define_parameter('poseT', 2)
        x, y = splines
        term_con = [(x, position[0]), (y, position[1])]
        term_con_der = []
        for d in range(1, self.degree+1):
            term_con_der.extend([(x.derivative(d), 0.), (y.derivative(d), 0.)])
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None):
        self.prediction['state'] = np.r_[state[:2], np.zeros(3)].T
        self.prediction['dspl'] = np.zeros(2)
        self.prediction['ddspl'] = np.zeros(2)

    def set_terminal_conditions(self, position):
        self.poseT = position

    def get_init_spline_value(self):
        init_value = np.zeros((len(self.basis), 2))
        pos0 = self.prediction['state'][:2]
        posT = self.poseT
        for k in range(2):
            init_value[:, k] = np.r_[pos0[k]*np.ones(self.degree), np.linspace(
                pos0[k], posT[k], len(self.basis) - 2*self.degree), posT[k]*np.ones(self.degree)]
        init_value = [init_value]
        return init_value

    def check_terminal_conditions(self):
        tol = self.options['stop_tol']
        if (np.linalg.norm(self.signals['pose'][:2, -1] - self.poseT) > tol or
                np.linalg.norm(self.signals['dspl'][:, -1])) > tol:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = Vehicle.set_parameters(self, current_time)
        parameters[self]['spl0'] = self.prediction['state'][:2]
        parameters[self]['dspl0'] = self.prediction['dspl']
        parameters[self]['ddspl0'] = self.prediction['ddspl']
        parameters[self]['poseT'] = self.poseT
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines, horizon_time=None):
        x, y = splines[0], splines[1]
        self.define_collision_constraints_2d(hyperplanes, environment, [x, y], horizon_time)

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
        signals['dspl'] = np.c_[dx_s, dy_s].T
        signals['ddspl'] = np.c_[ddx_s, ddy_s].T
        return signals

    def state2pose(self, state):
        return np.r_[state[0], state[1], -state[4]]

    def ode(self, state, input):
        theta = state[4]
        u1, u2 = input[0], input[1]
        return np.r_[state[2:4], u1*np.sin(theta), u1*np.cos(theta)-self.g, u2].T

    def draw(self, t=-1):
        theta = self.signals['pose'][2, t]
        cth, sth = np.cos(theta), np.sin(theta)
        rot = np.array([[cth, -sth], [sth, cth]])
        r = self.radius
        h, rw = 0.2*r, (1./3.)*r
        plt_x = [r, r-2*rw, r-rw, r-rw, -r+rw, -r+rw, -r, -r+2*rw]
        plt_y = [h, h, h, 0, 0, h, h, h]
        points = np.vstack((plt_x, plt_y))
        return [], [np.c_[self.signals['pose'][:2, t]] + rot.dot(points)]
