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
from ..basics.shape import Sphere
from ..basics.spline import BSplineBasis
from ..basics.spline_extra import sample_splines
from ..basics.spline_extra import evalspline, running_integral, concat_splines, definite_integral
from casadi import inf, SX, MX
import numpy as np
import time

# Vehicle model:
# ddx = (F/m)*cos(phi)*sin(theta)
# ddy = -(F/m)*sin(theta)
# ddz = (F/m)*cos(phi)*cos(theta) - g
# dphi = omega_phi
# dtheta = omega_theta

# Vehicle inputs
# u1 = F/m
# u2 = omega_phi
# u3 = omega_theta

# Output trajectory
# y1 = x
# y2 = y
# y3 = z

class Quadrotor3D(Vehicle):

    def __init__(self, radius=0.2, options=None, bounds=None):
        bounds = bounds or {}
        Vehicle.__init__(
            self, n_spl=3, degree=2, shapes=Sphere(radius), options=options)

        self.u1min = bounds['u1min'] if 'u1min' in bounds else 2.
        self.u1max = bounds['u1max'] if 'u1max' in bounds else 15.
        self.u2min = bounds['u2min'] if 'u2min' in bounds else -2.
        self.u2max = bounds['u2max'] if 'u2max' in bounds else 2.
        self.u3min = bounds['u3min'] if 'u3min' in bounds else -2.
        self.u3max = bounds['u3max'] if 'u3max' in bounds else 2.
        self.phimin = bounds['phimin'] if 'phimin' in bounds else -np.pi/6
        self.phimax = bounds['phimax'] if 'phimax' in bounds else np.pi/6
        self.thetamin = bounds['thetamin'] if 'thetamin' in bounds else -np.pi/6
        self.thetamax = bounds['thetamax'] if 'thetamax' in bounds else np.pi/6
        self.g = 9.81
        self.radius = radius

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options['stop_tol'] = 1.e-2
        self.options['substitution'] = True
        self.options['exact_substitution'] = False

    def init(self):
        # time horizon
        self.T = self.define_symbol('T')
        self.t = self.define_symbol('t')  # current time of first knot
        self.pos0 = self.define_parameter('pos0', 3)  # current position
        self.dpos0 = self.define_parameter('dpos0', 3)  # current velocity

    def define_trajectory_constraints(self, splines):
        f_til, q_phi, q_theta = splines
        dq_phi = q_phi.derivative()
        dq_theta = q_theta.derivative()
        # constraints on u1
        self.define_constraint(f_til*(1+q_phi**2)*(1+q_theta**2) - self.u1max, -inf, 0)
        self.define_constraint(-f_til*(1+q_phi**2)*(1+q_theta**2) + self.u1min, -inf, 0)
        # constraints on u2
        self.define_constraint(2*dq_phi - (1+q_phi**2)*self.T*self.u2max, -inf, 0.)
        self.define_constraint(-2*dq_phi + (1+q_phi**2)*self.T*self.u2min, -inf, 0.)
        # constraints on u3
        self.define_constraint(2*dq_theta - (1+q_theta**2)*self.T*self.u3max, -inf, 0.)
        self.define_constraint(-2*dq_theta + (1+q_theta**2)*self.T*self.u3min, -inf, 0.)
        # constraints on phi
        self.define_constraint(q_phi - np.tan(0.5*self.phimax), -inf, 0)
        self.define_constraint(-q_phi + np.tan(0.5*self.phimin), -inf, 0)
        self.define_constraint(q_theta - np.tan(0.5*self.thetamax), -inf, 0)
        self.define_constraint(-q_theta + np.tan(0.5*self.thetamin), -inf, 0)

        if self.options['substitution']:
            # substitute acceleration and introduce equality constraints
            ddx = f_til*(1-q_phi**2)*(2*q_theta)
            ddy = -f_til*(1+q_theta**2)*(2*q_phi)
            ddz = f_til*(1-q_phi**2)*(1-q_theta**2) - self.g
            if self.options['exact_substitution']:
                self.ddx = self.define_spline_variable('ddx', 1, 1, basis=ddx.basis)[0]
                self.ddy = self.define_spline_variable('ddy', 1, 1, basis=ddy.basis)[0]
                self.ddz = self.define_spline_variable('ddz', 1, 1, basis=ddz.basis)[0]
                self.x, self.dx = self.integrate_twice(self.ddx, self.dpos0[0], self.pos0[0], self.t, self.T)
                self.y, self.dy = self.integrate_twice(self.ddy, self.dpos0[1], self.pos0[1], self.t, self.T)
                self.z, self.dz = self.integrate_twice(self.ddz, self.dpos0[2], self.pos0[2], self.t, self.T)
                self.define_constraint(self.ddx - ddx, 0, 0)
                self.define_constraint(self.ddy - ddy, 0, 0)
                self.define_constraint(self.ddz - ddz, 0, 0)
            else:
                degree = 4
                knots = np.r_[np.zeros(degree), np.linspace(0., 1., 10+1), np.ones(degree)]
                basis = BSplineBasis(knots, degree)
                self.ddx = self.define_spline_variable('ddx', 1, 1, basis=basis)[0]
                self.ddy = self.define_spline_variable('ddy', 1, 1, basis=basis)[0]
                self.ddz = self.define_spline_variable('ddz', 1, 1, basis=basis)[0]
                self.x, self.dx = self.integrate_twice(self.ddx, self.dpos0[0], self.pos0[0], self.t, self.T)
                self.y, self.dy = self.integrate_twice(self.ddy, self.dpos0[1], self.pos0[1], self.t, self.T)
                self.z, self.dz = self.integrate_twice(self.ddz, self.dpos0[2], self.pos0[2], self.t, self.T)
                x, dx = self.integrate_twice(ddx, self.dpos0[0], self.pos0[0], self.t, self.T)
                y, dy = self.integrate_twice(ddy, self.dpos0[1], self.pos0[1], self.t, self.T)
                z, dz = self.integrate_twice(ddz, self.dpos0[2], self.pos0[2], self.t, self.T)
                # eps = 5e-2
                # self.define_constraint(definite_integral(self.ddx - ddx, 0, 1), 0, 0)
                # self.define_constraint(definite_integral(self.ddy - ddy, 0, 1), 0, 0)
                # self.define_constraint(definite_integral(self.ddz - ddz, 0, 1), 0, 0)
                eps = 1e-3
                self.define_constraint(self.x-x, -eps, eps)
                self.define_constraint(self.y-y, -eps, eps)
                self.define_constraint(self.z-z, -eps, eps)

    def get_initial_constraints(self, splines):
        # inputs are function of f_til, q_phi, dq_phi, q_theta and dq_theta -> impose constraints on these
        f_til0 = self.define_parameter('f_til0', 1)
        q_phi0 = self.define_parameter('q_phi0', 1)
        q_theta0 = self.define_parameter('q_theta0', 1)
        dq_phi0 = self.define_parameter('dq_phi0', 1)
        dq_theta0 = self.define_parameter('dq_theta0', 1)

        f_til, q_phi, q_theta = splines
        dq_phi, dq_theta = q_phi.derivative(), q_theta.derivative()
        return [(f_til, f_til0), (q_phi, q_phi0), (q_theta, q_theta0),
                (dq_phi, self.T*dq_phi0), (dq_theta, self.T*dq_theta0)]

    def get_terminal_constraints(self, splines):
        posT = self.define_parameter('posT', 3)
        q_phiT = self.define_parameter('q_phiT', 1)
        q_thetaT = self.define_parameter('q_thetaT', 1)
        f_til, q_phi, q_theta = splines
        if self.options['substitution']:
            x, y, z = self.x, self.y, self.z
            dx, dy, dz = self.dx, self.dy, self.dz
        else:
            ddx = f_til*(1-q_phi**2)*(2*q_theta)
            ddy = -f_til*(1+q_theta**2)*(2*q_phi)
            ddz = f_til*(1-q_phi**2)*(1-q_theta**2) - self.g
            x, dx = self.integrate_twice(ddx, self.dpos0[0], self.pos0[0], self.t, self.T)
            y, dy = self.integrate_twice(ddy, self.dpos0[1], self.pos0[1], self.t, self.T)
            z, dz = self.integrate_twice(ddz, self.dpos0[2], self.pos0[2], self.t, self.T)
        # soft constraint on position and hard constraint on orientation
        term_con = [(x, posT[0]), (y, posT[1]), (z, posT[2])]
        term_con_der = [(q_phi, q_phiT), (q_theta, q_thetaT), (f_til, self.g),
                        (q_phi.derivative(), 0.), (q_theta.derivative(), 0.),
                        (dx, 0.), (dy, 0.), (dz, 0.)]
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None):
        if input is None:
            input = np.array([self.g, 0., 0.])
        self.prediction['state'] = state
        self.prediction['input'] = input
        self.pose0 = np.r_[state[:3], state[6:], 0.]

    def set_terminal_conditions(self, position, roll=0, pitch=0):
        self.poseT = np.r_[position, roll, pitch, 0.].T

    def get_init_spline_value(self):
        init_value = np.zeros((len(self.basis), 3))
        f_til0 = np.zeros(len(self.basis))
        q_phi0 = np.tan(self.prediction['state'][6]/2.)
        q_theta0 = np.tan(self.prediction['state'][7]/2.)
        q_phiT = np.tan(self.poseT[3]/2.)
        q_thetaT = np.tan(self.poseT[4]/2.)
        init_value[:, 0] = f_til0
        init_value[:, 1] = np.linspace(q_phi0, q_phiT, len(self.basis))
        init_value[:, 2] = np.linspace(q_theta0, q_thetaT, len(self.basis))
        return init_value

    def check_terminal_conditions(self):
        tol = self.options['stop_tol']
        if (np.linalg.norm(self.signals['pose'][:3, -1] - self.poseT[:3]) > tol or
                np.linalg.norm(self.signals['input'][:, -1]) - self.g > tol):
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = Vehicle.set_parameters(self, current_time)
        parameters['q_phi0'] = np.tan(self.prediction['state'][6]/2.)
        parameters['q_theta0'] = np.tan(self.prediction['state'][7]/2.)
        parameters['f_til0'] = self.prediction['input'][0]/((1+parameters['q_phi0']**2)*(1+parameters['q_theta0']**2))
        parameters['dq_phi0'] = 0.5*self.prediction['input'][1]*(1+parameters['q_phi0']**2)
        parameters['dq_theta0'] = 0.5*self.prediction['input'][2]*(1+parameters['q_theta0']**2)
        parameters['pos0'] = self.prediction['state'][:3]
        parameters['dpos0'] = self.prediction['state'][3:6]
        parameters['posT'] = self.poseT[:3]
        parameters['q_phiT'] = np.tan(self.poseT[3]/2.)
        parameters['q_thetaT'] = np.tan(self.poseT[4]/2.)
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines):
        f_til, q_phi, q_theta = splines
        if self.options['substitution']:
            x, y, z = self.x, self.y, self.z
        else:
            ddx = f_til*(1-q_phi**2)*(2*q_theta)
            ddy = -f_til*(1+q_theta**2)*(2*q_phi)
            ddz = f_til*(1-q_phi**2)*(1-q_theta**2) - self.g
            x, _ = self.integrate_twice(ddx, self.dpos0[0], self.pos0[0], self.t, self.T)
            y, _ = self.integrate_twice(ddy, self.dpos0[1], self.pos0[1], self.t, self.T)
            z, _ = self.integrate_twice(ddz, self.dpos0[2], self.pos0[2], self.t, self.T)

        self.define_collision_constraints_3d(hyperplanes, environment, [x, y, z])

    def integrate_twice(self, ddx, dx0, x0, t, T=1.):
        # first integration
        ddx_int = T*running_integral(ddx)
        if isinstance(t, (SX, MX)):
            dx = ddx_int-evalspline(ddx_int, t/T) + dx0
        else:
            dx = ddx_int-ddx_int(t/T) + dx0
        # second integration
        dx_int = T*running_integral(dx)
        if isinstance(t, (SX, MX)):
            x = dx_int-evalspline(dx_int, t/T) + x0
        else:
            x = dx_int-dx_int(t/T) + x0
        return x, dx

    def splines2signals(self, splines, time):
        signals = {}
        f_til, q_phi, q_theta = splines
        dq_phi, dq_theta = q_phi.derivative(), q_theta.derivative()
        ddx = f_til*(1-q_phi**2)*(2*q_theta)
        ddy = -f_til*(1+q_theta**2)*(2*q_phi)
        ddz = f_til*(1-q_phi**2)*(1-q_theta**2) - self.g
        if not hasattr(self, 'signals'): # first iteration
            x, dx = self.integrate_twice(ddx, 0., self.pose0[0], time[0])
            y, dy = self.integrate_twice(ddy, 0., self.pose0[1], time[0])
            z, dz = self.integrate_twice(ddz, 0., self.pose0[2], time[0])
        else:
            x, dx = self.integrate_twice(ddx, self.signals['state'][3, -1], self.signals['state'][0, -1], time[0])
            y, dy = self.integrate_twice(ddy, self.signals['state'][4, -1], self.signals['state'][1, -1], time[0])
            z, dz = self.integrate_twice(ddz, self.signals['state'][5, -1], self.signals['state'][2, -1], time[0])
        x_s, y_s, z_s, dx_s, dy_s, dz_s = sample_splines([x, y, z, dx, dy, dz], time)
        f_til_s, q_phi_s, q_theta_s, dq_phi_s, dq_theta_s = sample_splines([f_til, q_phi, q_theta, dq_phi, dq_theta], time)
        den = sample_splines([(1+q_phi**2)*(1+q_theta**2)], time)[0]
        phi = 2*np.arctan2(q_phi_s, 1)
        theta = 2*np.arctan2(q_theta_s, 1)
        dphi = 2*np.array(dq_phi_s)/(1.+np.array(q_phi_s)**2)
        dtheta = 2*np.array(dq_theta_s)/(1.+np.array(q_theta_s)**2)
        f_s = f_til_s*den
        signals['state'] = np.c_[x_s, y_s, z_s, dx_s, dy_s, dz_s, phi, theta].T
        signals['input'] = np.c_[f_s, dphi.T, dtheta.T].T

        if (self.options['substitution'] and not self.options['exact_substitution']):  # don't plot error for exact_subs
            ddx2 = self.problem.father.get_variables(self, 'ddx')
            ddy2 = self.problem.father.get_variables(self, 'ddy')
            ddz2 = self.problem.father.get_variables(self, 'ddz')
            ddx2 = concat_splines([ddx2], [self.problem.options['horizon_time']])[0]
            ddy2 = concat_splines([ddy2], [self.problem.options['horizon_time']])[0]
            ddz2 = concat_splines([ddz2], [self.problem.options['horizon_time']])[0]
            if not hasattr(self, 'signals'): # first iteration
                x2, dx2 = self.integrate_twice(ddx2, 0., self.pose0[0], time[0])
                y2, dy2 = self.integrate_twice(ddy2, 0., self.pose0[1], time[0])
                z2, dz2 = self.integrate_twice(ddz2, 0., self.pose0[2], time[0])
            else:
                x2, dx2 = self.integrate_twice(ddx2, self.signals['state'][3, -1], self.signals['state'][0, -1], time[0])
                y2, dy2 = self.integrate_twice(ddy2, self.signals['state'][4, -1], self.signals['state'][1, -1], time[0])
                z2, dz2 = self.integrate_twice(ddz2, self.signals['state'][5, -1], self.signals['state'][2, -1], time[0])
            ddx_s, ddy_s, ddz_s = sample_splines([ddx, ddy, ddz], time)
            ddx_s2, ddy_s2, ddz_s2 = sample_splines([ddx2, ddy2, ddz2], time)
            x_s2, y_s2, z_s2, dx_s2, dy_s2, dz_s2 = sample_splines([x2, y2, z2, dx2, dy2, dz2], time)
            signals['err_ddpos'] = np.c_[ddx_s-ddx_s2, ddy_s-ddy_s2, ddz_s-ddz_s2].T
            signals['err_dpos'] = np.c_[dx_s-dx_s2, dy_s-dy_s2, dz_s-dz_s2].T
            signals['err_pos'] = np.c_[x_s-x_s2, y_s-y_s2, z_s-z_s2].T
        return signals

    def state2pose(self, state):
        return np.r_[state[0], state[1], state[2], state[6], state[7], 0.]

    def ode(self, state, input):
        phi = state[6]
        theta = state[7]
        u1, u2, u3 = input[0], input[1], input[2]
        return np.r_[state[3:6], u1*np.sin(theta)*np.cos(phi), -u1*np.sin(phi), -self.g + u1*np.cos(phi)*np.cos(theta), u2, u3].T

    def draw(self, t=-1):
        phi, theta = self.signals['pose'][3, t], self.signals['pose'][4, t]
        cth, sth = np.cos(theta), np.sin(theta)
        cphi, sphi = np.cos(phi), np.sin(phi)
        rot = np.array([[cth, sphi*sth, cphi*sth],
                        [0, cphi, -sphi],
                        [-sth, sphi*cth, cphi*cth]])

        r = self.radius
        h, rw = 0.2*r, (1./3.)*r
        s = np.linspace(0, 1-1./48, 48)

        # frame
        plt_xy = [r-rw, r-rw, -r+rw, -r+rw]
        plt_z = [h, 0, 0, h]
        points = np.vstack((plt_xy, np.zeros(len(plt_xy)), plt_z))
        points = rot.dot(points) + np.c_[self.signals['pose'][:3, t]]
        n_points = points.shape[1]
        lines = [points]

        # lines = [np.c_[points[:, l], points[:, l+1]] for l in range(n_points-1)]

        points = np.vstack((np.zeros(len(plt_xy)), plt_xy, plt_z))
        points = rot.dot(points) + np.c_[self.signals['pose'][:3, t]]
        n_points = points.shape[1]
        lines += [points]
        # lines += [np.c_[points[:, l], points[:, l+1]] for l in range(n_points-1)]

        # rotors
        circle_h = np.vstack((rw*np.cos(s*2*np.pi), rw*np.sin(s*2*np.pi), np.zeros(len(s)), ))
        for i, j in zip([r-rw, 0, -r+rw, 0], [0, r-rw, 0, -r+rw]):
            points = rot.dot(circle_h + np.vstack((i, j, h))) + np.c_[self.signals['pose'][:3, t]]
            lines += [points]
            # n_points = points.shape[1]
            # lines += [np.c_[points[:, l], points[:, l+1]] for l in range(n_points-1)]

        return [], lines
