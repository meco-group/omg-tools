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


from ..basics.optilayer import OptiChild
from ..basics.spline_extra import get_interval_T
from ..basics.spline import BSplineBasis, BSpline
from casadi import inf, vertcat, cos, sin
import numpy as np


class Obstacle(object):

    def __new__(cls, initial, shape, trajectories={}, **kwargs):
        if shape.n_dim == 2:
            return Obstacle2D(initial, shape, trajectories, **kwargs)
        if shape.n_dim == 3:
            return Obstacle3D(initial, shape, trajectories, **kwargs)


class ObstaclexD(OptiChild):

    def __init__(self, initial, shape, trajectories={}, **kwargs):
        OptiChild.__init__(self, 'obstacle')
        self.shape = shape
        self.n_dim = shape.n_dim
        self.basis = BSplineBasis([0, 0, 0, 1, 1, 1], 2)
        self.initial = initial
        if 'draw' in kwargs:
            self.draw_obstacle = kwargs['draw']
        else:
            self.draw_obstacle = True
        if 'avoid' in kwargs:
            self.avoid_obstacle = kwargs['avoid']
        else:
            self.avoid_obstacle = True
        self.init_signals(initial, trajectories)
        self.create_splines()

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def create_splines(self):
        # pos, vel, acc
        x = self.define_parameter('x', self.n_dim)
        v = self.define_parameter('v', self.n_dim)
        a = self.define_parameter('a', self.n_dim)
        # pos, vel, acc at time zero of time horizon
        self.t = self.define_symbol('t')
        self.T = self.define_symbol('T')
        v0 = v - self.t*a
        x0 = x - self.t*v0 - 0.5*(self.t**2)*a
        a0 = a
        # DEBUG!!!
        self.x0 = x0
        # pos spline over time horizon
        self.pos_spline = [BSpline(self.basis, vertcat(x0[k], 0.5*v0[k]*self.T + x0[k], x0[k] + v0[k]*self.T + 0.5*a0[k]*(self.T**2)))
                           for k in range(self.n_dim)]

    def define_collision_constraints(self, hyperplanes):
        raise ValueError('Please implement this method.')

    def set_parameters(self, current_time):
        parameters = {}
        parameters['x'] = self.signals['position'][:, -1]
        parameters['v'] = self.signals['velocity'][:, -1]
        parameters['a'] = self.signals['acceleration'][:, -1]
        return parameters

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def init_signals(self, initial, trajectories):
        # initialize trajectories
        self.trajectories = trajectories
        self.traj_times, self.index = {}, {}
        for key in trajectories:
            self.traj_times[key] = sorted(trajectories[key])
            self.index[key] = 0
        # initialize signals
        self.signals = {}
        self.signals['time'] = np.c_[0.]
        for key in ['position', 'velocity', 'acceleration']:
            if key in initial:
                self.signals[key] = np.c_[initial[key]]
            else:
                self.signals[key] = np.zeros((self.n_dim, 1))

    def update(self, update_time, sample_time):
        n_samp = int(update_time/sample_time)
        for k in range(n_samp):
            dpos, dvel, dacc = np.zeros(2), np.zeros(2), np.zeros(2)
            t1_vel, t2_vel = sample_time, 0.
            t1_acc, t2_acc = sample_time, 0.
            time = self.signals['time'][:, -1] + sample_time
            increment, t1, t2 = {}, {}, {}
            for key in ['position', 'velocity', 'acceleration']:
                increment[key] = np.zeros(self.n_dim)
                t1[key], t2[key] = sample_time, 0.
                if key in self.trajectories and self.index[key] < len(self.traj_times[key]):
                    if np.round(time - self.traj_times[key][self.index[key]], 3) >= 0:
                        t = self.traj_times[key][self.index[key]]
                        increment[key] = self.trajectories[key][t]
                        t1[key] = sample_time - (time - t)
                        t2[key] = time - t
                        self.index[key] += 1
            pos0 = self.signals['position'][:, -1]
            vel0 = self.signals['velocity'][:, -1]
            acc0 = self.signals['acceleration'][:, -1]
            dpos = increment['position']
            dvel = increment['velocity']
            dacc = increment['acceleration']
            t1_vel, t2_vel = t1['velocity'], t2['velocity']
            t1_acc, t2_acc = t1['acceleration'], t2['acceleration']

            position = (pos0 + dpos + t1_vel*vel0 + t2_vel*(vel0 + dvel) +
                        0.5*(t1_acc**2)*acc0 + 0.5*(t2_acc**2)*(acc0 + dacc))
            velocity = (vel0 + dvel + t1_acc*acc0 + t2_acc*(acc0 + dacc))
            acceleration = acc0 + dacc

            self.signals['time'] = np.c_[self.signals['time'], time]
            self.signals['position'] = np.c_[
                self.signals['position'], position]
            self.signals['velocity'] = np.c_[
                self.signals['velocity'], velocity]
            self.signals['acceleration'] = np.c_[
                self.signals['acceleration'], acceleration]

    def draw(self, t=-1):
        if not self.draw_obstacle:
            return []
        pose = np.zeros(2*self.n_dim)
        pose[:self.n_dim] = self.signals['position'][:, t]
        return self.shape.draw(pose)


class Obstacle2D(ObstaclexD):

    def __init__(self, initial, shape, trajectories={}, **kwargs):
        # horizon time (for nurbs representation of cos & sin)
        if 'horizon_time' in kwargs:
            self.horizon_time = kwargs['horizon_time']
        else:
            self.horizon_time = None
        ObstaclexD.__init__(self, initial, shape, trajectories, **kwargs)

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def create_splines(self):
        ObstaclexD.create_splines(self)
        theta = self.define_parameter('theta', 1)
        if self.signals['angular_velocity'][:, -1] == 0.:
            self.cos = cos(theta)
            self.sin = sin(theta)
            self.gon_weight = 1.
            return
        # theta, omega
        omega = self.define_parameter('omega', 1)
        # theta, omega at time zero of time horizon
        theta0 = theta - self.t*omega
        # cos, sin, weight spline over time horizon
        omega = self.signals['angular_velocity'][:, -1][0]
        Ts = 2.*np.pi/abs(omega)
        if self.horizon_time is None:
            raise ValueError(
                'You need to provide a horizon time when using rotating obstacles!')
        else:
            T = self.horizon_time
        n_quarters = int(np.ceil(4*T/Ts))
        knots_theta = np.r_[np.zeros(3), np.hstack(
            [0.25*k*np.ones(2) for k in range(1, n_quarters+1)]), 0.25*n_quarters]*(Ts/T)
        Tf, knots = get_interval_T(BSplineBasis(knots_theta, 2), 0, 1.)
        basis = BSplineBasis(knots, 2)
        # coefficients based on nurbs representation of circle
        cos_cfs = np.r_[1., np.sqrt(
            2.)/2., 0., -np.sqrt(2.)/2., -1., -np.sqrt(2.)/2.,  0.,  np.sqrt(2.)/2., 1.]
        sin_cfs = np.r_[0., np.sqrt(
            2.)/2., 1.,  np.sqrt(2.)/2.,  0., -np.sqrt(2.)/2., -1., -np.sqrt(2.)/2., 0.]
        weight_cfs = np.r_[1., np.sqrt(
            2.)/2., 1.,  np.sqrt(2.)/2.,  1.,  np.sqrt(2.)/2.,  1.,  np.sqrt(2.)/2., 1.]
        cos_cfs = Tf.dot(np.array([cos_cfs[k % 8] for k in range(len(basis))]))
        sin_cfs = Tf.dot(np.array([sin_cfs[k % 8] for k in range(len(basis))]))
        weight_cfs = Tf.dot(
            np.array([weight_cfs[k % 8] for k in range(len(basis))]))

        cos_wt = BSpline(basis, cos_cfs)
        sin_wt = BSpline(basis, sin_cfs)*np.sign(omega)
        self.cos = cos_wt*cos(theta0) - sin_wt*sin(theta0)
        self.sin = cos_wt*sin(theta0) + sin_wt*cos(theta0)
        self.gon_weight = BSpline(basis, weight_cfs)

    def define_collision_constraints(self, hyperplanes):
        for hyperplane in hyperplanes:
            a, b = hyperplane['a'], hyperplane['b']
            checkpoints, rad = self.shape.get_checkpoints()
            for l, chck in enumerate(checkpoints):
                # xpos = self.pos_spline[0]*self.gon_weight + chck[0]*self.cos - chck[1]*self.sin
                # ypos = self.pos_spline[1]*self.gon_weight + chck[0]*self.sin + chck[1]*self.cos
                xpos = self.x0[0]*self.gon_weight + \
                    chck[0]*self.cos - chck[1]*self.sin
                ypos = self.x0[1]*self.gon_weight + \
                    chck[0]*self.sin + chck[1]*self.cos
                self.define_constraint(-(a[0]*xpos + a[1] *
                                         ypos) + self.gon_weight*(b+rad[l]), -inf, 0.)

    def set_parameters(self, current_time):
        parameters = ObstaclexD.set_parameters(self, current_time)
        parameters['theta'] = self.signals['orientation'][:, -1]
        parameters['omega'] = self.signals['angular_velocity'][:, -1]
        # print parameters['theta'] - parameters['omega']*current_time
        return parameters

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def init_signals(self, initial, trajectories):
        ObstaclexD.init_signals(self, initial, trajectories)
        # initialize signals
        for key in ['orientation', 'angular_velocity']:
            if key in initial:
                self.signals[key] = np.c_[initial[key]]
            else:
                self.signals[key] = np.zeros((1, 1))

    def update(self, update_time, sample_time):
        ObstaclexD.update(self, update_time, sample_time)
        n_samp = int(update_time/sample_time)
        for k in range(n_samp):
            theta0 = self.signals['orientation'][:, -1][0]
            omega0 = self.signals['angular_velocity'][:, -1][0]
            theta = theta0 + sample_time*omega0
            omega = omega0
            self.signals['orientation'] = np.c_[
                self.signals['orientation'], theta]
            self.signals['angular_velocity'] = np.c_[
                self.signals['angular_velocity'], omega]

    def draw(self, t=-1):
        if not self.draw_obstacle:
            return []
        pose = np.zeros(2*self.n_dim)
        pose[:self.n_dim] = self.signals['position'][:, t]
        pose[2] = self.signals['orientation'][:, t]
        return self.shape.draw(pose)


class Obstacle3D(ObstaclexD):

    def __init__(self, initial, shape, trajectories, **kwargs):
        ObstaclexD.__init__(self, initial, shape, trajectories, **kwargs)

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def define_collision_constraints(self, hyperplanes):
        for hyperplane in hyperplanes:
            a, b = hyperplane['a'], hyperplane['b']
            checkpoints, rad = self.shape.get_checkpoints()
            for l, chck in enumerate(checkpoints):
                self.define_constraint(-sum([a[k]*(chck[k]+self.pos_spline[k])
                                             for k in range(self.n_dim)]) + b + rad[l], -inf, 0.)
