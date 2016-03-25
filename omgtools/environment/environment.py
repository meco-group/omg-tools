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
from ..basics.spline import BSplineBasis, BSpline
from casadi import inf, vertcat
import numpy as np


class Environment(OptiChild):

    def __init__(self, room, obstacles=[]):
        OptiChild.__init__(self, 'environment')

        # create room and define dimension of the space
        self.room, self.n_dim = room, room['shape'].n_dim
        if not ('position' in room):
            self.room['position'] = [0. for k in range(self.n_dim)]

        # add obstacles
        self.obstacles, self.n_obs = [], 0
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

    # ========================================================================
    # Copy function
    # ========================================================================

    def copy(self):
        obstacles = [Obstacle(o.initial, o.shape, o.trajectories)
                     for o in self.obstacles]
        return Environment(self.room, obstacles)

    # ========================================================================
    # Add obstacles/vehicles
    # ========================================================================

    def add_obstacle(self, obstacle):
        if isinstance(obstacle, list):
            for obst in obstacle:
                self.add_obstacle(obst)
        else:
            if obstacle.n_dim != self.n_dim:
                raise ValueError('Not possible to combine ' +
                                 str(obstacle.n_dim) + 'D obstacle with ' +
                                 str(self.n_dim) + 'D environment.')
            self.obstacles.append(obstacle)
            self.n_obs += 1

    def define_collision_constraints(self, vehicle, splines):
        if vehicle.n_dim != self.n_dim:
            raise ValueError('Not possible to combine ' +
                             str(vehicle.n_dim) + 'D vehicle with ' +
                             str(self.n_dim) + 'D environment.')
        hyp_veh, hyp_obs = {}, {}
        for k, shape in enumerate(vehicle.shapes):
            hyp_veh[shape] = []
            for l, obstacle in enumerate(self.obstacles):
                if obstacle not in hyp_obs:
                    hyp_obs[obstacle] = []
                degree = 1
                knots = np.r_[np.zeros(degree),
                              vehicle.knots[vehicle.degree:-vehicle.degree],
                              np.ones(degree)]
                basis = BSplineBasis(knots, degree)
                a = self.define_spline_variable(
                    'a'+str(k)+str(l), self.n_dim, basis=basis)
                b = self.define_spline_variable(
                    'b'+str(k)+str(l), 1, basis=basis)[0]
                self.define_constraint(
                    sum([a[p]*a[p] for p in range(self.n_dim)])-1, -inf, 0.)
                hyp_veh[shape].append({'a': a, 'b': b})
                hyp_obs[obstacle].append({'a': a, 'b': b})
        for obstacle in self.obstacles:
            obstacle.define_collision_constraints(hyp_obs[obstacle])
        limits = self.get_canvas_limits()
        for spline in vehicle.splines:
            vehicle.define_collision_constraints(hyp_veh, limits, spline)
        self.sample_time = vehicle.options['sample_time']

    # ========================================================================
    # Update environment
    # ========================================================================

    def update(self, update_time):
        for obstacle in self.obstacles:
            # obstacle.update(update_time, self.sample_time)
            obstacle.update(update_time, 0.01)

    def draw(self, t=-1):
        draw = []
        for obstacle in self.obstacles:
            draw.append(obstacle.draw(t))
        return draw

    def get_canvas_limits(self):
        limits = self.room['shape'].get_canvas_limits()
        return [limits[k]+self.room['position'][k] for k in range(self.n_dim)]


class Obstacle(OptiChild):

    def __init__(self, initial, shape, trajectories={}):
        OptiChild.__init__(self, 'obstacle')
        self.shape = shape
        self.n_dim = shape.n_dim
        self.basis = BSplineBasis([0, 0, 0, 1, 1, 1], 2)
        self.initial = initial

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
        # pos spline over time horizon
        self.pos_spline = [BSpline(self.basis, vertcat(
            [x0[k], 0.5*v0[k]*self.T + x0[k],
             x0[k] + v0[k]*self.T + 0.5*a0[k]*(self.T**2)]))
            for k in range(self.n_dim)]

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

    def set_parameters(self, current_time):
        parameters = {}
        parameters['x'] = self.signals['position'][:, -1]
        parameters['v'] = self.signals['velocity'][:, -1]
        parameters['a'] = self.signals['acceleration'][:, -1]
        return parameters

    # ========================================================================
    # Simulation related functions
    # ========================================================================

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
                        t1[key] = t - time + update_time
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
        pose = np.zeros(2*self.n_dim)
        pose[:self.n_dim] = self.signals['position'][:, t]
        return self.shape.draw(pose)
