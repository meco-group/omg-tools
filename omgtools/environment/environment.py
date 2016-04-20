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
from ..basics.spline import BSplineBasis
from obstacle import Obstacle
from casadi import inf
import numpy as np


class Environment(OptiChild):

    def __init__(self, room, obstacles=[]):
        OptiChild.__init__(self, 'environment')

        # create room and define dimension of the space
        self.room, self.n_dim = room, room['shape'].n_dim
        if not ('position' in room):
            self.room['position'] = [0. for k in range(self.n_dim)]
        if not ('orientation' in room):
            if self.n_dim == 2:
                self.room['orientation'] = 0.
            elif self.n_dim == 3:
                self.room['orientation'] = [0., 0., 0.]  # Euler angles
            else:
                raise ValueError('You defined a shape with dimension ' 
                                 + str(self.n_dim) + ', which is invalid.')

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
                if obstacle.avoid_obstacle:
                    if obstacle not in hyp_obs:
                        hyp_obs[obstacle] = []
                    degree = 1
                    knots = np.r_[np.zeros(degree),
                                  vehicle.knots[
                                      vehicle.degree:-vehicle.degree],
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
            if obstacle.avoid_obstacle:
                obstacle.define_collision_constraints(hyp_obs[obstacle])
        for spline in vehicle.splines:
            vehicle.define_collision_constraints(hyp_veh, self, spline)
        self.sample_time = vehicle.options['sample_time']

    # ========================================================================
    # Update environment
    # ========================================================================

    def update(self, update_time):
        for obstacle in self.obstacles:
            obstacle.update(update_time, self.sample_time)

    def draw(self, t=-1):
        draw = []
        for obstacle in self.obstacles:
            draw.append(obstacle.draw(t))
        draw.append(self.room['shape'].draw())
        return draw

    def get_canvas_limits(self):
        limits = self.room['shape'].get_canvas_limits()
        return [limits[k]+self.room['position'][k] for k in range(self.n_dim)]
