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
from ..execution.plotlayer import PlotLayer
from obstacle import Obstacle
from casadi import inf
import numpy as np


class Environment(OptiChild, PlotLayer):

    def __init__(self, room, obstacles=None):
        obstacles = obstacles or []
        OptiChild.__init__(self, 'environment')
        PlotLayer.__init__(self)

        # create room and define dimension of the space
        self.room, self.n_dim = room, room['shape'].n_dim
        if 'position' not in room:
            self.room['position'] = [0. for k in range(self.n_dim)]
        if not ('orientation' in room):
            if self.n_dim == 2:
                self.room['orientation'] = 0.
            elif self.n_dim == 3:
                self.room['orientation'] = [0., 0., 0.]  # Euler angles
            else:
                raise ValueError('You defined a shape with dimension '
                                 + str(self.n_dim) + ', which is invalid.')
        if 'draw' not in room:
            self.room['draw'] = False

        # add obstacles
        self.obstacles, self.n_obs = [], 0
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

    # ========================================================================
    # Copy function
    # ========================================================================

    def copy(self):
        obstacles = [Obstacle(o.initial, o.shape, o.simulation, o.options)
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
        degree = 1
        knots = np.r_[np.zeros(degree),
                      vehicle.knots[
                          vehicle.degree:-vehicle.degree],
                      np.ones(degree)]
        basis = BSplineBasis(knots, degree)
        hyp_veh, hyp_obs = {}, {}
        for k, shape in enumerate(vehicle.shapes):
            hyp_veh[shape] = []
            for l, obstacle in enumerate(self.obstacles):
                if obstacle.options['avoid']:
                    if obstacle not in hyp_obs:
                        hyp_obs[obstacle] = []
                    a = self.define_spline_variable(
                        'a'+'_'+vehicle.label+'_'+str(k)+str(l), self.n_dim, basis=basis)
                    b = self.define_spline_variable(
                        'b'+'_'+vehicle.label+'_'+str(k)+str(l), 1, basis=basis)[0]
                    self.define_constraint(
                        sum([a[p]*a[p] for p in range(self.n_dim)])-1, -inf, 0.)
                    hyp_veh[shape].append({'a': a, 'b': b})
                    hyp_obs[obstacle].append({'a': a, 'b': b})
        for obstacle in self.obstacles:
            if obstacle.options['avoid']:
                obstacle.define_collision_constraints(hyp_obs[obstacle])
        for spline in vehicle.splines:
            vehicle.define_collision_constraints(hyp_veh, self, spline)

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def init(self):
        for obstacle in self.obstacles:
            obstacle.init()

    # ========================================================================
    # Simulate environment
    # ========================================================================

    def simulate(self, simulation_time, sample_time):
        for obstacle in self.obstacles:
            obstacle.simulate(simulation_time, sample_time)
        self.update_plots()

    def draw(self, t=-1):
        draw = []
        for obstacle in self.obstacles:
            draw.extend(obstacle.draw(t))
        if self.room['draw']:
            draw.extend(self.room['shape'].draw())
        return draw

    def get_canvas_limits(self):
        limits = self.room['shape'].get_canvas_limits()
        return [limits[k]+self.room['position'][k] for k in range(self.n_dim)]

    # ========================================================================
    # Plot related functions
    # ========================================================================

    def init_plot(self, argument, **kwargs):
        lines = [{'color': 'black', 'linewidth': 1.2} for _ in self.draw()]
        if 'limits' in kwargs:
            limits = kwargs['limits']
        else:
            limits = self.get_canvas_limits()
        labels = ['' for k in range(self.n_dim)]
        if self.n_dim == 2:
            return [[{'labels': labels, 'lines': lines, 'aspect_equal': True,
                      'xlim': limits[0], 'ylim': limits[1]}]]
        else:
            return [[{'labels': labels, 'lines': lines, 'aspect_equal': True,
                      'xlim': limits[0], 'ylim': limits[1], 'zlim': limits[2],
                      'projection': '3d'}]]

    def update_plot(self, argument, t, **kwargs):
        lines = []
        for l in self.draw(t):
            lines.append([l[k, :] for k in range(self.n_dim)])
        return [[lines]]
