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
from ..execution.plotlayer import PlotLayer, mix_with_white
from obstacle import Obstacle
from casadi import inf
import numpy as np
import warnings


class Environment(OptiChild, PlotLayer):

    def __init__(self, room, obstacles=None):
        obstacles = obstacles or []
        OptiChild.__init__(self, 'environment')
        PlotLayer.__init__(self)

        # create rooms and define dimension of the space
        # note: in general self.room may contain a list of several rooms, this is the case for
        # e.g. multiframeproblems and gcodeproblems
        self.room = room if isinstance(room, list) else [room]
        self.n_dim = self.room[0]['shape'].n_dim  # use dimension of first room for all rooms
        for room in self.room:
            if room['shape'].n_dim != self.n_dim:
                raise ValueError('You try to combine rooms of different dimensions,' +
                                 ' which is invalid')
            if 'position' not in room:
                room['position'] = [0. for k in range(self.n_dim)]
            if not ('orientation' in room):
                if self.n_dim == 2:
                    room['orientation'] = 0.
                elif self.n_dim == 3:
                    room['orientation'] = [0., 0., 0.]  # Euler angles
                else:
                    raise ValueError('You defined a shape with dimension '
                                     + str(self.n_dim) + ', which is invalid.')
            if 'draw' not in room:
                room['draw'] = False

        # add obstacles
        self.obstacles, self.n_obs = [], 0
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

        # add danger zones
        self.danger_zones, self.n_zones = [], 0
        for zone in self.danger_zones:
            self.add_danger_zone(zone)

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
            if obstacle.n_dim == 2 and self.n_dim == 3:
                warnings.warn('You are combining a 2D obstacle with ' +
                                'a 3D environment. The 2D obstacle is transformed ' +
                                ' to a 3D one by extending it infinitely in ' +
                                'z dimension.')
            if obstacle.n_dim == 3 and self.n_dim == 2:
                raise ValueError('Not possible to combine ' +
                                 str(obstacle.n_dim) + 'D obstacle with ' +
                                 str(self.n_dim) + 'D environment.')
            self.obstacles.append(obstacle)
            self.n_obs += 1

    def add_danger_zone(self, danger_zone):
        if isinstance(danger_zone, list):
            for zone in danger_zone:
                self.add_danger_zone(zone)
        else:
            self.danger_zones.append(danger_zone)
            self.n_zones += 1

    def fill_room(self, room, obstacles):
        # if key didn't exist yet, it is created
        # if key existed already, all obstacles are replaced
        idx = self.room.index(room)
        self.room[idx]['obstacles'] = obstacles  # assign to room
        for o in obstacles:
            if not o in self.obstacles:
                self.obstacles += [o]  # save in total list

    def define_collision_constraints(self, vehicle, splines, horizon_times):
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
        for idx in range(vehicle.n_seg):
            # loop over vehicle segments, not over rooms since number of considered segments
            # may be different from total number of rooms
            room = self.room[idx]  # select current room
            hyp_veh, hyp_obs = {}, {}
            # add all obstacles, unless user specified it differently
            if 'obstacles' in room:
                obs_to_add = room['obstacles']
            else:
                obs_to_add = self.obstacles
            for k, shape in enumerate(vehicle.shapes):
                hyp_veh[shape] = []
                for l, obstacle in enumerate(obs_to_add):
                    # Todo: is it a problem to call obstacle.init multiple times?
                    # pass the horizon_time for current segment, and all the previous ones, allowing to
                    # gradually build up the position spline for the obstacle
                    obstacle.init(horizon_times=horizon_times[:idx+1])
                    if obstacle.options['avoid']:
                        if obstacle not in hyp_obs:
                            hyp_obs[obstacle] = []
                        a = self.define_spline_variable(
                            'a'+'_'+vehicle.label+'_'+'seg'+str(idx)+'_'+str(k)+str(l), obstacle.n_dim, basis=basis)
                        b = self.define_spline_variable(
                            'b'+'_'+vehicle.label+'_'+'seg'+str(idx)+'_'+str(k)+str(l), 1, basis=basis)[0]
                        self.define_constraint(
                            sum([a[p]*a[p] for p in range(obstacle.n_dim)])-1, -inf, 0.)
                        if self.n_dim == 3 and obstacle.n_dim == 2:
                            a2 = [a[0], a[1], BSpline(basis, np.zeros(len(basis)))]
                            hyp_veh[shape].append({'a': a2, 'b': b})
                        else:
                            hyp_veh[shape].append({'a': a, 'b': b})
                        hyp_obs[obstacle].append({'a': a, 'b': b})
                        obstacle.define_collision_constraints(hyp_obs[obstacle])
            vehicle.define_collision_constraints(hyp_veh, room, splines[idx], horizon_times[idx])

    def define_intervehicle_collision_constraints(self, vehicles, horizon_times):
        # Todo: added for idx in range(vehicles[0].n_seg) loop, okay?
        # For now supposed that all vehicles have the same amount of segments
        for idx in range(vehicles[0].n_seg):
            hyp_veh = {veh: {sh: [] for sh in veh.shapes} for veh in vehicles}
            for k in range(len(vehicles)):
                for l in range(k+1, len(vehicles)):
                    veh1 = vehicles[k]
                    veh2 = vehicles[l]
                    if veh1 != veh2:
                        if veh1.n_dim != veh2.n_dim:
                            raise ValueError('Not possible to combine ' +
                                             str(veh1.n_dim) + 'D and ' + str(veh2.n_dim) + 'D vehicle.')
                        degree = 1
                        knots = np.r_[np.zeros(degree), np.union1d(veh1.knots[veh1.degree:-veh1.degree], veh2.knots[veh2.degree:-veh2.degree]), np.ones(degree)]
                        basis = BSplineBasis(knots, degree)
                        for kk, shape1 in enumerate(veh1.shapes):
                            for ll, shape2 in enumerate(veh2.shapes):
                                a = self.define_spline_variable(
                                    'a'+'_'+veh1.label+'_'+'seg'+str(idx)+'_'+str(kk)+'_'+veh2.label+'_'+str(ll), self.n_dim, basis=basis)
                                b = self.define_spline_variable(
                                    'b'+'_'+veh1.label+'_'+'seg'+str(idx)+'_'+str(kk)+'_'+veh2.label+'_'+str(ll), 1, basis=basis)[0]
                                self.define_constraint(
                                    sum([a[p]*a[p] for p in range(self.n_dim)])-1, -inf, 0.)
                                hyp_veh[veh1][shape1].append({'a': a, 'b': b})
                                hyp_veh[veh2][shape2].append({'a': [-a_i for a_i in a], 'b': -b})
            for vehicle in vehicles:
                splines = vehicle.splines[idx]
                vehicle.define_collision_constraints(hyp_veh[vehicle], self.room[idx], splines, horizon_times[idx])

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def init(self, horizon_times=None):
        for obstacle in self.obstacles:
            obstacle.init(horizon_times=horizon_times)

    # ========================================================================
    # Simulate environment
    # ========================================================================

    def simulate(self, simulation_time, sample_time):
        for obstacle in self.obstacles:
            # check if obstacle moves
            if (('trajectories' in obstacle.simulation) and
                ('velocity' in obstacle.simulation['trajectories']) and
                obstacle.options['bounce']):
                # select current velocity
                vel = obstacle.signals['velocity'][:,-1]
                # check if it overlaps with any other obstacle
                for obs in self.obstacles:
                    # don't check overlap with itself
                    if obs != obstacle:
                        if obstacle.overlaps_with(obs):
                            # bounce straight off other obstacle
                            if any(v == 0 for v in vel):
                                vel_new = -vel
                            # bounce diagonally off other obstacle
                            else:
                                old_pos = np.copy(obstacle.signals['position'][:,-1])
                                if (vel[0] > 0 and vel[1] > 0):
                                    # direction = up_right
                                    # new direction may be down_right or up_left
                                    # test new direction down_right by shifting obstacle
                                    obstacle.signals['position'][:,-1] += [0.15,-0.15]
                                    if not obstacle.overlaps_with(obs):
                                        # no overlap so move down_right
                                        # new_direction = down_right
                                        vel_new = [vel[0], -vel[1]]
                                    else:
                                        # there was overlap so move up_left
                                        # new_direction = up_left
                                        vel_new = [-vel[0],vel[1]]
                                elif vel[0] < 0 and vel[1] > 0:
                                    # direction = up_left
                                    # new direction may be down_left or up_right
                                    # test new direction down_left by shifting obstacle
                                    obstacle.signals['position'][:,-1] += [-0.15,-0.15]
                                    if not obstacle.overlaps_with(obs):
                                        # no overlap so move down_left
                                        # new_direction = down_left
                                        vel_new = [vel[0], -vel[1]]
                                    else:
                                        # there was overlap so move up_right
                                        # new_direction = up_right
                                        vel_new = [-vel[0],vel[1]]
                                elif vel[0] > 0 and vel[1] < 0:
                                    # direction = down_right
                                    # new direction may be down_left or up_right
                                    # test new direction down_left by shifting obstacle
                                    obstacle.signals['position'][:,-1] += [-0.15,-0.15]
                                    if not obstacle.overlaps_with(obs):
                                        # no overlap so move down_left
                                        # new_direction = down_left
                                        vel_new = [-vel[0], vel[1]]
                                    else:
                                        # there was overlap so move up_right
                                        # new_direction = up_right
                                        vel_new = [vel[0],-vel[1]]
                                elif vel[0] < 0 and vel[1] < 0:
                                    # direction = down_left
                                    # new direction may be down_right or up_left
                                    # test new direction down_right by shifting obstacle
                                    obstacle.signals['position'][:,-1] += [0.15,-0.15]
                                    if not obstacle.overlaps_with(obs):
                                        # no overlap so move down_right
                                        # new_direction = down_right
                                        vel_new = [-vel[0], vel[1]]
                                    else:
                                        # there was overlap so move up_left
                                        # new_direction = up_left
                                        vel_new = [vel[0],-vel[1]]

                                # reset position
                                obstacle.signals['position'][:,-1] = old_pos
                            obstacle.signals['velocity'][:,-1] = vel_new
                # check if the obstacle doesn't hit the borders
                # Todo: supposed that self.room[0] is the total room, containing outer border
                if obstacle.is_outside_of(self.room[0]):
                    # bounce straight off border
                    if any(v == 0 for v in vel):
                        vel_new = -vel
                    # bounce diagonally off border
                    else:
                        old_pos = np.copy(obstacle.signals['position'][:,-1])
                        if (vel[0] > 0 and vel[1] > 0):
                            # direction = up_right
                            # new direction may be down_right or up_left
                            # test new direction down_right by shifting obstacle
                            obstacle.signals['position'][:,-1] += [0.15,-0.15]
                            if not obstacle.is_outside_of(self.room[0]):
                                # no overlap so move down_right
                                # new_direction = down_right
                                vel_new = [vel[0], -vel[1]]
                            else:
                                # there was overlap so move up_left
                                # new_direction = up_left
                                vel_new = [-vel[0],vel[1]]
                        elif vel[0] < 0 and vel[1] > 0:
                            # direction = up_left
                            # new direction may be down_left or up_right
                            # test new direction down_left by shifting obstacle
                            obstacle.signals['position'][:,-1] += [-0.15,-0.15]
                            if not obstacle.is_outside_of(self.room[0]):
                                # no overlap so move down_left
                                # new_direction = down_left
                                vel_new = [vel[0], -vel[1]]
                            else:
                                # there was overlap so move up_right
                                # new_direction = up_right
                                vel_new = [-vel[0],vel[1]]
                        elif vel[0] > 0 and vel[1] < 0:
                            # direction = down_right
                            # new direction may be down_left or up_right
                            # test new direction down_left by shifting obstacle
                            obstacle.signals['position'][:,-1] += [-0.15,-0.15]
                            if not obstacle.is_outside_of(self.room[0]):
                                # no overlap so move down_left
                                # new_direction = down_left
                                vel_new = [-vel[0], vel[1]]
                            else:
                                # there was overlap so move up_right
                                # new_direction = up_right
                                vel_new = [vel[0],-vel[1]]
                        elif vel[0] < 0 and vel[1] < 0:
                            # direction = down_left
                            # new direction may be down_right or up_left
                            # test new direction down_right by shifting obstacle
                            obstacle.signals['position'][:,-1] += [0.15,-0.15]
                            if not obstacle.is_outside_of(self.room[0]):
                                # no overlap so move down_right
                                # new_direction = down_right
                                vel_new = [-vel[0], vel[1]]
                            else:
                                # there was overlap so move up_left
                                # new_direction = up_left
                                vel_new = [vel[0],-vel[1]]
                        # reset position
                        obstacle.signals['position'][:,-1] = old_pos
                    print 'setting new velocity'
                    obstacle.signals['velocity'][:,-1] = vel_new
            obstacle.simulate(simulation_time, sample_time)
        for zone in self.danger_zones:
            zone.simulate(simulation_time, sample_time)
        self.update_plots()

    def draw(self, t=-1):
        surfaces, lines = [], []
        for room in self.room:
            if room['draw']:
                s, l = room['shape'].draw(pose = np.r_[room['position'], room['orientation']])
                surfaces += s
                lines += l
        for obstacle in self.obstacles:
            s, l = obstacle.draw(t)
            if self.n_dim == 3 and obstacle.n_dim == 2:
                for k in range(len(s)):
                    s[k] = np.vstack((s[k], np.zeros((1, s[k].shape[1]))))
                for k in range(len(l)):
                    l[k] = np.vstack((l[k], np.zeros((1, l[k].shape[1]))))
            surfaces += s
            lines += l
        for zone in self.danger_zones:
            s, l = zone.draw(t)
            surfaces += s
            lines += l
        return surfaces, lines

    def get_canvas_limits(self):
        limits = []
        for room in self.room:
            lims = room['shape'].get_canvas_limits()
            limits += [[lims[k]+room['position'][k] for k in range(self.n_dim)]]
        return limits

    # ========================================================================
    # Plot related functions
    # ========================================================================

    def init_plot(self, argument, **kwargs):
        s, l = self.draw()
        gray = [60./255., 61./255., 64./255.]
        surfaces = [{'facecolor': mix_with_white(gray, 50), 'edgecolor': 'black', 'linewidth': 1.2} for _ in s]
        lines = [{'color': 'black', 'linewidth': 1.2} for _ in l]

        s_, l_ = [], []
        for room in self.room:
            if room['draw']:
                shape, line = room['shape'].draw()
                s_.append(shape)
                l_.append(line)
        for k, _ in enumerate(s_):
            surfaces[k]['facecolor'] = 'none'  # make room surface transparent

        for zone in self.danger_zones:
            if zone.options['draw']:
                shape, line = room['shape'].draw()
                s_.append(shape)
                l_.append(line)
        for k, _ in enumerate(s_):
            surfaces[k]['facecolor'] = 'none'

        if 'limits' in kwargs:
            limits = kwargs['limits']
        else:
            # find minimum and maximum x and y values over all rooms
            lims = np.r_[self.get_canvas_limits()]  # gives limits for all rooms
            values = [lims[:,k] for k in range(self.n_dim)]  # gather values over all dimensions
            limits = [[np.amin(values[k]), np.amax(values[k])] for k in range(self.n_dim)]
        labels = ['' for k in range(self.n_dim)]
        if self.n_dim == 2:
            return [[{'labels': labels,'surfaces': surfaces, 'lines': lines, 'aspect_equal': True,
                      'xlim': limits[0], 'ylim': limits[1]}]]
        else:
            return [[{'labels': labels, 'surfaces': surfaces, 'lines': lines, 'aspect_equal': True,
                      'xlim': limits[0], 'ylim': limits[1], 'zlim': limits[2],
                      'projection': '3d'}]]

    def update_plot(self, argument, t, **kwargs):
        s, l = self.draw(t)
        return [[{'surfaces': s, 'lines': l}]]
