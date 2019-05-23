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

from __future__ import print_function
from ..basics.shape import Rectangle, Circle
from ..basics.geometry import point_in_rectangle, distance_between_points
from ..basics.geometry import intersect_line_segments, intersect_lines

import numpy as np
import time


class Frame(object):
    def __init__(self, environment, start_pos, global_path, veh_size, margin, options={}):

        self.environment = environment # environment in which the frame is situated
        self.border = []  # border of the frame
        self.start_pos = start_pos  # initial position of vehicle in frame, or first waypoint of the frame
        self.global_path = global_path  # global path trough the environment, used to create a frame
        self.veh_size = veh_size  # vehicle size to take into account when making a frame
        self.margin = margin  # margin to take into account when making a frame
        self.stationary_obstacles = []  # stationary obstacles inside the frame
        self.moving_obstacles = []  # moving obstacles inside the frame
        self.waypoints = []  # waypoints of the global path inside the frame
        self.endpoint = []  # goal position within the frame
        self.options = options
        if not 'verbose' in options:
            self.options['verbose'] = 2  # default value
        # set sample time used to check if moving obstacle is inside the frame,
        # e.g. 0.5 then check if obstacle is inside frame on time [s] 0,0.5,1,...
        self.check_moving_obs_ts = options['check_moving_obs_ts'] if 'check_moving_obs_ts' in options else 0.5

    def get_stationary_obstacles(self, frame=None):
        if frame is None:
            frame = self
        obstacles = []
        xmin, ymin, xmax, ymax = frame.border['limits']
        shape = frame.border['shape']
        pos = np.array(frame.border['position'][:2])
        # Note: these checkpoints already include pos
        checkpoints = [[xmin, ymin],[xmin, ymax],[xmax, ymax],[xmax, ymin]]

        for obstacle in frame.environment.obstacles:
            # check if obstacle is stationary, this is when:
            # there is no entry trajectories or there are trajectories but no velocity or
            # all velocities are 0.
            if ((not 'trajectories' in obstacle.simulation) or (not 'velocity' in obstacle.simulation['trajectories'])
               or (all(vel == [0.]*obstacle.n_dim for vel in obstacle.simulation['trajectories']['velocity']['values']))):
                # we have a stationary obstacle, Circle or Rectangle
                # now check if frame intersects with the obstacle

                ###############################################
                ###### Option1: handle circle as circular #####
                ###############################################
                # if isinstance(obstacle.shape, Circle):
                #     if (point_in_polyhedron(obstacle.signals['position'][:,-1], shape_f, pos_f) or
                #        circle_polyhedron_intersection(obstacle, shape_f, pos_f)):
                #         obstacles_in_frame.append(obstacle)
                #         break
                # elif isinstance(obstacle.shape, Rectangle):
                #     if obstacle.shape.orientation == 0:
                #         # is frame vertex inside obstacle? Check rectangle overlap
                #         [[xmin_obs, xmax_obs],[ymin_obs, ymax_obs]] = obstacle.shape.get_canvas_limits()
                #         posx, posy = obstacle.signals['position'][:,-1]
                #         xmin_obs += posx
                #         xmax_obs += posx
                #         ymin_obs += posy
                #         ymax_obs += posy
                #         # based on: http://stackoverflow.com/questions/306316/determine-if-two-rectangles-overlap-each-other
                #         if (xmin_f <= xmax_obs and xmax_f >= xmin_obs and ymin_f <= ymax_obs and ymax_f >= ymin_obs):
                #                 obstacles_in_frame.append(obstacle)
                #                 break
                #     else:
                #         raise RuntimeError('Only rectangle with zero orientation\
                #                             are supported in multiframeproblem for now')
                # else:
                #     raise RuntimeError('Only Circle and Rectangle shaped obstacles\
                #                         are supported for now')

                #####################################################
                ###### Option2: approximate circle as as square #####
                #####################################################
                if ((isinstance(obstacle.shape, Rectangle) and obstacle.shape.orientation == 0) or
                    isinstance(obstacle.shape, Circle)):
                    # is frame vertex inside obstacle? check rectangle overlap
                    # if obstacle is Circle, it gets approximated by a square
                    [[xmin_obs, xmax_obs],[ymin_obs, ymax_obs]] = obstacle.shape.get_canvas_limits()
                    posx, posy = obstacle.signals['position'][:,-1]
                    xmin_obs += posx
                    xmax_obs += posx
                    ymin_obs += posy
                    ymax_obs += posy
                    # based on: http://stackoverflow.com/questions/306316/determine-if-two-rectangles-overlap-each-other
                    if (xmin <= xmax_obs and xmax >= xmin_obs and ymin <= ymax_obs and ymax >= ymin_obs):
                            obstacles.append(obstacle)
                            # don't break, add all obstacles
                else:
                    raise RuntimeError('Only Circle and Rectangle shaped obstacles\
                                        with orientation 0 are supported for now')
        return obstacles

    def get_moving_obstacles(self, motion_time):
        # determine which moving obstacles are in frame for motion_time

        start_time = time.time()

        moving_obstacles = []
        for obstacle in self.environment.obstacles:
            # check if obstacle is moving, this is when:
            # not all velocities, saved in signals, are 0
            if not all(obstacle.signals['velocity'][:,-1] == [0.]*obstacle.n_dim):
                # get obstacle checkpoints
                if not isinstance(obstacle.shape, Circle):
                    # element [0] gives vertices, not the corresponding radii
                    obs_chck = obstacle.shape.get_checkpoints()[0]
                else:
                    # for a circle only the center is returned as a checkpoint
                    # make a square representation of it and use those checkpoints

                    [[xmin, xmax],[ymin, ymax]] = obstacle.shape.get_canvas_limits()
                    obs_chck = [[xmin, ymin], [xmin, ymax], [xmax, ymax], [xmax, ymin]]
                    # also check center, because vertices of square approximation may be outside
                    # of the frame, while the center is in the frame
                    obs_chck.insert(0, [0,0])
                obs_pos = obstacle.signals['position'][:,-1]
                obs_vel = obstacle.signals['velocity'][:,-1]

                for chck in obs_chck:
                    # if it is not a circle, rotate the vertices
                    if hasattr(obstacle.shape, 'orientation'):
                        vertex = obstacle.shape.rotate(obstacle.shape.orientation, chck)
                    else:
                        vertex = chck
                    # move to correct position
                    vertex += obs_pos
                    # check if vertex is in frame during movement
                    if self.point_in_frame(vertex, time=motion_time,
                                           sample_time=self.check_moving_obs_ts, velocity=obs_vel):
                        # avoid corresponding obstacle
                        obstacle.set_options({'avoid': True})
                        moving_obstacles.append(obstacle)
                        # break from for chck in obs_chck, move on to next obstacle, since
                        # obstacle is added to the frame if any of its vertices is in the frame
                        break

        end_time = time.time()
        if self.options['verbose'] >= 3:
            print('elapsed time in get_moving_obstacles', end_time-start_time)

        return moving_obstacles

    def make_border(self, xmin, ymin, xmax, ymax):
        width = xmax - xmin
        height = ymax - ymin
        angle = 0.
        # add angle to position
        position = [(xmax - xmin)*0.5+xmin,(ymax-ymin)*0.5+ymin, angle]
        limits = [xmin, ymin, xmax, ymax]

        return {'shape': Rectangle(width=width, height=height),
         'position': position, 'orientation': angle, 'limits': limits}

    def distance_to_border(self, point):
        # returns the x- and y-direction distance from point to the border of frame
        # based on: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        # when point is outside of the border, the function also returns the distance to the border,
        # and you can use point_in_rectangle() to check if point is inside the border or not

        x2, y2, x3, y3 = self.border['limits']
        # number vertices of border
        # v2--v3
        # |   |
        # v1--v4
        v1 = [x2,y2]
        v2 = [x2,y3]
        v3 = [x3,y3]
        v4 = [x3,y2]

        # dist from v1-v2
        # Note the minus sign! A negative shift in x-direction is required to lower the distance
        dist12 = -abs((v2[1]-v1[1])*point[0] - (v2[0]-v1[0])*point[1] + v2[0]*v1[1] - v2[1]*v1[0])/(np.sqrt((v2[1]-v1[1])**2+(v2[0]-v1[0])**2))
        #dist from v2-v3
        dist23 = abs((v3[1]-v2[1])*point[0] - (v3[0]-v2[0])*point[1] + v3[0]*v2[1] - v3[1]*v2[0])/(np.sqrt((v3[1]-v2[1])**2+(v3[0]-v2[0])**2))
        #dist from v3-v4
        dist34 = abs((v4[1]-v3[1])*point[0] - (v4[0]-v3[0])*point[1] + v4[0]*v3[1] - v4[1]*v3[0])/(np.sqrt((v4[1]-v3[1])**2+(v4[0]-v3[0])**2))
        #dist from v4-v1
        # Note the minus sign! A negative shift in y-direction is required to lower the distance
        dist41 = -abs((v1[1]-v4[1])*point[0] - (v1[0]-v4[0])*point[1] + v1[0]*v4[1] - v1[1]*v4[0])/(np.sqrt((v1[1]-v4[1])**2+(v1[0]-v4[0])**2))

        distance = [0.,0.]
        distance[0] = dist12 if abs(dist12) < abs(dist34) else dist34  # x-direction: from point to side12 or side34
        distance[1] = dist23 if abs(dist23) < abs(dist41) else dist41  # y-direction: from point to side23 or side41

        return distance

    def make_last_waypoint_reachable(self, method='move_frame'):
        # after scaling up or moving the frame, the last waypoint can still be too close
        # to the frame border, i.e. the vehicle cannot reach it without colliding
        # two solutions:
        #   method1: shift frame (now stationary obstacles can end up inside the frame for corridor frame)
        #   method2: shift last waypoint inwards
        # afterwards update limits of frame

        xmin,ymin,xmax,ymax = self.border['limits']
        # is last waypoint inside the border?
        inside_border = point_in_rectangle(self.waypoints[-1], self.border['limits'])
        # compute distance to border
        dist_to_border = self.distance_to_border(self.waypoints[-1])
        if method == 'move_frame':  # move frame borders, keep waypoint
            if abs(dist_to_border[0]) <= self.veh_size:
                if self.options['verbose'] >= 2:
                    print('Last waypoint too close in x-direction, moving frame')
                # move in x-direction
                if not inside_border:
                    move_distance = abs(dist_to_border[0]) + self.veh_size*self.margin
                else:
                    move_distance = -abs(dist_to_border[0]) + self.veh_size*self.margin
                if dist_to_border[0] <= 0:
                    xmin = xmin - move_distance
                else:
                    xmax = xmax + move_distance
                self.border = self.make_border(xmin, ymin, xmax, ymax)
            if abs(dist_to_border[1]) <= self.veh_size:
                if self.options['verbose'] >= 2:
                    print('Last waypoint too close in y-direction, moving frame')
                # move in y-direction
                if not inside_border:
                    move_distance = abs(dist_to_border[1]) + self.veh_size*self.margin
                else:
                    move_distance = -abs(dist_to_border[1]) + self.veh_size*self.margin
                if dist_to_border[1] <= 0:
                    ymin = ymin - move_distance
                else:
                    ymax = ymax + move_distance
                self.border = self.make_border(xmin, ymin, xmax, ymax)
        elif method == 'move_point':  # move waypoint, keep frame borders
            new_waypoint = None
            # compute distance from last waypoint to border
            if (not inside_border or any(abs(d) <= self.veh_size*self.margin for d in dist_to_border)):
                # waypoint was outside of border, or too close to border
                count = 1
                while True:  # find waypoint that is far enough from border
                    inside_border = point_in_rectangle(self.waypoints[-1-count], self.border['limits'])
                    dist_to_border = self.distance_to_border(self.waypoints[-1-count])
                    if (not inside_border or any(abs(d) <= self.veh_size for d in dist_to_border)):
                        count += 1  # this waypoint was also outside of, or too close to border
                    else:  # found waypoint that is far enough from border
                        break
                # check if waypoint is inside rectangle in x- and/or y-direction
                inside_border = point_in_rectangle(self.waypoints[-1], self.border['limits'], xy_check=True)
                # recompute distance from last waypoint to border
                dist_to_border = self.distance_to_border(self.waypoints[-1])
                # set desired distance from waypoint to border
                desired_distance = 2*[self.veh_size*self.margin]

                # use desired distance to move waypoint to a reachable position
                x1, y1 = self.waypoints[-1-count]  # reachable waypoint inside frame
                x2, y2 = self.waypoints[-1]  # unreachable waypoint inside or outside frame
                if (not inside_border[0] or abs(dist_to_border[0]) < self.veh_size):
                    # to avoid floating point comparison problems, check distance without margin here
                    # problem lies in the x-direction
                    new_waypoint = [0, 0]
                    if dist_to_border[0]<=0:
                        new_waypoint[0] = xmin + desired_distance[0]
                    else:
                        new_waypoint[0] = xmax - desired_distance[0]
                    if (y1 == y2):
                        new_waypoint[1] = y1
                    else:
                        # use equation of line to compute y-coordinate
                        new_waypoint[1] = (new_waypoint[0]-x1)*(float((y2-y1))/(x2-x1))+y1

                    # check if new_waypoint is reachable
                    inside_border = point_in_rectangle(new_waypoint, self.border['limits'], xy_check=True)
                    # compute distance from new waypoint to border
                    dist_to_border = self.distance_to_border(new_waypoint)
                # x-direction was fixed above, now re-check only for the y-direction
                if (not inside_border[1] or abs(dist_to_border[1]) < self.veh_size):
                    # to avoid floating point comparison problems, check distance without margin here
                    # problem lies in the y-direction
                    new_waypoint = [0, 0]
                    if dist_to_border[1]<=0:
                        new_waypoint[1] = ymin + desired_distance[1]
                    else:
                        new_waypoint[1] = ymax - desired_distance[1]
                    if (x1 == x2):
                        new_waypoint[0] = x1
                    else:
                        # use equation of line to compute x-coordinate
                        new_waypoint[0] = (new_waypoint[1]-y1)*(float((x2-x1))/(y2-y1))+x1
                if new_waypoint is None:
                    # last waypoint was reachable when not taking into account the margin, so use this one
                    new_waypoint = self.waypoints[-1]
                # remove the last count waypoints from the old frame,
                # and change them by the newly computed (reachable) waypoint
                for i in range(count):
                    self.waypoints.pop()  # remove last count waypoints
                self.waypoints.append(new_waypoint)  # add new waypoint
        else:
            raise ValueError('Method should be move_frame or move_point, not: ', method)

    def point_in_frame(self, point, sample_time=None, time=None, velocity=None, distance=0):

        # check if the provided point is inside frame
        # both for stationary or moving points
        # distance is the margin to take into account
        xmin, ymin, xmax, ymax = self.border['limits']

        # check stationary point
        if time is None:
            if (xmin+distance < point[0] < xmax-distance) and (ymin+distance < point[1] < ymax-distance):
                return True
            else:
                return False
        # check moving point
        elif (isinstance(time, (float, int)) and sample_time is not None):
            # amount of times to check
            N = int(round(time/sample_time)+1)
            x, y = point
            vx, vy = velocity
            for l in range(N+1):
                if xmin <= (x+l*sample_time*vx) <= xmax and ymin <= (y+l*sample_time*vy) <= ymax:
                    return True
            return False
        elif sample_time is None:
            raise RuntimeError('No sample time provided')
        else:
            raise RuntimeError('Argument time was of the wrong type, not None, float or int')

    def find_closest_waypoint(self, position=None, waypoints=None):
        # if no specific arguments provided, use default arguments
        if position is None:
            position = self.start_pos
        if waypoints is None:
            waypoints = self.global_path

        # only consider the waypoints past 'position' = the current vehicle position or a certain waypoint
        # so find the waypoint that is closest to position
        dist = max(self.environment.room[0]['shape'].width, self.environment.room[0]['shape'].height)
        closest_waypoint = waypoints[0]
        index = 0
        for idx, point in enumerate(waypoints):
            d = distance_between_points(point, position)
            if d < dist:
                dist = d
                closest_waypoint = point
                index = idx
        return closest_waypoint, index

class ShiftFrame(Frame):

    def __init__(self, environment, start_pos, size, move_limit, global_path, veh_size, margin, options={}):

        Frame.__init__(self, environment, start_pos, global_path, veh_size, margin, options)
        self.type = 'shift'
        self.frame_size = size
        self.move_limit = move_limit

        # frame with current vehicle position as center
        xmin = self.start_pos[0] - self.frame_size*0.5
        ymin = self.start_pos[1] - self.frame_size*0.5
        xmax = self.start_pos[0] + self.frame_size*0.5
        ymax = self.start_pos[1] + self.frame_size*0.5
        self.border = self.make_border(xmin,ymin,xmax,ymax)

        # determine next waypoint outside frame so we can
        # change the position of the frame if needed
        waypoint = None  # holds waypoint outside the frame
        endpoint = None  # holds the goal point if it is inside the frame
        points_in_frame = []  # holds all waypoints in the frame

        # start with waypoint that is closest to start_pos
        _, start_idx = self.find_closest_waypoint()
        for idx, point in enumerate(self.global_path[start_idx:]):
            if not self.point_in_frame(point):
                # is point also out of frame when moving the frame towards the waypoint?
                # if so, we move the window over a distance of 'move_limit' extra, and we have found
                # the first waypoint outside the shifted frame

                # determine distance between waypoint outside of frame and current state
                delta_x = point[0] - self.start_pos[0]
                delta_y = point[1] - self.start_pos[1]
                if (abs(delta_x) > self.move_limit+self.frame_size*0.5 or
                    abs(delta_y) > self.move_limit+self.frame_size*0.5):
                    # waypoint outside frame, even after shifting
                    waypoint = point
                    break
                # point is last point, and no points outside frame were found yet
                elif point == self.global_path[-1]:
                    endpoint = point
                else:
                    # waypoint inside frame after shifting
                    points_in_frame.append(point)
            else:
                # waypoint inside frame
                points_in_frame.append(point)

        # optimize frame position based on next waypoint (='waypoint')
        if waypoint is not None:
            # found waypoint outside frame, which is not even inside the frame after shifting
            # over move_limit: shift frame in the direction of this point and check if point is
            # far enough from border
            xmin, ymin, xmax, ymax = self.move_frame(delta_x, delta_y)
            self.border = self.make_border(xmin, ymin, xmax, ymax)

            # move waypoint to a reachable position
            self.waypoints = points_in_frame  # assign waypoints, used inside next function
            # append last waypoint, that is not reachable for the moment
            self.waypoints.append(waypoint)
            # move the waypoint, such that it is reachable within the frame
            self.make_last_waypoint_reachable(method='move_point')
            points_in_frame = self.waypoints  # update points_in_frame
            self.endpoint = points_in_frame[-1]  # assign end point

            # # make line between last reachable waypoint inside frame and first waypoint outside frame,
            # waypoint_line = [points_in_frame[-1], waypoint]
            # # find intersection point between line and frame
            # intersection_point = self.find_intersection_line_segment_frame(frame, waypoint_line)
            # # move endpoint away from border
            # endpoint = self.move_from_border(points_in_frame[-1], intersection_point, frame,
            #                                                          distance=self.veh_size*self.margin)
        elif self.endpoint is not None:
            # vehicle goal is inside frame after shifting
            # shift frame over calculated distance
            xmin, ymin, xmax, ymax = self.move_frame(delta_x, delta_y)
            self.border = self.make_border(xmin, ymin, xmax, ymax)

            # assign waypoints to frame, is used in next function
            self.waypoints = points_in_frame
            # append endpoint, that may not be reachable for the moment (too close to border)
            self.waypoints.append(endpoint)
            # if vehicle goal is too close to the frame border, move the frame extra in that direction
            self.make_last_waypoint_reachable(method='move_frame')
        else:
            # all waypoints are within the frame, even without shifting, so don't shift the frame
            self.endpoint = self.global_path[-1]

        # finish frame description
        # frame['border'] is already determined
        self.stationary_obstacles = self.get_stationary_obstacles()
        if self.options['verbose'] >= 2:
            print('Stationary obstacles inside new frame: ', self.stationary_obstacles)
            print('first waypoint in new frame: ', self.waypoints[0])
            print('last waypoint in new frame:', self.endpoint)
        # If generated frame contains goal position, endpoint will be = goal position, since the goal position
        # was added to the global path. This was necessary because otherwise you will end up on a grid point
        # and not necessarily in the goal position (which can lie between grid points, anywhere on the map)
        self.waypoints.append(endpoint)

    def move_frame(self, delta_x, delta_y, start_pos=None):
        # if no start position is given, default position is used
        if start_pos is None:
            start_pos = self.start_pos

        # determine direction we have to move in
        newx_lower = newx_upper = self.frame_size*0.5
        newy_lower = newy_upper = self.frame_size*0.5

        # while moving, take into account veh_size
        # waypoint is outside frame in x-direction, shift in the waypoint direction
        if abs(delta_x) > self.frame_size*0.5:
            # move frame in x-direction, over a max of move_limit
            move = min(self.move_limit, abs(delta_x)-self.frame_size*0.5 + self.veh_size*self.margin)
            if delta_x > 0:
                newx_lower = self.frame_size*0.5 - move
                newx_upper = self.frame_size*0.5 + move
            else:
                newx_lower = self.frame_size*0.5 + move
                newx_upper = self.frame_size*0.5 - move

        # waypoint is outside frame in y-direction, shift in the waypoint direction
        if abs(delta_y) > self.frame_size*0.5:
            # move frame in y-direction, over a max of move_limit
            move = min(self.move_limit, abs(delta_y)-self.frame_size*0.5 + self.veh_size*self.margin)
            if delta_y > 0:
                newy_lower = self.frame_size*0.5 - move
                newy_upper = self.frame_size*0.5 + move
            else:
                newy_lower = self.frame_size*0.5 + move
                newy_upper = self.frame_size*0.5 - move

        # move from origin to start position (that may be off-center, after shifting)
        newx_min = start_pos[0] - newx_lower
        newy_min = start_pos[1] - newy_lower
        newx_max = start_pos[0] + newx_upper
        newy_max = start_pos[1] + newy_upper

        # make sure new limits are not outside the main environment room
        # if so, shrink the frame to fit inside the environment
        environment_limits = self.environment.get_canvas_limits()  #[[xmin,xmax],[ymin,ymax]]
        environment_limits = environment_limits[0]  # select first room, the main one
        if newx_min < environment_limits[0][0]:
            newx_min = environment_limits[0][0]
        if newx_max > environment_limits[0][1]:
            newx_max = environment_limits[0][1]
        if newy_min < environment_limits[1][0]:
            newy_min = environment_limits[1][0]
        if newy_max > environment_limits[1][1]:
            newy_max = environment_limits[1][1]

        return newx_min, newy_min, newx_max, newy_max


class CorridorFrame(Frame):

    def __init__(self, environment, start_pos, global_path, veh_size, margin, options={}):

        Frame.__init__(self, environment, start_pos, global_path, veh_size, margin, options)
        self.type = 'corridor'  # default value
        self.scale_up_fine = options['scale_up_fine']

        self.create_corridor_base_frame()

        # scale up frame in all directions until it hits the borders or an obstacle
        self.scale_up_frame()

        # possibly the last waypoint is not reachable by the vehicle, fix this
        self.make_last_waypoint_reachable(method='move_point')

        # finish frame description
        # frame['border'] is already determined
        self.stationary_obstacles = self.get_stationary_obstacles()
        if self.options['verbose'] >= 2:
            print('Stationary obstacles inside new frame: ', self.stationary_obstacles)
            print('first waypoint in new frame: ', self.waypoints[0])
            print('last waypoint in new frame:', self.waypoints[-1])
        # If generated frame contains goal position, endpoint will be = goal position, since the goal position
        # was added to the global path. This was necessary because otherwise you will end up on a grid point
        # and not necessarily in the goal position (which can lie between grid points, anywhere on the map)
        self.endpoint = self.waypoints[-1]

    def create_corridor_base_frame(self):
        # generates a minimal area frame without any stationary obstacles inside
        # this is done by subsequently including more waypoints, until an obstacle
        # is found inside the frame
        # this gives a frame with width or height of the vehicle size,
        # later this frame is scaled up

        start_time = time.time()

        # frame with current vehicle position as center,
        # width and height are determined by the vehicle size
        xmin = self.start_pos[0] - self.veh_size*self.margin
        ymin = self.start_pos[1] - self.veh_size*self.margin
        xmax = self.start_pos[0] + self.veh_size*self.margin
        ymax = self.start_pos[1] + self.veh_size*self.margin
        self.border = self.make_border(xmin,ymin,xmax,ymax)

        _, index = self.find_closest_waypoint()
        points_in_frame = []  # holds all waypoints in the frame

        # run over all waypoints, starting from the waypoint closest to start_pos
        # first try if endpoint can be inside the frame without obstacles
        point = self.global_path[-1]
        if not self.point_in_frame(point):
            # next waypoint is not inside frame,
            # enlarge frame such that it is in there
            # determine which borders to move

            # make frame with first point = start_pos and next point = goal
            prev_point = self.start_pos

            self.update_frame_with_waypoint(prev_point, point)
            # check if there are any obstacles inside this new frame
            self.stationary_obstacles = self.get_stationary_obstacles()

            if self.stationary_obstacles:
                # there is an obstacle inside the frame after enlarging
                # don't add point and keep old frame
                self.border = self.make_border(xmin,ymin,xmax,ymax)
                # current frame cannot contain the endpoint
                for idx, point in enumerate(self.global_path[index:]):
                    # update limits of frame
                    xmin,ymin,xmax,ymax = self.border['limits']

                    if not self.point_in_frame(point):
                        # next waypoint is not inside frame,
                        # enlarge frame such that it is in there
                        # determine which borders to move
                        if points_in_frame:
                            # assign last point in frame
                            prev_point = points_in_frame[-1]
                        else:
                            # no points in frame yet, so compare with current state
                            prev_point = self.start_pos

                        self.update_frame_with_waypoint(prev_point, point)
                        # check if there are any obstacles inside this new frame
                        self.stationary_obstacles = self.get_stationary_obstacles()

                        if self.stationary_obstacles:
                            # there is an obstacle inside the frame after enlarging
                            # don't add point and keep old frame
                            self.border = self.make_border(xmin,ymin,xmax,ymax)
                            # frame is finished
                            break
                        else:
                            points_in_frame.append(point)
                    else:
                        # next waypoint is inside frame, without enlarging it
                        # so add it anyway
                        points_in_frame.append(point)
            else:
                # frame with first waypoint and goal position contained no obstacles
                # directly using this frame
                points_in_frame = self.global_path[index:]
        else:
            # all the remaining waypoints are in the current frame
            points_in_frame.extend(self.global_path[index:])
            # make endpoint_frame equal to the goal (point = self.global_path[-1] here)
            self.endpoint = point
        if not points_in_frame:
            raise RuntimeError('No waypoint was found inside corridor frame, something wrong with frame')
        else:
            self.waypoints = points_in_frame

        end_time = time.time()
        print('time in CorridorFrame', end_time-start_time)

    def update_frame_with_waypoint(self, prev_point, point):
        # updates the frame when adding a new waypoint to the frame
        # compare point with prev_point
        xmin,ymin,xmax,ymax = self.border['limits']
        xmin_new,ymin_new,xmax_new,ymax_new = xmin,ymin,xmax,ymax
        if point[0] > prev_point[0]:
            xmax_new = point[0] + self.veh_size*self.margin
        elif point[0] < prev_point[0]:
            xmin_new = point[0] - self.veh_size*self.margin
        # else: xmin and xmax are kept
        if point[1] > prev_point[1]:
            ymax_new = point[1] + self.veh_size*self.margin
        elif point[1] < prev_point[1]:
            ymin_new = point[1] - self.veh_size*self.margin
            ymax_new = ymax
        # else: ymin and ymax are kept

        self.border = self.make_border(xmin_new,ymin_new,xmax_new,ymax_new)

    def scale_up_frame(self):
        # scale up the current frame in all directions, until it hits the borders
        # or it contains an obstacle

        # Note: when incrementally making the frame larger, we update with a certain size.
        # Updating with self.veh_size*self.margin may be too big, such that frames are not as
        # wide/large as they can be. Changing e.g. to xmax_new = xmax + 0.1, is more accurate,
        # but this takes more time to compute.
        # the selection is set by the self.scale_up_fine boolean

        start_time = time.time()

        xmin,ymin,xmax,ymax = self.border['limits']

        # enlarge in positive x-direction
        # first try to put xmax = frame border
        xmax_new = self.environment.room[0]['shape'].get_canvas_limits()[0][1] + self.environment.room[0]['position'][0]
        self.border = self.make_border(xmin,ymin,xmax_new,ymax)
        if not self.get_stationary_obstacles():
            xmax = xmax_new  # assign new xmax
        else:
            while True:
                    if self.scale_up_fine:
                        xmax_new = xmax + 0.1
                    else:
                        xmax_new = xmax + self.veh_size*self.margin
                    self.border = self.make_border(xmin,ymin,xmax_new,ymax)
                    if xmax_new > self.environment.room[0]['shape'].get_canvas_limits()[0][1] + self.environment.room[0]['position'][0]:
                        # the frame hit the borders, this is the maximum size in this direction
                        break
                    if not self.get_stationary_obstacles():
                        # there is no obstacle in the enlarged frame, so enlarge it
                        xmax = xmax_new
                    else:
                        # there is an obstacle in the enlarged frame, don't enlarge it
                        self.border = self.make_border(xmin,ymin,xmax,ymax)
                        break

        # enlarge in negative x-direction
        # first try to put xmin = frame border
        xmin_new = self.environment.room[0]['shape'].get_canvas_limits()[0][0] + self.environment.room[0]['position'][0]
        self.border = self.make_border(xmin_new,ymin,xmax,ymax)
        if not self.get_stationary_obstacles():
            xmin = xmin_new  # assign new xmin
        else:
            while True:
                if self.scale_up_fine:
                    xmin_new = xmin - 0.1
                else:
                    xmin_new = xmin - self.veh_size*self.margin
                self.border = self.make_border(xmin_new,ymin,xmax,ymax)
                if xmin_new < self.environment.room[0]['shape'].get_canvas_limits()[0][0] + self.environment.room[0]['position'][0]:
                    # the frame hit the borders, this is the maximum size in this direction
                    break
                if not self.get_stationary_obstacles():
                    xmin = xmin_new
                else:
                    self.border = self.make_border(xmin,ymin,xmax,ymax)
                    break

        # enlarge in positive y-direction
        # first try to put ymax = frame border
        ymax_new = self.environment.room[0]['shape'].get_canvas_limits()[1][1] + self.environment.room[0]['position'][1]
        self.border = self.make_border(xmin,ymin,xmax,ymax_new)
        if not self.get_stationary_obstacles():
            ymax = ymax_new  # assign new ymax
        else:
            while True:
                if self.scale_up_fine:
                    ymax_new = ymax + 0.1
                else:
                    ymax_new = ymax + self.veh_size*self.margin
                self.border = self.make_border(xmin,ymin,xmax,ymax_new)
                if ymax_new > self.environment.room[0]['shape'].get_canvas_limits()[1][1] + self.environment.room[0]['position'][1]:
                    # the frame hit the borders, this is the maximum size in this direction
                    break
                if not self.get_stationary_obstacles():
                    ymax = ymax_new
                else:
                    self.border = self.make_border(xmin,ymin,xmax,ymax)
                    break

        # enlarge in negative y-direction
        # first try to put ymin = frame border
        ymin_new = self.environment.room[0]['shape'].get_canvas_limits()[1][0] + self.environment.room[0]['position'][1]
        self.border = self.make_border(xmin,ymin_new,xmax,ymax)
        if not self.get_stationary_obstacles():
            ymin = ymin_new  # assign new ymin
        else:
            while True:
                if self.scale_up_fine:
                    ymin_new = ymin - 0.1
                else:
                    ymin_new = ymin - self.veh_size*self.margin
                self.border = self.make_border(xmin,ymin_new,xmax,ymax)
                if ymin_new < self.environment.room[0]['shape'].get_canvas_limits()[1][0] + self.environment.room[0]['position'][1]:
                    # the frame hit the borders, this is the maximum size in this direction
                    break
                if not self.get_stationary_obstacles():
                    ymin = ymin_new
                else:
                    self.border = self.make_border(xmin,ymin,xmax,ymax)
                    break

        # update waypoints
        # starting from the last waypoint that was already in the frame
        # vehicle size was already taken into account above, when shifting borders
        index = self.global_path.index(self.waypoints[-1])
        for idx, point in enumerate(self.global_path[index:]):
            if self.point_in_frame(point):
                if not point in self.waypoints:
                    # point was not yet a waypoint of the frame,
                    # but it is inside the scaled frame
                    self.waypoints.append(point)
            else:
                # waypoint was not inside the scaled_frame, stop looking
                break

        end_time = time.time()
        if self.options['verbose'] >= 2:
            print('time in scale_up_frame', end_time-start_time)

    def create_l_shape(self, next_frame):
        frame1 = self  # current frame
        frame2 = next_frame

        # check for shift in the first direction: horizontal or vertical

        # connect beginning of current and next frame
        line = [frame1.waypoints[0], frame2.waypoints[0]]
        # compute intersection point between this line segment and the next frame
        intersection_point = self.find_intersection_line_segment_frame(frame2, line)
        if intersection_point is not None:
            xmin1,ymin1,xmax1,ymax1 = frame1.border['limits']
            xmin2,ymin2,xmax2,ymax2 = frame2.border['limits']
            # check if intersection point is on the border of frame2,
            # if so, there is also a part outside of the line segment that intersects
            # this gives a new limit of frame1: for this particular limit,
            # the values for frame1 and frame2 coincide
            if abs(intersection_point[0] - xmin2) <= 1e-4:
                frame1.border = self.make_border(xmin1,ymin1,xmax2,ymax1)
                if (not frame1.point_in_frame(self.start_pos) or
                    self.get_stationary_obstacles(frame=frame1)):
                    # start position is not inside the frame anymore,
                    # or there is a stationary obstacle in the new frame,
                    # meaning that the shift of the boundary was invalid:
                    # restore old frame
                    frame1.border = self.make_border(xmin1,ymin1,xmax1,ymax1)
                else:
                    # update limits
                    xmin1,ymin1,xmax1,ymax1 = frame1.border['limits']
            if abs(intersection_point[0] - xmax2) <= 1e-4:
                frame1.border = self.make_border(xmin2,ymin1,xmax1,ymax1)
                if (not frame1.point_in_frame(self.start_pos) or
                    self.get_stationary_obstacles(frame=frame1)):
                    # restore old frame
                    frame1.border = self.make_border(xmin1,ymin1,xmax1,ymax1)
                else:
                    # update limits
                    xmin1,ymin1,xmax1,ymax1 = frame1.border['limits']
            if abs(intersection_point[1] - ymax2) <= 1e-4:
                frame1.border = self.make_border(xmin1,ymin2,xmax1,ymax1)
                if (not frame1.point_in_frame(self.start_pos) or
                    self.get_stationary_obstacles(frame=frame1)):
                    # restore old frame
                    frame1.border = self.make_border(xmin1,ymin1,xmax1,ymax1)
                else:
                    # update limits
                    xmin1,ymin1,xmax1,ymax1 = frame1.border['limits']
            if abs(intersection_point[1] - ymin2) <= 1e-4:
                frame1.border = self.make_border(xmin1,ymin1,xmax1,ymax2)
                if (not frame1.point_in_frame(self.start_pos) or
                    self.get_stationary_obstacles(frame=frame1)):
                    # restore old frame
                    frame1.border = self.make_border(xmin1,ymin1,xmax1,ymax1)
                else:
                    # update limits
                    xmin1,ymin1,xmax1,ymax1 = frame1.border['limits']

        # check for shift in the second direction: vertical or horizontal

        # connect first and last waypoint of next frame
        line = [frame2.waypoints[0], frame2.waypoints[-1]]
        # compute intersection with first frame
        intersection_point = self.find_intersection_line_segment_frame(frame1, line)
        if intersection_point is not None:
            xmin1,ymin1,xmax1,ymax1 = frame1.border['limits']
            xmin2,ymin2,xmax2,ymax2 = frame2.border['limits']
            # check if intersection point is on the border of frame1,
            # if so, there is also a part outside of the line segment that intersects
            # this gives a new limit of frame2: for this particular limit,
            # the values for frame2 and frame1 coincide
            if abs(intersection_point[0] - xmin1) <= 1e-4:
                frame2.border = self.make_border(xmin2,ymin2,xmax1,ymax2)
                if (not frame2.point_in_frame(frame2.waypoints[-1]) or
                    self.get_stationary_obstacles(frame=frame2)):
                    # last waypoint is not inside the frame2 anymore,
                    # or there is a stationary obstacle in the new frame2,
                    # meaning that the shift of the boundary was invalid:
                    # restore old frame
                    frame2.border = self.make_border(xmin2,ymin2,xmax2,ymax2)
            if abs(intersection_point[0] - xmax1) <= 1e-4:
                frame2.border = self.make_border(xmin1,ymin2,xmax2,ymax2)
                if (not frame2.point_in_frame(frame2.waypoints[-1]) or
                    self.get_stationary_obstacles(frame=frame2)):
                    # restore old frame
                    frame2.border = self.make_border(xmin2,ymin2,xmax2,ymax2)
            if abs(intersection_point[1] - ymax1) <= 1e-4:
                frame2.border = self.make_border(xmin2,ymin1,xmax2,ymax2)
                if (not frame2.point_in_frame(frame2.waypoints[-1]) or
                    self.get_stationary_obstacles(frame=frame2)):
                    # restore old frame
                    frame2.border = self.make_border(xmin2,ymin2,xmax2,ymax2)
            if abs(intersection_point[1] - ymin1) <= 1e-4:
                frame2.border = self.make_border(xmin2,ymin2,xmax2,ymax1)
                if (not frame2.point_in_frame(frame2.waypoints[-1]) or
                    self.get_stationary_obstacles(frame=frame2)):
                    # restore old frame
                    frame2.border = self.make_border(xmin2,ymin2,xmax2,ymax2)
        # return (updated) frames
        return frame1, frame2

    def find_intersection_line_segment_frame(self, frame, line):
        # find intersection point of the provided line with frame
        x3, y3, x4, y4 = frame.border['limits']

        # frame border representation:
        # [x3,y4]---------[x4,y4]
        #    |               |
        #    |               |
        #    |               |
        # [x3,y3]---------[x4,y3]

        # move over border in clockwise direction:
        top_side    = [[x3,y4],[x4,y4]]
        right_side  = [[x4,y4],[x4,y3]]
        bottom_side = [[x4,y3],[x3,y3]]
        left_side   = [[x3,y3],[x3,y4]]

        # First find which line segments intersect, afterwards use a method
        # for line intersection to find the intersection point. Not possible
        # to use intersect_lines immediately since it doesn't take into account
        # the segments, but considers infinitely long lines.

        #intersection with top side?
        if intersect_line_segments(line, top_side):
            # find intersection point
            intersection_point = intersect_lines(line, top_side)
        #intersection with right side?
        elif intersect_line_segments(line, right_side):
            # find intersection point
            intersection_point = intersect_lines(line, right_side)
        #intersection with bottom side?
        elif intersect_line_segments(line, bottom_side):
            # find intersection point
            intersection_point = intersect_lines(line, bottom_side)
        #intersection with left side?
        elif intersect_line_segments(line, left_side):
            # find intersection point
            intersection_point = intersect_lines(line, left_side)
        else:
            intersection_point = None
        return intersection_point

