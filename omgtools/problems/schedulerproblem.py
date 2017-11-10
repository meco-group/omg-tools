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

from problem import Problem
from multiframeproblem import MultiFrameProblem
from point2point import Point2point
from ..basics.shape import Rectangle, Circle
from ..basics.geometry import distance_between_points, intersect_lines, intersect_line_segments
from ..basics.geometry import point_in_rectangle
from ..basics.spline import BSplineBasis
from ..basics.spline_extra import concat_splines
from ..environment.environment import Environment
from globalplanner import AStarPlanner

from scipy.interpolate import interp1d
import scipy.linalg as la
import numpy as np
import time
import warnings


class SchedulerProblem(Problem):

    def __init__(self, fleet, environment, global_planner=None, options=None, **kwargs):
        Problem.__init__(self, fleet, environment, options, label='schedulerproblem')
        self.curr_state = self.vehicles[0].prediction['state'] # initial vehicle position
        self.goal_state = self.vehicles[0].poseT # overall goal
        self.problem_options = options  # e.g. selection of problem type (freeT, fixedT)
        if not 'freeT' in options:
            # select default type
            self.problem_options['freeT'] = True
        self.start_time = 0.
        self.update_times=[]
        self.cnt = 1  # frame counter
        self.n_frames = kwargs['n_frames'] if 'n_frames' in kwargs else 1  # amount of frames to combine
        # sample time used to check if moving obstacle is inside the frame,
        # e.g. 0.5 then check if obstacle is inside frame on time 0,0.5,1,...
        self.check_moving_obs_ts = options['check_moving_obs_ts'] if 'check_moving_obs_ts' in options else 0.5

        if (self.n_frames > 1 and not self.problem_options['freeT']):
            raise ValueError('Fixed time problems are only supported for n_frames = 1')
        self._n_frames = self.n_frames  # save original value

        # save vehicle dimension, determines how close waypoints can be to the border
        shape = self.vehicles[0].shapes[0]
        if isinstance(shape, Circle):
            self.veh_size = shape.radius
            # used to check if vehicle fits completely in a cell
            # radius is only half the vehicle size
            size_to_check = self.veh_size*2
        elif isinstance(shape, Rectangle):
            self.veh_size = max(shape.width, shape.height)
            # veh_size is complete width or height for rectangular shape
            size_to_check = self.veh_size
        # margin, applied to vehicle size to avoid goal positions very close to the border
        self.margin = 1.1

        # assign global planner
        if global_planner is not None:
            # save the provided planner
            self.global_planner = global_planner
        else:
            # make a default global planner
            self.global_planner = AStarPlanner(environment, [20, 20], self.curr_state, self.goal_state)

        # frame settings
        self.frames = []
        self.frame_type = kwargs['frame_type'] if 'frame_type' in kwargs else 'shift'
        # set frame size for frame_type shift
        if self.frame_type is 'shift':
            self.frame_size = kwargs['frame_size'] if 'frame_size' in kwargs else environment.room[0]['shape'].width*0.2

        if self.frame_type is 'min_nobs':
            self.scale_up_fine = options['scale_up_fine'] if 'scale_up_fine' in options else True
        # check if vehicle size is larger than the cell size
        n_cells = self.global_planner.grid.n_cells
        if (size_to_check >= (min(environment.room[0]['shape'].width/float(n_cells[0]), \
                                  environment.room[0]['shape'].height/float(n_cells[1])))
           and self.frame_type == 'min_nobs'):
            warnings.warn('Vehicle is bigger than one cell, this may cause problems' +
                          ' when switching frames. Consider reducing the amount of cells or reducing' +
                          ' the size of the vehicle')

    def init(self):
        # otherwise the init of Problem is called, which is not desirable
        pass

    def initialize(self, current_time):
        self.local_problem.initialize(current_time)

    def reinitialize(self):
        # this function is called at the start and creates the first frame

        self.global_path = self.global_planner.get_path()
        # plot grid and global path
        self.global_planner.grid.draw()
        self.global_planner.plot_path(self.global_path)
        # append goal state to waypoints of global path,
        # since desired goal is not necessarily a waypoint
        # remove orientation info, since this is not relevant for the global path
        self.global_path.append(self.goal_state[:2])

        # fill in self.frames, according to self.n_frames
        self.create_frames()

        # get initial guess (based on global path), get motion time, for all frames
        init_guess, self.motion_times = self.get_init_guess()

        # get moving obstacles inside frame, taking into account the calculated motion time
        for k in range(self.n_frames):
            # updates self.frames[:]['moving_obstacles']
            self.frames[k]['moving_obstacles'] = self.get_moving_obstacles_in_frame(self.frames[k], self.motion_times[k])

        # get a problem representation of the frames
        # the schedulerproblem (self) has a local problem (multiframeproblem) at each moment
        self.local_problem = self.generate_problem()
        # pass on initial guess
        self.local_problem.reset_init_guess(init_guess)

    def solve(self, current_time, update_time):
        # solve the local problem with a receding horizon,
        # and update frames if necessary

        # update current state
        if not hasattr(self.vehicles[0], 'signals'):
            # first iteration
            self.curr_state = self.vehicles[0].prediction['state']
        else:
            # all other iterations
            self.curr_state = self.vehicles[0].signals['state'][:,-1]

        frames_valid = self.check_frames()
        if not frames_valid:
            self.cnt += 1  # count frame

            # frames were not valid anymore, update based on current position

            # if n_frames=1, we need self.next_frame
            if (self.n_frames == 1 and
                hasattr(self, 'next_frame')):
                self.update_frames(next_frame=self.next_frame)
            else:
                self.update_frames()

            # transform frames into local_problem and simulate
            self.local_problem = self.generate_problem()
            # update_frames() updates self.motion_times and self.init_guess
            self.local_problem.reset_init_guess(self.init_guess)
        else:
            # update remaining motion time
            if self.problem_options['freeT']:
                # freeT: there is a time variable
                for k in range(self.n_frames):
                    self.motion_times[k] = self.local_problem.father.get_variables(
                                           self.local_problem, 'T'+str(k),)[0][0]
            else:
                # fixedT: the remaining motion time is always the horizon time
                # for fixedT, self.n_frames is always = 1
                self.motion_times = [self.local_problem.options['horizon_time']]

            # check if amount of moving obstacles changed
            # if the obstacles just change their trajectory, the simulator takes this into account
            # if moving_obstacles is different from the existing moving_obstacles (new obstacle,
            # or disappeared obstacle) make a new problem
            new_problem = False
            for k in range(self.n_frames):
                moving_obs_in_frame = self.get_moving_obstacles_in_frame(self.frames[k], self.motion_times[k])
                if set(moving_obs_in_frame) != set(self.frames[k]['moving_obstacles']):
                    # the amount of moving obstacles changed, or there is a new obstacle
                    new_problem = True
                    self.frames[k]['moving_obstacles'] = moving_obs_in_frame
                else:
                    new_problem = False
            if new_problem:
                # new moving obstacle in one of the frames or obstacle disappeared from frame
                # np.array converts DM to array
                # save previous solution
                init_guess = []
                for k in range(self.n_frames):
                    init_guess.append(np.array(
                                      self.local_problem.father.get_variables()[self.vehicles[0].label, 'splines_seg'+str(k)]))
                # make a new local problem, using self.frames
                self.local_problem = self.generate_problem()
                # use solution from previous problem as guess
                self.local_problem.reset_init_guess(init_guess)

        # solve local problem
        self.local_problem.solve(current_time, update_time)

        # save solving time
        self.update_times.append(self.local_problem.update_times[-1])

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def store(self, current_time, update_time, sample_time):
        # call store of local problem
        self.local_problem.store(current_time, update_time, sample_time)

    def simulate(self, current_time, simulation_time, sample_time):
        # save global path and frame border
        # store trajectories
        if not hasattr(self, 'frame_storage'):
            self.frame_storage = []
            self.global_path_storage = []
        if simulation_time == np.inf:
            # using simulator.run_once()
            simulation_time = sum(self.motion_times)
        repeat = int(simulation_time/sample_time)
        # copy frames, to avoid problems when removing elements from self.frames
        frames_to_save = self.frames[:]
        for k in range(repeat):
            self._add_to_memory(self.frame_storage, frames_to_save)
        self._add_to_memory(self.global_path_storage, self.global_path, repeat)

        # simulate the multiframe problem
        Problem.simulate(self, current_time, simulation_time, sample_time)

    def _add_to_memory(self, memory, data_to_add, repeat=1):
        memory.extend([data_to_add for k in range(repeat)])

    def stop_criterium(self, current_time, update_time):
        # check if the current frame is the last one
        if self.frames[-1]['endpoint_frame'] == self.goal_state[:2]:  # remove orientation info
            # if we now reach the goal, the vehicle has arrived
            if self.local_problem.stop_criterium(current_time, update_time):
                return True
        else:
            return False

    def final(self):
        print 'The robot has reached its goal!'
        print 'The problem was divided over ', self.cnt,' frames'
        if self.options['verbose'] >= 1:
            print '%-18s %6g ms' % ('Max update time:',
                                    max(self.update_times)*1000.)
            print '%-18s %6g ms' % ('Av update time:',
                                    (sum(self.update_times)*1000. /
                                     len(self.update_times)))

    # ========================================================================
    # Export related functions
    # ========================================================================

    def export(self, options=None):
        raise NotImplementedError('Please implement this method!')

    # ========================================================================
    # Plot related functions
    # ========================================================================

    def init_plot(self, argument, **kwargs):
        # initialize environment plot
        info = Problem.init_plot(self, argument)
        gray = [60./255., 61./255., 64./255.]
        if info is not None:
            for k in range(self._n_frames):
                # initialize frame plot, always use frames[0]
                s, l = self.frames[0]['border']['shape'].draw(self.frames[0]['border']['position'])
                surfaces = [{'facecolor': 'none', 'edgecolor': gray, 'linestyle' : '--', 'linewidth': 1.2} for _ in s]
                info[0][0]['surfaces'] += surfaces
                # initialize global path plot
                info[0][0]['lines'] += [{'color': 'red', 'linestyle' : '--', 'linewidth': 1.2}]
        return info

    def update_plot(self, argument, t, **kwargs):
        # plot environment
        data = Problem.update_plot(self, argument, t)
        if data is not None:
            for k in range(len(self.frame_storage[t])):
                # for every frame at this point in time
                # plot frame border
                s, l = self.frame_storage[t][k]['border']['shape'].draw(self.frame_storage[t][k]['border']['position'])
                data[0][0]['surfaces'] += s
            # plot global path
            # remove last waypoint, since this is the goal position,
            # which was manually added and is not necessarily a grid point
            length = np.shape(self.global_path_storage[t][:-1])[0]
            waypoints = np.array(np.zeros((2,length)))
            for idx, waypoint in enumerate(self.global_path_storage[t][:-1]):
                waypoints[0,idx] = waypoint[0]
                waypoints[1,idx] = waypoint[1]
            data[0][0]['lines'] += [waypoints]
        return data

    # ========================================================================
    # MultiFrameProblem specific functions
    # ========================================================================

    def create_frames(self):
        # makes frames, based on the environment,
        # the current state and the global path (waypoints)

        # there are two different options: shift and min_nobs
        # min_nobs: change frame size such that the frame is as large as possible, without
        # containing any stationary obstacles
        # shift: keep frame_size fixed, shift frame in the direction of the movement
        # over a maximum distance of move_limit

        start_time = time.time()

        if self.frames:
            # already computed frames before, end up here because frame[0] was not valid anymore
            # shift all frames forward over one frame, i.e. remove self.frames[0]
            # but only if there is more than one frame
            if self.n_frames != 1:
                self.frames.pop(0)
            # compute new frame, using the last frame in the current list
            frame = self.frames[-1]
            # add one extra frame
            n_frames_to_create = 1
        else:
            n_frames_to_create = self.n_frames
            frame = {}

        # frame of fixed size
        for k in range(n_frames_to_create):
            # check if previously created frame was last one
            if frame and frame['waypoints'][-1] == self.goal_state[:2]:  # remove orientation info
                # reduce amount of frames that are considered and stop loop
                self.n_frames -= 1
                break
            # set start position for computation of frame that we will add
            if frame:
                start_pos = frame['waypoints'][-1]
            else:
                # there were no frames yet, so compute self.n_frames, starting from current state
                # remove orientation from state (if using e.g. differential drive)
                start_pos = self.curr_state[:2]
            if self.frame_type == 'shift':
                frame = self.create_frame_shift(start_pos)
            elif self.frame_type == 'min_nobs':
                frame = self.create_frame_min_nobs(start_pos)
            else:
                raise RuntimeError('Invalid frame type: ', self.frame_type)
            # append new frame to the frame list
            self.frames.append(frame)

            if self.n_frames == 1:
                # then we need a next_frame to create a region of overlap, determining when to switch frames
                self.next_frame = self.create_next_frame(frame)
        end_time = time.time()
        if self.options['verbose'] >= 2:
            print 'elapsed time while creating new ' + self.frame_type + ' frame: ', end_time-start_time

    def create_frame_shift(self, start_pos):
        frame = {}

        # frame with current vehicle position as center
        xmin = start_pos[0] - self.frame_size*0.5
        ymin = start_pos[1] - self.frame_size*0.5
        xmax = start_pos[0] + self.frame_size*0.5
        ymax = start_pos[1] + self.frame_size*0.5
        frame['border'] = self.make_border(xmin,ymin,xmax,ymax)

        # determine next waypoint outside frame so we can
        # change the position of the frame if needed
        waypoint = None  # holds waypoint outside the frame
        endpoint = None  # holds the goal point if it is inside the frame
        points_in_frame = []  # holds all waypoints in the frame

        # start with waypoint that is closest to start_pos
        _, start_idx = self.find_closest_waypoint(start_pos, self.global_path)
        for idx, point in enumerate(self.global_path[start_idx:]):
            if not self.point_in_frame(frame, point):
                # is point also out of frame when moving the frame towards the waypoint?
                # if so, we move the window over a distance of 'move_limit' extra, and we have found
                # the first waypoint outside the shifted frame

                # determine distance between waypoint outside of frame and current state
                delta_x = point[0] - start_pos[0]
                delta_y = point[1] - start_pos[1]
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
            xmin, ymin, xmax, ymax = self.move_frame(start_pos, delta_x, delta_y)
            frame['border'] = self.make_border(xmin, ymin, xmax, ymax)

            # move waypoint to a reachable position
            frame['waypoints'] = points_in_frame  # assign waypoints, used inside next function
            # append last waypoint, that is not reachable for the moment
            frame['waypoints'].append(waypoint)
            # move the waypoint, such that it is reachable within the frame
            frame = self.make_last_waypoint_reachable(frame, method='move_point')
            points_in_frame = frame['waypoints']  # update points_in_frame
            endpoint = points_in_frame[-1]  # assign end point

            # # make line between last reachable waypoint inside frame and first waypoint outside frame,
            # waypoint_line = [points_in_frame[-1], waypoint]
            # # find intersection point between line and frame
            # intersection_point = self.find_intersection_line_segment_frame(frame, waypoint_line)
            # # move endpoint away from border
            # endpoint = self.move_from_border(points_in_frame[-1], intersection_point, frame,
            #                                                          distance=self.veh_size*self.margin)
        elif endpoint is not None:
            # vehicle goal is inside frame after shifting
            # shift frame over calculated distance
            xmin, ymin, xmax, ymax = self.move_frame(start_pos, delta_x, delta_y)
            frame['border'] = self.make_border(xmin, ymin, xmax, ymax)

            # assign waypoints to frame, is used in next function
            frame['waypoints'] = points_in_frame
            # append endpoint, that may not be reachable for the moment (too close to border)
            frame['waypoints'].append(endpoint)
            # if vehicle goal is too close to the frame border, move the frame extra in that direction
            frame = self.make_last_waypoint_reachable(frame, method='move_frame')
        else:
            # all waypoints are within the frame, even without shifting, so don't shift the frame
            endpoint = self.global_path[-1]

        # check if last waypoint is too close to the frame border, move the frame extra in that direction
        dist_to_border = self.distance_to_border(frame, endpoint)
        if abs(dist_to_border[0]) <= self.veh_size:
            if self.options['verbose'] >= 2:
                print 'Last waypoint too close in x-direction, moving frame'
            # move in x-direction
            move_distance = (self.veh_size - abs(dist_to_border[0]))*self.scale_factor
            if dist_to_border[0]<=0:
                xmin = xmin - move_distance
            else:
                xmax = xmax + move_distance
            frame['border'] = self.make_border(xmin, ymin, xmax, ymax)
        if abs(dist_to_border[1]) <= self.veh_size:
            if self.options['verbose'] >= 2:
                print 'Last waypoint too close in y-direction, moving frame'
            # move in y-direction
            move_distance = (self.veh_size - abs(dist_to_border[1]))*self.scale_factor
            if dist_to_border[1]<=0:
                ymin = ymin - move_distance
            else:
                ymax = ymax + move_distance
            frame['border'] = self.make_border(xmin, ymin, xmax, ymax)

        # finish frame description
        # frame['border'] is already determined
        stationary_obstacles = self.get_stationary_obstacles_in_frame(frame)
        if self.options['verbose'] >= 2:
            print 'Stationary obstacles inside new frame: ', stationary_obstacles
            print 'first waypoint in new frame: ', points_in_frame[0]
            print 'last waypoint in new frame:', endpoint
        frame['stationary_obstacles'] = stationary_obstacles
        # If generated frame contains goal position, endpoint will be = goal position, since the goal position
        # was added to the global path. This was necessary because otherwise you will end up on a grid point
        # and not necessarily in the goal position (which can lie between grid points, anywhere on the map)
        frame['endpoint_frame'] = endpoint
        points_in_frame.append(endpoint)
        frame['waypoints'] = points_in_frame

        return frame

    def create_frame_min_nobs(self, start_pos):
        # create small frame around vehicle, including as many waypoints
        # as possible, without containing any stationary obstacles
        frame = self.create_min_nobs_base_frame(start_pos)

        # scale up frame in all directions until it hits the borders or an obstacle
        frame['border'], frame['waypoints'] = self.scale_up_frame(frame)

        # possibly the last waypoint is not reachable by the vehicle, fix this
        frame = self.make_last_waypoint_reachable(frame, method='move_point')

        # finish frame description
        # frame['border'] is already determined
        stationary_obstacles = self.get_stationary_obstacles_in_frame(frame)
        if self.options['verbose'] >= 2:
            print 'Stationary obstacles inside new frame: ', stationary_obstacles
            print 'first waypoint in new frame: ', frame['waypoints'][0]
            print 'last waypoint in new frame:', frame['waypoints'][-1]
        frame['stationary_obstacles'] = stationary_obstacles
        # If generated frame contains goal position, endpoint will be = goal position, since the goal position
        # was added to the global path. This was necessary because otherwise you will end up on a grid point
        # and not necessarily in the goal position (which can lie between grid points, anywhere on the map)
        frame['endpoint_frame'] = frame['waypoints'][-1]

        return frame

    def make_last_waypoint_reachable(self, frame, method='move_frame'):
        # after scaling up or moving the frame, the last waypoint can still be too close
        # to the frame border, i.e. the vehicle cannot reach it without colliding
        # two solutions:
        #   method1: shift frame (now stationary obstacles can end up inside the frame for min_nobs)
        #   method2: shift last waypoint inwards
        # afterwards update limits of frame

        xmin,ymin,xmax,ymax = frame['border']['limits']
        # is last waypoint inside the border?
        inside_border = point_in_rectangle(frame['border']['limits'], frame['waypoints'][-1])
        # compute distance to border
        dist_to_border = self.distance_to_border(frame, frame['waypoints'][-1])
        if method == 'move_frame':  # move frame borders, keep waypoint
            if abs(dist_to_border[0]) <= self.veh_size:
                if self.options['verbose'] >= 2:
                    print 'Last waypoint too close in x-direction, moving frame'
                # move in x-direction
                if not inside_border:
                    move_distance = abs(dist_to_border[0]) + self.veh_size*self.margin
                else:
                    move_distance = -abs(dist_to_border[0]) + self.veh_size*self.margin
                if dist_to_border[0] <= 0:
                    xmin = xmin - move_distance
                else:
                    xmax = xmax + move_distance
                frame['border'] = self.make_border(xmin, ymin, xmax, ymax)
            if abs(dist_to_border[1]) <= self.veh_size:
                if self.options['verbose'] >= 2:
                    print 'Last waypoint too close in y-direction, moving frame'
                # move in y-direction
                if not inside_border:
                    move_distance = abs(dist_to_border[1]) + self.veh_size*self.margin
                else:
                    move_distance = -abs(dist_to_border[1]) + self.veh_size*self.margin
                if dist_to_border[1] <= 0:
                    ymin = ymin - move_distance
                else:
                    ymax = ymax + move_distance
                frame['border'] = self.make_border(xmin, ymin, xmax, ymax)
        elif method == 'move_point':  # move waypoint, keep frame borders
            # compute distance from last waypoint to border
            if (not inside_border or any(abs(d) <= self.veh_size*self.margin for d in dist_to_border)):
                # waypoint was outisde of border, or too close to border
                count = 1
                while True:  # find waypoint that is far enough from border
                    inside_border = point_in_rectangle(frame['border']['limits'], frame['waypoints'][-1-count])
                    dist_to_border = self.distance_to_border(frame, frame['waypoints'][-1-count])
                    if (not inside_border or any(abs(d) <= self.veh_size for d in dist_to_border)):
                        count += 1  # this waypoint was also outside of, or too close to border
                    else:  # found waypoint that is far enough from border
                        break
                # check if waypoint is inside rectangle in x- and/or y-direction
                inside_border = point_in_rectangle(frame['border']['limits'], frame['waypoints'][-1], xy_check=True)
                # recompute distance from last waypoint to border
                dist_to_border = self.distance_to_border(frame, frame['waypoints'][-1])

                # compute desired distance from waypoint to border, if outside border, add dist_to_border
                desired_distance = [0.,0.]
                if inside_border[0]:
                    # point is inside the border in the x-direction
                    desired_distance[0] = self.veh_size*self.margin  # desired distance from last waypoint to border
                else:
                    # point is outside the border in the x-direction
                    desired_distance[0] = self.veh_size*self.margin + abs(dist_to_border[0])
                if inside_border[1]:
                    # point is inside the border in the y-direction
                    desired_distance[1] = self.veh_size*self.margin  # desired distance from last waypoint to border
                else:
                    # point is outside the border in the y-direction
                    desired_distance[1] = self.veh_size*self.margin + abs(dist_to_border[1])

                # use desired distance to move waypoint to a reachable position
                x1, y1 = frame['waypoints'][-1-count]  # reachable waypoint inside frame
                x2, y2 = frame['waypoints'][-1]  # unreachable waypoint inside or outside frame
                if (not inside_border[0] or abs(dist_to_border[0]) <= self.veh_size*self.margin):
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
                    inside_border = point_in_rectangle(frame['border']['limits'], new_waypoint, xy_check=True)
                    # compute distance from new waypoint to border
                    dist_to_border = self.distance_to_border(frame, new_waypoint)
                    # re-compute the desired distance for the y-direction
                    desired_distance = [0.,0.]
                    if inside_border[1]:
                        desired_distance[1] = self.veh_size*self.margin  # desired distance from last waypoint to border
                    else:
                        desired_distance[1] = self.veh_size*self.margin + abs(dist_to_border[1])
                # x-direction was fixed above, now re-check only for the y-direction
                if (not inside_border[1] or abs(dist_to_border[1]) <= self.veh_size*self.margin):
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
                # remove the last count waypoints from the old frame,
                # and change them by the newly computed (reachable) waypoint
                for i in range(count):
                    frame['waypoints'].pop()  # remove last count waypoints
                frame['waypoints'].append(new_waypoint)  # add new waypoint
        else:
            raise ValueError('Method should be move_frame or move_point, not: ', method)
        return frame

    def check_frames(self):
        # if final goal is not in the current frame, compare current distance
        # to the local goal with the initial distance
        if not self.point_in_frame(self.frames[0], self.goal_state[:2]):
            if (self.n_frames == 1 and hasattr(self, 'next_frame')):
                if self.point_in_frame(self.next_frame, self.curr_state[:2], distance=self.veh_size):
                    # only called if self.n_frames = 1,
                    # then self.frames[1] doesn't exist and self.next_frame does exist
                    # current state is inside the next frame, so switch
                    valid = False
                else:
                    valid = True
                return valid
            elif self.point_in_frame(self.frames[1], self.curr_state[:2], distance=self.veh_size):
                # vehicle is in overlap region between current and next frame
                valid = False
                return valid
            else:
                valid = True
                return valid
        else:  # keep frame, since you arrived at last frame
            valid = True
            return valid

    def update_frames(self, next_frame=None):
        # update global path from current position,
        # since there may be a deviation from original global path
        # and since you moved over the path so a part needs to be removed

        start_time = time.time()

        self.global_path = self.global_planner.get_path(start=self.curr_state, goal=self.goal_state)
        self.global_path.append(self.goal_state[:2])  # append goal state to path, remove orientation info

        # make new frame
        if next_frame is not None:
            # only possible if self.n_frames=1

            # we already have the next frame, so we first compute the frame
            # after the next one
            new_frame = self.create_next_frame(next_frame)
            # the next frame becomes the current frame
            self.frames[0] = self.next_frame.copy()
            # new_frame becomes the next frame
            # Note: if the current frame is the last one, new_frame will be None
            if new_frame is not None:
                self.next_frame = new_frame.copy()
            else:
                self.next_frame = None
        else:
            self.create_frames()

        # use previous solution to get an initial guess for all frames except the last one,
        # for this one get initial guess based on global path
        # analogously for the motion_times
        self.init_guess, self.motion_times = self.get_init_guess()

        # get moving obstacles inside frame for this time
        for k in range(self.n_frames):
            self.frames[k]['moving_obstacles'] = self.get_moving_obstacles_in_frame(self.frames[k], self.motion_times[k])

        end_time = time.time()
        if self.options['verbose'] >= 2:
            print 'elapsed time while updating frame: ', end_time-start_time

    def get_stationary_obstacles_in_frame(self, frame):
        obstacles_in_frame = []
        xmin_f, ymin_f, xmax_f, ymax_f = frame['border']['limits']
        shape_f = frame['border']['shape']
        pos_f = np.array(frame['border']['position'][:2])
        # Note: these checkpoints already include pos_f
        frame_checkpoints = [[xmin_f, ymin_f],[xmin_f, ymax_f],[xmax_f, ymax_f],[xmax_f, ymin_f]]
        for obstacle in self.environment.obstacles:
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
                    if (xmin_f <= xmax_obs and xmax_f >= xmin_obs and ymin_f <= ymax_obs and ymax_f >= ymin_obs):
                            obstacles_in_frame.append(obstacle)
                            # don't break, add all obstacles
                else:
                    raise RuntimeError('Only Circle and Rectangle shaped obstacles\
                                        with orientation 0 are supported for now')
        return obstacles_in_frame

    def get_moving_obstacles_in_frame(self, frame, motion_time):
        # determine which moving obstacles are in frame for motion_time

        start_time = time.time()

        moving_obs_in_frame = []
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
                    if self.point_in_frame(frame, vertex, time=motion_time, velocity=obs_vel):
                        # avoid corresponding obstacle
                        obstacle.set_options({'avoid': True})
                        moving_obs_in_frame.append(obstacle)
                        # break from for chck in obs_chck, move on to next obstacle, since
                        # obstacle is added to the frame if any of its vertices is in the frame
                        break

        end_time = time.time()
        if self.options['verbose'] >= 3:
            print 'elapsed time in get_moving_obstacles_in_frame', end_time-start_time

        return moving_obs_in_frame

    def get_init_guess(self, **kwargs):
        # local import to avoid circular dependency with Problem
        from ..vehicles.dubins import Dubins
        from ..vehicles.holonomic import Holonomic
        # if first iteration, compute init_guess based on global_path for all frames
        # else, use previous solutions to build a new initial guess:
        #   if 2 frames: combine splines in frame 1 and 2 to form a new spline in a single frame = new frame1
        #   if 3 frames or more: combine frame1 and 2 and keep splines of frame 3 and next as new splines of
        #        frame2 and next
        # only use global path for initial guess of new frame
        start_time = time.time()

        # initialize variables to hold guesses
        init_splines = []
        motion_times = []

        if hasattr(self, 'local_problem') and hasattr(self.local_problem.father, '_var_result'):
            # local_problem was already solved, re-use previous solutions to form initial guess
            if self.n_frames > 1:
                # combine first two spline segments into a new spline = guess for new current frame
                init_spl, motion_time = self.get_init_guess_combined_frame()
                init_splines.append(init_spl)
                motion_times.append(motion_time)
            if self.n_frames > 2:
                # use old solutions for frame 2 until second last frame, these don't change
                for k in range(2, self.n_frames):
                    init_splines.append(self.local_problem.father.get_variables(self.vehicles[0].label,'splines_seg'+str(k)))
                    motion_times.append(self.local_problem.father.get_variables(self.local_problem, 'T'+str(k),)[0][0])
            # only make guess using global path for last frame
            guess_idx = [self.n_frames-1]
        else:
            # local_problem was not solved yet, make guess using global path for all frames
            guess_idx = range(self.n_frames)

        # make guesses based on global path
        for k in guess_idx:
            init_spl, motion_time = self.get_init_guess_new_frame(self.frames[k])
            init_splines.append(init_spl)
            motion_times.append(motion_time)
        # pass on initial guess
        self.vehicles[0].set_init_spline_values(init_splines, n_seg = self.n_frames)

        # set start and goal
        if hasattr (self.vehicles[0], 'signals'):
            # use current vehicle velocity as starting velocity for next frame
            self.vehicles[0].set_initial_conditions(self.curr_state, input=self.vehicles[0].signals['input'][:,-1])
        else:
            self.vehicles[0].set_initial_conditions(self.curr_state)
        # if Dubins vehicle, add orientation 0
        if isinstance(self.vehicles[0], Dubins):
            if self.frames[-1]['waypoints'][-1] == self.goal_state[:2]:
                self.vehicles[0].set_terminal_conditions(self.goal_state)  # setting goal for final frame
            else:
                if hasattr(self, 'next_frame'):
                    # use last waypoint from current and first waypoint
                    # from next frame to compute desired orientation
                    # take second-last waypoint, since last one was moved in make_reachable()
                    x1,y1 = self.frames[-1]['waypoints'][-2]
                    # take second waypoint, since first will be == second-last of previous frame
                    x2,y2 = self.next_frame['waypoints'][1]
                    # compute angle
                    angle = np.arctan2((y2-y1),(x2-x1))
                else:
                    # compute angle between last and second last waypoint inside frame
                    # to use as a desired orientation
                    x1,y1 = self.frames[-1]['waypoints'][-2]
                    x2,y2 = self.frames[-1]['waypoints'][-1]
                    angle = np.arctan2((y2-y1),(x2-x1))
                # desired pose = last waypoint, with compute angle
                pose = self.frames[-1]['waypoints'][-1] + [angle]
                self.vehicles[0].set_terminal_conditions(pose)
        elif isinstance(self.vehicles[0], Holonomic):
            self.vehicles[0].set_terminal_conditions(self.frames[-1]['waypoints'][-1])
        else:
            raise RuntimeError('You selected an unsupported vehicle type, choose Holonomic or Dubins')

        end_time = time.time()
        if self.options['verbose'] >= 2:
            print 'elapsed time in get_init_guess ', end_time - start_time

        return init_splines, motion_times

    def get_init_guess_new_frame(self, frame):
        # local import to avoid circular dependency with Problem
        from ..vehicles.dubins import Dubins
        from ..vehicles.holonomic import Holonomic
        # generate initial guess for new frame, based on the waypoints of
        # the global path that are inside the frame

        waypoints = frame['waypoints']
        if frame is self.frames[0]:
            # current frame: change first waypoint to current state,
            # the startpoint of the init guess
            waypoints[0] = [self.curr_state[0], self.curr_state[1]]

        # use waypoints for prediction
        x, y = [], []
        for waypoint in waypoints:
            x.append(waypoint[0])
            y.append(waypoint[1])
        # calculate total length in x- and y-direction
        l_x, l_y = 0., 0.
        for i in range(len(waypoints)-1):
            l_x += waypoints[i+1][0] - waypoints[i][0]
            l_y += waypoints[i+1][1] - waypoints[i][1]

        # suppose vehicle is moving at half of vmax to calculate motion time,
        # instead of building and solving the problem
        length_to_travel = np.sqrt((l_x**2+l_y**2))
        max_vel = self.vehicles[0].vmax if hasattr(self.vehicles[0], 'vmax') else (self.vehicles[0].vxmax+self.vehicles[0].vymax)*0.5
        motion_time = length_to_travel/(max_vel*0.5)

        if isinstance(self.vehicles[0], Dubins):
            # initialize splines as zeros, since due to the change of variables it is not possible
            # to easily generate a meaningfull initial guess for r and v_tilde
            coeffs = 0*self.vehicles[0].knots[self.vehicles[0].degree-1:-(self.vehicles[0].degree-1)]
            init_guess = np.c_[coeffs, coeffs]
        elif isinstance(self.vehicles[0], Holonomic):
            # calculate distance in x and y between each 2 waypoints
            # and use it as a relative measure to build time vector
            time_x = [0.]
            time_y = [0.]

            for i in range(len(waypoints)-1):
                if (l_x == 0. and l_y !=0.):
                    time_x.append(0.)
                    time_y.append(time_y[-1] + float(waypoints[i+1][1] - waypoints[i][1])/l_y)
                elif (l_x != 0. and l_y == 0.):
                    time_x.append(time_x[-1] + float(waypoints[i+1][0] - waypoints[i][0])/l_x)
                    time_y.append(0.)
                elif (l_x == 0. and l_y == 0.):
                    time_x.append(0.)
                    time_y.append(0.)
                else:
                    time_x.append(time_x[-1] + float(waypoints[i+1][0] - waypoints[i][0])/l_x)
                    time_y.append(time_y[-1] + float(waypoints[i+1][1] - waypoints[i][1])/l_y)  # gives time 0...1

            # make approximate one an exact one
            # otherwise fx(1) = 1
            for idx, t in enumerate(time_x):
                if (1 - t < 1e-5):
                    time_x[idx] = 1
            for idx, t in enumerate(time_y):
                if (1 - t < 1e-5):
                    time_y[idx] = 1

            # make interpolation functions
            if (all( t == 0 for t in time_x) and all(t == 0 for t in time_y)):
                # motion_times.append(0.1)
                # coeffs_x = x[0]*np.ones(len(self.vehicles[0].knots[self.vehicles[0].degree-1:-(self.vehicles[0].degree-1)]))
                # coeffs_y = y[0]*np.ones(len(self.vehicles[0].knots[self.vehicles[0].degree-1:-(self.vehicles[0].degree-1)]))
                # init_splines.append(np.c_[coeffs_x, coeffs_y])
                # break
                raise RuntimeError('Trying to make a prediction for goal = current position. This may' +
                    ' be because vehicle is larger than one cell, this may cause problems when switching frames.'
                    ' Consider reducing the amount of cells, or scale your vehicle.')
            elif all(t == 0 for t in time_x):
                # if you don't do this, f evaluates to NaN for f(0)
                time_x = time_y
            elif all(t == 0 for t in time_y):
                # if you don't do this, f evaluates to NaN for f(0)
                time_y = time_x
            # kind='cubic' requires a minimum of 4 waypoints
            fx = interp1d(time_x, x, kind='linear', bounds_error=False, fill_value=1.)
            fy = interp1d(time_y, y, kind='linear', bounds_error=False, fill_value=1.)

            # evaluate resulting splines to get evaluations at knots = coeffs-guess
            # Note: conservatism is neglected here (spline value = coeff value)
            coeffs_x = fx(self.vehicles[0].basis.greville())
            coeffs_y = fy(self.vehicles[0].basis.greville())
            init_guess = np.c_[coeffs_x, coeffs_y]

            init_guess[-2] = init_guess[-1]  # final velocity and acceleration = 0
            init_guess[-3] = init_guess[-1]
        else:
            raise RuntimeError('You selected an unsupported vehicle type, choose Holonomic or Dubins')

        return init_guess, motion_time

    def get_init_guess_combined_frame(self):
        # combines the splines in the first two frames into a new one, forming the guess
        # for the new current frame

        # remaining spline through current frame
        spl1 = self.local_problem.father.get_variables(self.vehicles[0], 'splines_seg0')
        # frame through next frame
        spl2 = self.local_problem.father.get_variables(self.vehicles[0], 'splines_seg1')

        time1 = self.local_problem.father.get_variables(self.local_problem, 'T0',)[0][0]
        time2 = self.local_problem.father.get_variables(self.local_problem, 'T1',)[0][0]
        motion_time = time1 + time2  # guess for motion time

        # form connection of spl1 and spl2, in union basis
        spl = concat_splines([spl1, spl2], [time1, time2])

        # now find spline in original basis (the one of spl1 = the one of spl2) which is closest to
        # the one in the union basis, by solving a system

        coeffs = []  # holds new coeffs
        degree = [s.basis.degree for s in spl1]
        knots = [s.basis.knots*motion_time for s in spl1]  # scale knots with guess for motion time
        for l in range (len(spl1)):
            new_basis =  BSplineBasis(knots[l], degree[l])  # make basis with new knot sequence
            grev_bc = new_basis.greville()
            # shift greville points inwards, to avoid that evaluation at the greville points returns
            # zero, because they fall outside the domain due to numerical errors
            grev_bc[0] = grev_bc[0] + (grev_bc[1]-grev_bc[0])*0.01
            grev_bc[-1] = grev_bc[-1] - (grev_bc[-1]-grev_bc[-2])*0.01
            # evaluate connection of splines greville points of new basis
            eval_sc = spl[l](grev_bc)
            # evaluate basis at its greville points
            eval_bc = new_basis(grev_bc).toarray()
            # solve system to obtain coefficients of spl in new_basis
            coeffs.append(la.solve(eval_bc, eval_sc))
        # put in correct format
        init_splines = np.r_[coeffs].transpose()

        return init_splines, motion_time

    def make_border(self, xmin, ymin, xmax, ymax):
        width = xmax - xmin
        height = ymax - ymin
        angle = 0.
        # add angle to position
        position = [(xmax - xmin)*0.5+xmin,(ymax-ymin)*0.5+ymin, angle]
        limits = [xmin, ymin, xmax, ymax]
        return {'shape': Rectangle(width=width, height=height),
         'position': position, 'orientation': angle, 'limits': limits}

    def create_next_frame(self, frame):
        # only used if self.n_frames = 1

        if not frame['endpoint_frame'] == self.goal_state[:2]:  # remove orientation info
            start = time.time()
            start_position = frame['endpoint_frame']  # start at end of current frame
            if self.frame_type is 'shift':
               next_frame = self.create_frame_shift(start_position)

            elif self.frame_type is 'min_nobs':
                # only search the next frame if the current frame doesn't contain the goal state
                next_frame = self.create_min_nobs_base_frame(start_position)

                # try to scale up frame
                next_frame['border'], next_frame['waypoints'] = self.scale_up_frame(next_frame)

                # possibly the last waypoint is not reachable by the vehicle, fix this
                next_frame = self.make_last_waypoint_reachable(next_frame, method='move_point')

                # finish frame description
                # next_frame['border'] is already determined
                stationary_obstacles = self.get_stationary_obstacles_in_frame(next_frame)
                next_frame['stationary_obstacles'] = stationary_obstacles
                # Last waypoint of frame is a point of global_path or the goal_state which
                # was added to self.global_path. This was necessary because otherwise you
                # will end up on a grid point and not necessarily in the goal position
                # (which can lie between grid points, anywhere on the map)
                next_frame['endpoint_frame'] = next_frame['waypoints'][-1]

            end = time.time()
            if self.options['verbose'] >= 2:
                print 'time spend in create_next_frame, ', end-start
            return next_frame
        else:
            # tried to create the next frame, while the goal position is already inside the current frame
            return None

    def create_min_nobs_base_frame(self, start_position):
        # generates a minimal area frame without any stationary obstacles inside
        # this is done by subsequently including more waypoints, until an obstacle
        # is found inside the frame
        # this gives a frame with width or height of the vehicle size,
        # later this frame is scaled up

        start_time = time.time()

        # make new dictionary, to avoid that self.frame keeps the same reference
        frame = {}

        # frame with current vehicle position as center,
        # width and height are determined by the vehicle size
        xmin = start_position[0] - self.veh_size*self.margin
        ymin = start_position[1] - self.veh_size*self.margin
        xmax = start_position[0] + self.veh_size*self.margin
        ymax = start_position[1] + self.veh_size*self.margin
        frame['border'] = self.make_border(xmin,ymin,xmax,ymax)

        _, index = self.find_closest_waypoint(start_position, self.global_path)
        points_in_frame = []  # holds all waypoints in the frame

        # run over all waypoints, starting from the waypoint closest to start_position
        # first try if endpoint can be inside the frame without obstacles
        point = self.global_path[-1]
        if not self.point_in_frame(frame, point):
            # next waypoint is not inside frame,
            # enlarge frame such that it is in there
            # determine which borders to move

            # make frame with first point = start_position and next point = goal
            prev_point = start_position

            frame = self.update_frame_with_waypoint(frame, prev_point, point)
            # check if there are any obstacles inside this new frame
            stationary_obstacles = self.get_stationary_obstacles_in_frame(frame)

            if stationary_obstacles:
                # there is an obstacle inside the frame after enlarging
                # don't add point and keep old frame
                frame['border'] = self.make_border(xmin,ymin,xmax,ymax)
                # current frame cannot contain the endpoint
                for idx, point in enumerate(self.global_path[index:]):
                    # update limits of frame
                    xmin,ymin,xmax,ymax = frame['border']['limits']

                    if not self.point_in_frame(frame, point):
                        # next waypoint is not inside frame,
                        # enlarge frame such that it is in there
                        # determine which borders to move
                        if points_in_frame:
                            # assign last point in frame
                            prev_point = points_in_frame[-1]
                        else:
                            # no points in frame yet, so compare with current state
                            prev_point = start_position

                        frame = self.update_frame_with_waypoint(frame, prev_point, point)
                        # check if there are any obstacles inside this new frame
                        stationary_obstacles = self.get_stationary_obstacles_in_frame(frame)

                        if stationary_obstacles:
                            # there is an obstacle inside the frame after enlarging
                            # don't add point and keep old frame
                            frame['border'] = self.make_border(xmin,ymin,xmax,ymax)
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
            frame['endpoint_frame'] = point
        if not points_in_frame:
            raise RuntimeError('No waypoint was found inside min_nobs frame, something wrong with frame')
        else:
            frame['waypoints'] = points_in_frame

        end_time = time.time()
        print 'time in min_nobs_frame', end_time-start_time

        return frame

    def update_frame_with_waypoint(self, frame, prev_point, point):
        # updates the frame when adding a new waypoint to the frame
        # compare point with prev_point
        xmin,ymin,xmax,ymax = frame['border']['limits']
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

        frame['border'] = self.make_border(xmin_new,ymin_new,xmax_new,ymax_new)

        return frame

    def scale_up_frame(self, frame):
        # scale up the current frame in all directions, until it hits the borders
        # or it contains an obstacle

        # Note: when incrementally making the frame larger, we update with a certain size.
        # Updating with self.veh_size*self.margin may be too big, such that frames are not as
        # wide/large as they can be. Changing e.g. to xmax_new = xmax + 0.1, is more accurate,
        # but this takes more time to compute.
        # the selection is set by the self.scale_up_fine boolean

        start_time = time.time()

        scaled_frame = frame.copy()
        xmin,ymin,xmax,ymax = scaled_frame['border']['limits']

        # enlarge in positive x-direction
        # first try to put xmax = frame border
        xmax_new = self.environment.room[0]['shape'].get_canvas_limits()[0][1] + self.environment.room[0]['position'][0]
        scaled_frame['border'] = self.make_border(xmin,ymin,xmax_new,ymax)
        if not self.get_stationary_obstacles_in_frame(scaled_frame):
            xmax = xmax_new  # assign new xmax
        else:
            while True:
                    if self.scale_up_fine:
                        xmax_new = xmax + 0.1
                    else:
                        xmax_new = xmax + self.veh_size*self.margin
                    scaled_frame['border'] = self.make_border(xmin,ymin,xmax_new,ymax)
                    if xmax_new > self.environment.room[0]['shape'].get_canvas_limits()[0][1] + self.environment.room[0]['position'][0]:
                        # the frame hit the borders, this is the maximum size in this direction
                        break
                    if not self.get_stationary_obstacles_in_frame(scaled_frame):
                        # there is no obstacle in the enlarged frame, so enlarge it
                        xmax = xmax_new
                    else:
                        # there is an obstacle in the enlarged frame, don't enlarge it
                        scaled_frame['border'] = self.make_border(xmin,ymin,xmax,ymax)
                        break

        # enlarge in negative x-direction
        # first try to put xmin = frame border
        xmin_new = self.environment.room[0]['shape'].get_canvas_limits()[0][0] + self.environment.room[0]['position'][0]
        scaled_frame['border'] = self.make_border(xmin_new,ymin,xmax,ymax)
        if not self.get_stationary_obstacles_in_frame(scaled_frame):
            xmin = xmin_new  # assign new xmin
        else:
            while True:
                if self.scale_up_fine:
                    xmin_new = xmin - 0.1
                else:
                    xmin_new = xmin - self.veh_size*self.margin
                scaled_frame['border'] = self.make_border(xmin_new,ymin,xmax,ymax)
                if xmin_new < self.environment.room[0]['shape'].get_canvas_limits()[0][0] + self.environment.room[0]['position'][0]:
                    # the frame hit the borders, this is the maximum size in this direction
                    break
                if not self.get_stationary_obstacles_in_frame(scaled_frame):
                    xmin = xmin_new
                else:
                    scaled_frame['border'] = self.make_border(xmin,ymin,xmax,ymax)
                    break

        # enlarge in positive y-direction
        # first try to put ymax = frame border
        ymax_new = self.environment.room[0]['shape'].get_canvas_limits()[1][1] + self.environment.room[0]['position'][1]
        scaled_frame['border'] = self.make_border(xmin,ymin,xmax,ymax_new)
        if not self.get_stationary_obstacles_in_frame(scaled_frame):
            ymax = ymax_new  # assign new ymax
        else:
            while True:
                if self.scale_up_fine:
                    ymax_new = ymax + 0.1
                else:
                    ymax_new = ymax + self.veh_size*self.margin
                scaled_frame['border'] = self.make_border(xmin,ymin,xmax,ymax_new)
                if ymax_new > self.environment.room[0]['shape'].get_canvas_limits()[1][1] + self.environment.room[0]['position'][1]:
                    # the frame hit the borders, this is the maximum size in this direction
                    break
                if not self.get_stationary_obstacles_in_frame(scaled_frame):
                    ymax = ymax_new
                else:
                    scaled_frame['border'] = self.make_border(xmin,ymin,xmax,ymax)
                    break

        # enlarge in negative y-direction
        # first try to put ymin = frame border
        ymin_new = self.environment.room[0]['shape'].get_canvas_limits()[1][0] + self.environment.room[0]['position'][1]
        scaled_frame['border'] = self.make_border(xmin,ymin_new,xmax,ymax)
        if not self.get_stationary_obstacles_in_frame(scaled_frame):
            ymin = ymin_new  # assign new ymin
        else:
            while True:
                if self.scale_up_fine:
                    ymin_new = ymin - 0.1
                else:
                    ymin_new = ymin - self.veh_size*self.margin
                scaled_frame['border'] = self.make_border(xmin,ymin_new,xmax,ymax)
                if ymin_new < self.environment.room[0]['shape'].get_canvas_limits()[1][0] + self.environment.room[0]['position'][1]:
                    # the frame hit the borders, this is the maximum size in this direction
                    break
                if not self.get_stationary_obstacles_in_frame(scaled_frame):
                    ymin = ymin_new
                else:
                    scaled_frame['border'] = self.make_border(xmin,ymin,xmax,ymax)
                    break

        # update waypoints
        # starting from the last waypoint that was already in the frame
        # vehicle size was already taken into account above, when shifting borders
        index = self.global_path.index(frame['waypoints'][-1])
        for idx, point in enumerate(self.global_path[index:]):
            if self.point_in_frame(scaled_frame, point):
                if not point in scaled_frame['waypoints']:
                    # point was not yet a waypoint of the frame,
                    # but it is inside the scaled frame
                    scaled_frame['waypoints'].append(point)
            else:
                # waypoint was not inside the scaled_frame, stop looking
                break

        end_time = time.time()
        if self.options['verbose'] >= 2:
            print 'time in scale_up_frame', end_time-start_time
        return scaled_frame['border'], scaled_frame['waypoints']

    def move_frame(self, start_position, delta_x, delta_y, move_limit):
        # only used for 'frame_type'='shift'

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

        # newx_lower = self.frame_size*0.5, meaning that we keep the frame in the center, or adapted with move_limit
        newx_min = start_position[0] - newx_lower
        newy_min = start_position[1] - newy_lower
        newx_max = start_position[0] + newx_upper
        newy_max = start_position[1] + newy_upper

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

    def point_in_frame(self, frame, point, time=None, velocity=None, distance=0):
        # check if the provided point is inside frame
        # both for stationary or moving points
        # distance is the margin to take into account
        xmin, ymin, xmax, ymax = frame['border']['limits']

        # check stationary point
        if time is None:
            if (xmin+distance < point[0] < xmax-distance) and (ymin+distance < point[1] < ymax-distance):
                return True
            else:
                return False
        # check moving point
        elif isinstance(time, (float, int)):
            # amount of times to check
            N = int(round(time/self.check_moving_obs_ts)+1)
            Ts = self.check_moving_obs_ts
            x, y = point
            vx, vy = velocity
            for l in range(N+1):
                if xmin <= (x+l*Ts*vx) <= xmax and ymin <= (y+l*Ts*vy) <= ymax:
                    return True
            return False
        else:
            raise RuntimeError('Argument time was of the wrong type, not None, float or int')

    def find_intersection_line_segment_frame(self, frame, line):
        # find intersection point of the provided line with frame
        x3, y3, x4, y4 = frame['border']['limits']

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
            raise RuntimeError('No intersection between line and frame found!')
        return intersection_point

    # def move_from_border(self, start, end, frame, distance=0):
    #     # Note: this function is only used when frame_type='shift'

    #     # this function moves 'end' away from the frame border
    #     # approach: first try to move the end point over the line between start and end
    #     # if point is still too close to border, move in x and y separately, not over the
    #     # line anymore
    #     # the second step is required e.g. when the point is too close to y-limit,
    #     # but the line between start and end is horizontal

    #     # move the end point/goal position inside the frame back such that it
    #     # is not too close to the border
    #     length = np.sqrt((end[0]-start[0])**2+(end[1]-start[1])**2)  # distance between start and end
    #     angle = np.arctan2(end[1]-start[1],end[0]-start[0])  # angle of line between start and end

    #     # 'distance' is the required perpendicular distance
    #     # 'move_back' is how far to move back over the connection
    #     # to obtain the desired distance
    #     if (abs(start[0] - end[0]) <= 1e-4) or (abs(start[1] - end[1]) <= 1e-4):
    #         # vertical or horizontal line: move over distance
    #         move_back = distance
    #     else:
    #         # take into account the angle
    #         # find required distance to obtain required distance in both x- and y-direction
    #         move_back = max(abs(distance/np.cos(angle)), abs(distance/np.sin(angle)))
    #     new_length = length - move_back

    #     end_shifted = [0.,0.]
    #     end_shifted[0] = start[0] + new_length*np.cos(angle)
    #     end_shifted[1] = start[1] + new_length*np.sin(angle)

    #     # if still too close to border, move in x and y separately
    #     dist_to_border = self.distance_to_border(frame, end_shifted)
    #     if abs(dist_to_border[0]) - distance < 0:
    #         move_distance = distance - abs(dist_to_border[0])
    #         if dist_to_border[0]<=0:
    #             end_shifted[0] = end_shifted[0] + move_distance
    #         else:
    #             end_shifted[0] = end_shifted[0] - move_distance
    #     if abs(dist_to_border[1]) - distance < 0:
    #         move_distance = distance - abs(dist_to_border[1])
    #         if dist_to_border[1]<=0:
    #             end_shifted[1] = end_shifted[1] + move_distance
    #         else:
    #             end_shifted[1] = end_shifted[1] - move_distance

    #     return end_shifted

    def distance_to_border(self, frame, point):
        # returns the x- and y-direction distance from point to the border of frame
        # based on: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        # when point is outside of the border, the function also returns the distance to the border,
        # and you can use point_in_rectangle() to check if point is inside the border or not

        x2, y2, x3, y3 = frame['border']['limits']
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

    def find_closest_waypoint(self, position, waypoints):

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

    def generate_problem(self):
        # transform frames into a multiframe problem

        room = []
        for k in range(self.n_frames):
            new_room = {}
            new_room['shape'] = self.frames[k]['border']['shape']
            new_room['position'] = self.frames[k]['border']['position']
            new_room['draw'] = True
            room.append(new_room)
        environment = Environment(room=room)

        for k in range(self.n_frames):
            obstacles = self.frames[k]['stationary_obstacles']+self.frames[k]['moving_obstacles']
            environment.fill_room(room[k], obstacles)

        # create problem
        problem_options = {}
        for key, value in self.problem_options.items():
            problem_options[key] = value
        if not self.problem_options['freeT']:
            # fixedT problem, only possible with Point2point problem
            problem = Point2point(self.vehicles, environment, freeT=self.problem_options['freeT'], options=problem_options)
        else:
            problem = MultiFrameProblem(self.vehicles, environment, n_frames=self.n_frames)
        problem.set_options({'solver_options': self.options['solver_options']})
        problem.init()
        # reset the current_time, to ensure that predict uses the provided
        # last input of previous problem and vehicle velocity is kept from one frame to another
        problem.initialize(current_time=0.)
        return problem