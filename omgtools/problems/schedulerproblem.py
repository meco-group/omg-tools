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
from globalplanner import AStarPlanner
from ..environment.environment import Environment
from ..environment.frame import ShiftFrame, CorridorFrame
from ..basics.shape import Rectangle, Circle
from ..basics.spline import BSplineBasis
from ..basics.spline_extra import concat_splines

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
        self.cnt = 1  # frame counter
        self.n_frames = options['n_frames'] if 'n_frames' in options else 1  # amount of frames to combine

        if (self.n_frames > 1 and not self.problem_options['freeT']):
            raise ValueError('Fixed time problems are only supported for n_frames = 1')
        self._n_frames = self.n_frames  # save original value
        self.frame_type = options['frame_type'] if 'frame_type' in options else 'shift'
        # set frame size for frame_type shift
        if self.frame_type is 'shift':
            # by default frame size is set to 1/5 of total environment width
            self.frame_size = options['frame_size'] if 'frame_size' in options else environment.room[0]['shape'].width*0.2
            # by default move limit is set to 1/4 of frame size
            self.move_limit = options['move_limit'] if 'move_limit' in options else self.frame_size*0.25
        if self.frame_type is 'corridor':
            # scale up frame with small steps or not
            self.scale_up_fine = options['scale_up_fine'] if 'scale_up_fine' in options else True
            self.l_shape = options['l_shape'] if 'l_shape' in options else False
        # check if vehicle size is larger than the cell size
        n_cells = self.global_planner.grid.n_cells
        if (size_to_check >= (min(environment.room[0]['shape'].width/float(n_cells[0]), \
                                  environment.room[0]['shape'].height/float(n_cells[1])))
           and self.frame_type == 'corridor'):
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
        for idx, frame in enumerate(self.frames):
            # updates self.frames[:]['moving_obstacles']
            frame.moving_obstacles = frame.get_moving_obstacles(self.motion_times[idx])

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
            for idx, frame in enumerate(self.frames):
                moving_obs_in_frame = frame.get_moving_obstacles(self.motion_times[idx])
                if set(moving_obs_in_frame) != set(frame.moving_obstacles):
                    # the amount of moving obstacles changed, or there is a new obstacle
                    new_problem = True
                    frame.moving_obstacles = moving_obs_in_frame
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
        if self.frames[-1].endpoint == self.goal_state[:2]:  # remove orientation info
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
                s, l = self.frames[0].border['shape'].draw(self.frames[0].border['position'])
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
                s, l = self.frame_storage[t][k].border['shape'].draw(self.frame_storage[t][k].border['position'])
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

        # there are two different options: shift and corridor
        # corridor: change frame size such that the frame is as large as possible, without
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
            if frame and frame.waypoints[-1] == self.goal_state[:2]:  # remove orientation info
                # reduce amount of frames that are considered and stop loop
                self.n_frames -= 1
                break
            # set start position for computation of frame that we will add
            if frame:
                start_pos = frame.waypoints[-1]
            else:
                # there were no frames yet, so compute self.n_frames, starting from current state
                # remove orientation from state (if using e.g. differential drive)
                start_pos = self.curr_state[:2]
            if self.frame_type == 'shift':
                frame = ShiftFrame(self.environment, start_pos, self.frame_size, self.move_limit,
                                   self.global_path, self.veh_size, self.margin, self.options)
            elif self.frame_type == 'corridor':
                frame = CorridorFrame(self.environment, start_pos, self.global_path,
                                      self.veh_size, self.margin, self.options)
            else:
                raise RuntimeError('Invalid frame type: ', self.frame_type)
            # append new frame to the frame list
            self.frames.append(frame)

            if self.n_frames == 1:
                # then we need a next_frame to create a region of overlap, determining when to switch frames
                self.next_frame = self.create_next_frame()

        if (self.frame_type == 'corridor' and self.l_shape):
            # reshape the current frames into an L-shape
            if (self.n_frames == 1 and hasattr(self, 'next_frame')):
                # looking only one frame ahead
                frames = self.frames[0].create_l_shape(self.next_frame)
                self.frames = [frames[0]]
                self.next_frame = frames[1]
            else:
                # reshape all frames until the last one
                for k in range(len(self.frames)-1):
                    self.frames[k], self.frames[k+1]= self.frames[k].create_l_shape(self.frames[k+1])
                # reshape last frame by computing next_frame
                next_frame = self.create_next_frame(frame=self.frames[-1])
                if next_frame is not None:
                    # there is still a frame after the next one
                    self.frames[-1], _ = self.frames[-1].create_l_shape(next_frame)

        end_time = time.time()
        if self.options['verbose'] >= 2:
            print 'elapsed time while creating new ' + self.frame_type + ' frame: ', end_time-start_time

    def create_next_frame(self, frame=None):
        # only used if self.n_frames = 1

        if frame is None:
            frame = self.frames[0]

        if not frame.endpoint == self.goal_state[:2]:  # remove orientation info
            start = time.time()
            start_position = frame.endpoint  # start at end of current frame
            if frame.type is 'shift':
                next_frame = ShiftFrame(self.environment, start_position, self.frame_size, self.move_limit,
                                   self.global_path, self.veh_size, self.margin, self.options)
            elif frame.type is 'corridor':
                next_frame = CorridorFrame(self.environment, start_position, self.global_path,
                                   self.veh_size, self.margin, self.options)

            end = time.time()
            if self.options['verbose'] >= 2:
                print 'time spend in create_next_frame, ', end-start
            return next_frame
        else:
            # tried to create the next frame, while the goal position is already inside the current frame
            return None

    def check_frames(self):
        # if final goal is not in the current frame, compare current distance
        # to the local goal with the initial distance
        if not self.frames[0].point_in_frame(self.goal_state[:2]):
            if (self.n_frames == 1 and hasattr(self, 'next_frame')):
                if self.next_frame.point_in_frame(self.curr_state[:2], distance=self.veh_size):
                    # only called if self.n_frames = 1,
                    # then self.frames[1] doesn't exist and self.next_frame does exist
                    # current state is inside the next frame, so switch
                    valid = False
                else:
                    valid = True
                return valid
            elif self.frames[1].point_in_frame(self.curr_state[:2], distance=self.veh_size):
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
            new_frame = self.create_next_frame(frame=self.next_frame)
            # the next frame becomes the current frame
            self.frames[0] = self.next_frame
            # new_frame becomes the next frame
            # Note: if the current frame is the last one, new_frame will be None
            if new_frame is not None:
                self.next_frame = new_frame
            else:
                self.next_frame = None
            if (self.frame_type == 'corridor' and self.l_shape):
                # reshape the current and next frame into an L-shape
                if self.next_frame is not None:
                    # there is still coming a next frame
                    frames = self.frames[0].create_l_shape(self.next_frame)
                    self.frames = [frames[0]]
                    self.next_frame = frames[1]
        else:
            self.create_frames()

        # use previous solution to get an initial guess for all frames except the last one,
        # for this one get initial guess based on global path
        # analogously for the motion_times
        self.init_guess, self.motion_times = self.get_init_guess()

        # get moving obstacles inside frame for this time
        for idx, frame in enumerate(self.frames):
            frame.moving_obstacles = frame.get_moving_obstacles(self.motion_times[idx])

        end_time = time.time()
        if self.options['verbose'] >= 2:
            print 'elapsed time while updating frame: ', end_time-start_time

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
                    init_splines.append(np.array(self.local_problem.father.get_variables()[self.vehicles[0].label,'splines_seg'+str(k)]))
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
            if self.frames[-1].waypoints[-1] == self.goal_state[:2]:
                self.vehicles[0].set_terminal_conditions(self.goal_state)  # setting goal for final frame
            else:
                if hasattr(self, 'next_frame'):
                    # use last waypoint from current and first waypoint
                    # from next frame to compute desired orientation
                    # take second-last waypoint, since last one was moved in make_reachable()
                    x1,y1 = self.frames[-1].waypoints[-2]
                    # take second waypoint, since first will be == second-last of previous frame
                    x2,y2 = self.next_frame.waypoints[1]
                    # compute angle
                    angle = np.arctan2((y2-y1),(x2-x1))
                else:
                    # compute angle between last and second last waypoint inside frame
                    # to use as a desired orientation
                    x1,y1 = self.frames[-1].waypoints[-2]
                    x2,y2 = self.frames[-1].waypoints[-1]
                    angle = np.arctan2((y2-y1),(x2-x1))
                # desired pose = last waypoint, with compute angle
                pose = self.frames[-1].waypoints[-1] + [angle]
                self.vehicles[0].set_terminal_conditions(pose)
        elif isinstance(self.vehicles[0], Holonomic):
            self.vehicles[0].set_terminal_conditions(self.frames[-1].waypoints[-1])
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

        waypoints = frame.waypoints
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

    def generate_problem(self):
        # transform frames into a multiframe problem

        room = []
        for k in range(self.n_frames):
            new_room = {}
            new_room['shape'] = self.frames[k].border['shape']
            new_room['position'] = self.frames[k].border['position']
            new_room['draw'] = True
            room.append(new_room)
        environment = Environment(room=room)

        for k in range(self.n_frames):
            obstacles = self.frames[k].stationary_obstacles+self.frames[k].moving_obstacles
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

    # def find_intersection_line_segment_frame(self, frame, line):
    #     # find intersection point of the provided line with frame
    #     x3, y3, x4, y4 = frame['border']['limits']

    #     # frame border representation:
    #     # [x3,y4]---------[x4,y4]
    #     #    |               |
    #     #    |               |
    #     #    |               |
    #     # [x3,y3]---------[x4,y3]

    #     # move over border in clockwise direction:
    #     top_side    = [[x3,y4],[x4,y4]]
    #     right_side  = [[x4,y4],[x4,y3]]
    #     bottom_side = [[x4,y3],[x3,y3]]
    #     left_side   = [[x3,y3],[x3,y4]]

    #     # First find which line segments intersect, afterwards use a method
    #     # for line intersection to find the intersection point. Not possible
    #     # to use intersect_lines immediately since it doesn't take into account
    #     # the segments, but considers infinitely long lines.

    #     #intersection with top side?
    #     if intersect_line_segments(line, top_side):
    #         # find intersection point
    #         intersection_point = intersect_lines(line, top_side)
    #     #intersection with right side?
    #     elif intersect_line_segments(line, right_side):
    #         # find intersection point
    #         intersection_point = intersect_lines(line, right_side)
    #     #intersection with bottom side?
    #     elif intersect_line_segments(line, bottom_side):
    #         # find intersection point
    #         intersection_point = intersect_lines(line, bottom_side)
    #     #intersection with left side?
    #     elif intersect_line_segments(line, left_side):
    #         # find intersection point
    #         intersection_point = intersect_lines(line, left_side)
    #     else:
    #         raise RuntimeError('No intersection between line and frame found!')
    #     return intersection_point

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