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
from point2point import Point2point
from ..basics.shape import Rectangle, Circle
from ..basics.geometry import distance_between_points, intersect_lines, intersect_line_segments
from ..environment.environment import Environment
from globalplanner import AStarPlanner

from scipy.interpolate import interp1d
import numpy as np

class MultiFrameProblem(Problem):

    def __init__(self, fleet, environment, global_planner=None, options=None):
        Problem.__init__(self, fleet, environment, options, label='multiproblem')
        # Todo: positionT is for Holonomic, should this become poseT?
        self.curr_state = self.vehicles[0].prediction['state'] # initial vehicle position
        self.goal_state = self.vehicles[0].positionT # overall goal 

        if global_planner is not None:
            self.global_planner = global_planner
        else: 
            self.global_planner = AStarPlanner(environment, 10, self.curr_state, self.goal_state)

        self.problem_options = options  # e.g. selection of problem type (freeT, fixedT)
        
        self.start_time = 0.
        self.update_times=[]

        self.frame = {}
        self.frame['type'] = 'frame_shift'
        self.frame_size = 2.5 #150 #2.5
        self.cnt = 1  # frame counter        

    def init(self):
        # otherwise the init of Problem is called, which is not desirable
        pass

    def initialize(self, current_time):
        self.local_problem.initialize(current_time)

    def reinitialize(self,):
        # this function is called at the start

        # Create first frame:

        # get a global path
        if self.problem_options['fixed_example']:
            # # for now fix global path to solve one case
            print 'Using global_path for known example'
            self.global_path = [[192.546875,   32.171875], [178.53125 ,   38.40625 ], 
                                [178.53125,   63.34375], [178.53125,   88.28125],
                                [164.515625,  106.984375], [164.515625,  119.453125],
                                [159.84375 ,  138.15625 ],[141.15625,  163.09375],
                                [136.484375,  181.796875], [136.484375,  194.265625],
                                [122.46875 ,  212.96875 ], [122.46875,  237.90625],
                                [110.7890625,  247.2578125],[106.1171875,  247.2578125],
                                [101.4453125,  247.2578125],[ 96.7734375,  247.2578125],
                                [ 92.1015625,  247.2578125],[ 87.4296875,  247.2578125],
                                [ 82.7578125,  247.2578125],[ 78.0859375,  247.2578125],
                                [ 73.4140625,  247.2578125],[ 68.7421875,  247.2578125],
                                [ 64.0703125,  247.2578125],[ 59.3984375,  247.2578125],
                                [ 47.71875  ,  262.84375  ],[ 29.03125,  287.78125],
                                [ 29.03125,  312.71875],[ 29.03125,  337.65625],
                                [ 29.03125,  362.59375]]
        else:
            self.global_path = self.global_planner.get_path()
        # append goal state to waypoints, since desired goal is not necessarily a waypoint
        self.global_path.append(self.goal_state)

        # plot global path
        self.global_planner.grid.draw()
        # don't include last waypoint, since it is the manually added goal_state
        self.global_planner.plot_path(self.global_path[:-1])

        # get a frame, fills in self.frame
        self.create_frame(frame_size=self.frame_size)

        # get initial guess based on global path, get motion time
        init_guess, self.motion_time, _ = self.get_init_guess()

        # get moving obstacles inside frame for this time
        self.frame['moving_obstacles'] = self.get_moving_obstacles_in_frame()

        # get a problem representation of the frame
        problem = self.generate_problem()  # transform frame into point2point problem
        problem.reset_init_guess(init_guess)  # Todo: is this function doing what we want?

        # the big multiframe problem (self) has a local problem at each moment
        self.local_problem = problem

    def solve(self, current_time, update_time):
        frame_valid = self.check_frame()
        if not frame_valid:
            self.cnt += 1  # count frame

            # frame was not valid anymore, update frame based on current position
            self.update_frame()

            # transform frame into problem and simulate
            problem = self.generate_problem()
            # Todo: is this a new guess?
            # init_guess, _, _ = self.get_init_guess()
            # problem.init()
            # self.init_guess comes from update_frame
            problem.reset_init_guess(self.init_guess)
            self.local_problem = problem        
        else: 
            moving_obstacles = self.get_moving_obstacles_in_frame()
            # Check if amount of moving obstacles changed
            # If the obstacles just change their trajectory, the simulator takes this into account            
            # If moving_obstacles is different from the existing moving_obstacles (new obstacle,
            # or disappeared obstacle) make a new problem.

            # Todo: if obstacle is not relevant anymore, place it far away and set hyperplanes accordingly?
            # Todo: make new problem or place dummy obstacles and overwrite their state? for now dummies have influence on solving time
            for obs1 in moving_obstacles:
                for obs2 in self.frame['moving_obstacles']:
                    if obs1==obs2 and obs1.options['avoid'] != obs2.options['avoid']:
                        # new moving obstacle in frame or obstacle disappeared from frame
                        problem = self.generate_problem()
                        # problem.init()
                        init_guess = self.local_problem.father.get_variables()[self.vehicles[0].label, 'splines0']
                        problem.reset_init_guess(init_guess)  # use init_guess from previous problem = best we can do
                        self.local_problem = problem
        
        # solve local problem
        self.local_problem.solve(current_time, update_time)

        # save solving time
        self.update_times.append(self.local_problem.update_times[-1])

        # Todo: how get current position in a clean way?
        # np.array converts CasADi DM to numbers
        if not hasattr(self.vehicles[0], 'signals'):
            # first iteration
            self.curr_state = self.vehicles[0].prediction['state']
        else:
            # all other iterations
            self.curr_state = self.vehicles[0].signals['pose'][:,-1]
            
    def store(self, current_time, update_time, sample_time):        
        # call store of local problem
        self.local_problem.store(current_time, update_time, sample_time)

    def stop_criterium(self, current_time, update_time):
        # check if the current frame is the last one
        if self.frame['endpoint_frame'] == self.goal_state:
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
    
    def simulate(self, current_time, simulation_time, sample_time):
        # save global path and frame border
        # store trajectories
        if not hasattr(self, 'frame_storage'):
            self.frame_storage = []
            self.global_path_storage = []
        repeat = int(simulation_time/sample_time)
        self._add_to_memory(self.frame_storage, self.frame, repeat)
        self._add_to_memory(self.global_path_storage, self.global_path, repeat)
        
        #this is gonna call self.update_plots, so don't do it here
        Problem.simulate(self, current_time, simulation_time, sample_time)

    def _add_to_memory(self, memory, data_to_add, repeat=1):
            # if not isinstance(data_to_add, list):
            #     data_to_add = [data_to_add]
            memory.extend([data_to_add for k in range(repeat)])

    def export(self, options=None):
        # ignored this functionality for now
        raise NotImplementedError('Please implement this method!')

    def init_plot(self, argument, **kwargs):
        # initialize environment plot
        info = Problem.init_plot(self, argument)
        if info is not None:
            # initialize frame plot
            for _ in self.frame['border']['shape'].draw(self.frame['border']['position']):
                info[0][0]['lines'].append({'color': 'grey', 'linestyle' : '--', 'linewidth': 1.2})
            # initialize global path plot
            info[0][0]['lines'].append({'color': 'red', 'linestyle' : '--', 'linewidth': 1.2})
        return info        

    def update_plot(self, argument, t, **kwargs):
        # plot environment
        data = Problem.update_plot(self, argument, t)

        if data is not None:
            # plot frame border
            for l in self.frame_storage[t]['border']['shape'].draw(self.frame_storage[t]['border']['position']):
                data[0][0].append([l[k, :] for k in range(self.frame_storage[t]['border']['shape'].n_dim)])
            # plot global path
            # remove last waypoint, since this is the goal position,
            # which was manually added and is not necessarily a grid point
            length = np.shape(self.global_path_storage[t][:-1])[0]
            waypoints = np.array(np.zeros((2,length)))
            for idx, waypoint in enumerate(self.global_path_storage[t][:-1]):
                waypoints[0,idx] = waypoint[0]
                waypoints[1,idx] = waypoint[1]
            data[0][0].append([waypoints[k, :] for k in range(2)])
        return data

    # ========================================================================
    # Multiproblem specific functions
    # ========================================================================

    def create_frame(self, frame_size=1.5):
        # Makes a frame, based on using the environment, the current state and the global path (waypoints)

        # There are different options: frame_shift, min_nobs, corridor
        # frame_shift: keep frame_size fixed, shift frame in the direction of the movement
        # over a maximum distance of move_limit
        # min_nobs: start with a certain frame size, shift frame and adapt size to get a minimum amount
        # of obstacles inside a frame
        # corridor: a frame consists solely of a corridor, without any static obstacles
        if self.frame['type'] == 'frame_shift':
            # make new dictionary, to avoid that self.frame keeps the same reference
            self.frame = {}
            self.frame['type'] = 'frame_shift'
            # frame with current position as center
            xmin = self.curr_state[0] - frame_size*0.5
            ymin = self.curr_state[1] - frame_size*0.5
            print 'Move_point_to_grid is not implemented for now'
            # Todo: but this is also not necessary, a frame can be placed wherever you want,
            # since it is not coupled to the grid planner
            # xmin, ymin = self.global_planner.move_point_to_grid(xmin, ymin)
            xmax = self.curr_state[0] + frame_size*0.5
            ymax = self.curr_state[1] + frame_size*0.5
            # xmax, ymax = self.global_planner.move_point_to_grid(xmax, ymax)
            self.frame['border'] = self.make_border(xmin,ymin,xmax,ymax)
            move_limit = frame_size*0.25  # move frame max over this distance

            # determine next waypoint outside frame so we can
            # change the position of the frame if needed
            waypoint = None  # holds waypoint outside the frame
            endpoint = None  # holds the goal point if it is inside the frame
            points_in_frame = []  # holds all waypoints in the frame
            for idx, point in enumerate(self.global_path):
                if not self.point_in_frame(point):
                    # Is point also out of frame when moving the frame towards the waypoint?
                    # If so, we move the window maximum 1/4 extra of size frame
                    
                    # Determine distance between waypoint out of frame and current state
                    delta_x = point[0] - self.curr_state[0]
                    delta_y = point[1] - self.curr_state[1]
                    if (abs(delta_x) > move_limit+frame_size*0.5 or abs(delta_y) > move_limit+frame_size*0.5):
                        waypoint = point  # waypoint outside frame, even after shifting
                        print 'Found waypoint outside frame'
                        break
                    # point is last point, and no points outside frame were found yet
                    elif point == self.global_path[-1]:
                        endpoint = point
                    else:
                        points_in_frame.append(point)  # waypoint inside frame after shifting
                else:
                    points_in_frame.append(point)  # waypoint inside frame

            # Optimize frame position based on next waypoint (= 'waypoint')
            if waypoint is not None:
                # found waypoint outside frame, which is not even inside the frame after shifting over move_limit
                # shift frame in the direction of this point
                # Todo: is this a good function to define? Or change inputs?
                xmin, ymin, xmax, ymax = self.move_frame(delta_x, delta_y, frame_size, move_limit)
                self.frame['border'] = self.make_border(xmin, ymin, xmax, ymax)
                # make line between last waypoint inside frame and first waypoint outside frame
                waypoint_line = [points_in_frame[-1], waypoint]
                # find intersection point between line and frame
                intersection_point = self.find_intersection_line_frame(waypoint_line)
                # shift endpoint away from border
                shape = self.vehicles[0].shapes[0]
                if isinstance(shape, Circle):
                    veh_size = shape.radius
                elif isinstance(shape, Rectangle):
                    veh_size = max(shape.width, shape.height)
                endpoint = self.shift_point_back(points_in_frame[-1], intersection_point, distance=veh_size*1.1)
            elif endpoint is not None:
                # vehicle goal is inside frame after shifting
                # shift frame over calculated distance
                xmin, ymin, xmax, ymax = self.move_frame(delta_x, delta_y, frame_size, move_limit)
                self.frame['border'] = self.make_border(xmin, ymin, xmax, ymax)
            else:
                # all waypoints are within frame, even without shifting, so don't shift the frame
                endpoint = self.global_path[-1]
            # Check if last waypoint is too close to the frame border, move the frame extra in that direction
            # Only needs to be checked in the else, since in other cases the frame was already shifted
            dist_to_border = self.distance_to_border(endpoint)
            shape = self.vehicles[0].shapes[0]
            if isinstance(shape, Circle):
                veh_size = shape.radius
            elif isinstance(shape, Rectangle):
                veh_size = max(shape.width, shape.height)
            if abs(dist_to_border[0]) <= veh_size:
                print 'Last waypoint too close in x-direction, moving frame'
                # move in x-direction
                move_distance = (veh_size - abs(dist_to_border[0]))*1.1
                if dist_to_border[0]<0: 
                    move_distance = -move_distance
                xmin = xmin + move_distance
                xmax = xmax + move_distance
                self.frame['border'] = self.make_border(xmin, ymin, xmax, ymax)
            if abs(dist_to_border[1]) <= veh_size:
                print 'Last waypoint too close in y-direction, moving frame'
                # move in y-direction
                move_distance = (veh_size - abs(dist_to_border[1]))*1.1
                if dist_to_border[1]<0: 
                    move_distance = -move_distance
                ymin = ymin + move_distance
                ymax = ymax + move_distance
                self.frame['border'] = self.make_border(xmin, ymin, xmax, ymax)                

        # Finish frame description
        # frame['border'] is already determined
        stationary_obstacles = self.get_stationary_obstacles_in_frame()
        print 'Stationary obstacles inside new frame: ', stationary_obstacles
        print 'first waypoint in new frame: ', points_in_frame[0]
        print 'last waypoint in new frame:', endpoint
        self.frame['stationary_obstacles'] = stationary_obstacles
        # If generated frame contains goal position, endpoint will be = goal position, since the goal position
        # was added to the global path. This was necessary because otherwise you will end up on a grid point
        # and not necessarily in the goal position (which can lie between grid points, anywhere on the map)
        self.frame['endpoint_frame'] = endpoint
        points_in_frame.append(endpoint)
        self.frame['waypoints'] = points_in_frame
        # Todo: added endpoint to waypoints, okay?

    def move_frame(self, delta_x, delta_y, frame_size, move_limit):
        # determine direction we have to move in
        newx_lower = newx_upper = frame_size*0.5
        newy_lower = newy_upper = frame_size*0.5

        # while moving, take into account veh_size
        shape = self.vehicles[0].shapes[0]
        if isinstance(shape, Circle):
            veh_size = shape.radius
        elif isinstance(shape, Rectangle):
            veh_size = max(shape.width, shape.height)

        # waypoint is outside frame in x-direction, shift in the waypoint direction
        if abs(delta_x) > frame_size*0.5:
            # move frame in x-direction, over a max of move_limit
            move = min(move_limit, abs(delta_x)-frame_size*0.5 + veh_size*1.1)
            if delta_x > 0:
                newx_lower = frame_size*0.5 - move
                newx_upper = frame_size*0.5 + move
            else:
                newx_lower = frame_size*0.5 + move
                newx_upper = frame_size*0.5 - move
        
        # waypoint is outside frame in y-direction, shift in the waypoint direction
        if abs(delta_y) > frame_size*0.5:
            # move frame in y-direction, over a max of move_limit
            move = min(move_limit, abs(delta_y)-frame_size*0.5 + veh_size*1.1)
            if delta_y > 0:
                newy_lower = frame_size*0.5 - move
                newy_upper = frame_size*0.5 + move
            else:
                newy_lower = frame_size*0.5 + move
                newy_upper = frame_size*0.5 - move

        # newx_lower = frame_size*0.5, meaning that we keep the frame in the center, or adapted with move_limit
        newx_min = self.curr_state[0] - newx_lower
        newy_min = self.curr_state[1] - newy_lower
        print 'Move_point_to_grid is not implemented for now'
        # Todo: this is also not necessary since frame is independent of global planner
        # newx_min, newy_min = self.global_planner.move_point_to_grid(newx_min, newy_min)
        newx_max = self.curr_state[0] + newx_upper
        newy_max = self.curr_state[1] + newy_upper
        # newx_max, newy_max = self.global_planner.move_point_to_grid(newx_max, newy_max)

        # make sure new limits are not outside the main environment room
        # Todo: shift frame or just change size, like now?
        environment_limits = self.environment.get_canvas_limits()  #[[xmin,xmax],[ymin,ymax]]
        if newx_min < environment_limits[0][0]:
            newx_min = environment_limits[0][0]
        if newx_max > environment_limits[0][1]:
            newx_max = environment_limits[0][1]
        if newy_min < environment_limits[1][0]:
            newy_min = environment_limits[1][0]
        if newy_max > environment_limits[1][1]:
            newy_max = environment_limits[1][1]

        return newx_min, newy_min, newx_max, newy_max

    def point_in_frame(self, point, time=None, velocity=None):
        xmin, ymin, xmax, ymax= self.frame['border']['limits']
        if time is None:
            if (xmin <= point[0] <= xmax) and (ymin <= point[1] <= ymax):
                return True
            else:
                return False
        elif isinstance(time, (float, int)):
            # time interval to check
            time_interval = 0.5
            # amount of times to check
            N = int(round(self.motion_time/time_interval)+1)
            # sample time of check
            Ts = self.motion_time/N
            x, y = point
            vx, vy = velocity
            for l in range(N+1):
                if xmin <= (x+l*Ts*vx) <= xmax and ymin <= (y+l*Ts*vy) <= ymax:
                    return True
        else:
            raise RuntimeError('Argument time was of the wrong type, not None, float or int')  

    def find_intersection_line_frame(self, line):
        x3, y3, x4, y4 = self.frame['border']['limits']

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
            raise ValueError('No intersection point was found, while a point outside the frame was found!')

        return intersection_point

    def shift_point_back(self, start, end, percentage=0, distance=0):
        length = np.sqrt((end[0]-start[0])**2+(end[1]-start[1])**2)
        angle = np.arctan2(end[1]-start[1],end[0]-start[0])

        # Todo: percentage is not robust, since it totally depends on the length?
        if percentage !=0:
            new_length = length * (1 - percentage/100.)
        elif distance != 0:
            # distance is the required perpendicular distance
            # move_back is how far to move back over the connection to obtain
            # the desired distance
            # vertical or horizontal line then move over distance
            if (start[0] == end[0]) or (start[1] == end[1]):
                move_back = distance
            else:
                move_back = max(abs(distance/np.cos(angle)), abs(distance/np.sin(angle)))
            new_length = length - move_back
            if new_length < 0:
                new_length = 0
                print 'Distance between last waypoint inside frame and', \
                      'intersection point with border is too small to shift',\
                      '(i.e. smaller than desired shift distance), selecting last waypoint inside frame'
                # raise ValueError('In shift_point_back, trying to shift too far')

        end_shifted = [0.,0.]
        end_shifted[0] = start[0] + new_length*np.cos(angle)
        end_shifted[1] = start[1] + new_length*np.sin(angle)

        return end_shifted

    def distance_to_border(self, point):
        # Returns the x- and y-direction distance from point to the border of frame
        # Based on: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        x2, y2, x3, y3 = self.frame['border']['limits']
        # number vertices of border
        # v2--v3
        # |   |
        # v1--v4
        v1 = [x2,y2]
        v2 = [x2,y3]
        v3 = [x3,y3]
        v4 = [x3,y2]

        # dist from v1-v2
        dist12 = abs((v2[1]-v1[1])*point[0] - (v2[0]-v1[0])*point[1] + v2[0]*v1[1] - v2[1]*v1[0])/(np.sqrt((v2[1]-v1[1])**2+(v2[0]-v1[0])**2))
        #dist from v2-v3
        dist23 = abs((v3[1]-v2[1])*point[0] - (v3[0]-v2[0])*point[1] + v3[0]*v2[1] - v3[1]*v2[0])/(np.sqrt((v3[1]-v2[1])**2+(v3[0]-v2[0])**2))
        #dist from v3-v4
        # Note the minus sign! A negative shift in y-direction is required to lower the distance
        dist34 = -abs((v4[1]-v3[1])*point[0] - (v4[0]-v3[0])*point[1] + v4[0]*v3[1] - v4[1]*v3[0])/(np.sqrt((v4[1]-v3[1])**2+(v4[0]-v3[0])**2))
        #dist from v4-v1
        # Note the minus sign! A negative shift in x-direction is required to lower the distance
        dist41 = -abs((v1[1]-v4[1])*point[0] - (v1[0]-v4[0])*point[1] + v1[0]*v4[1] - v1[1]*v4[0])/(np.sqrt((v1[1]-v4[1])**2+(v1[0]-v4[0])**2))
   
        distance = [0.,0.]
        distance[0] = min(abs(dist12), abs(dist34))  # x-direction: from point to side12 or side34
        distance[1] = min(abs(dist23), abs(dist41))  # y-direction

        return distance

    def get_stationary_obstacles_in_frame(self):
        obstacles_in_frame = []
        for obstacle in self.environment.obstacles:
            # check if obstacle is stationary, this is when:
            # there is no entry trajectories or there are trajectories but no velocity or 
            # all velocities are 0.
            if ((not 'trajectories' in obstacle.simulation) or (not 'velocity' in obstacle.simulation['trajectories'])
               or (all(vel == [0.]*obstacle.n_dim for vel in obstacle.simulation['trajectories']['velocity']['values']))):
                # get obstacle checkpoints
                obs_chck = obstacle.shape.get_checkpoints()[0]  # element [0] gives vertices, not the corresponding radii
                obs_pos = obstacle.signals['position'][:,-1]
                for chck in obs_chck:
                    # if it is not a circle, rotate the vertices
                    if hasattr(obstacle.shape, 'orientation'):
                        vertex = obstacle.shape.rotate(obstacle.shape.orientation, chck)
                    # move to correct position
                    vertex += obs_pos
                    # check if vertex is in frame
                    if self.point_in_frame(vertex):
                        # add corresponding obstacle
                        obstacles_in_frame.append(obstacle)
                        # break from for chck in obs_chck, move on to next obstacle,
                        # since if one vertex of obstacle is in frame then the whole obstacle
                        # is added
                        break
        return obstacles_in_frame

    def check_frame(self):
        if self.frame['type'] == 'frame_shift':
            percentage = 80  # if travelled over x% then get new frame

            # if final goal is not in the current frame, compare current distance
            # to the local goal with the initial distance
            if not self.point_in_frame(self.goal_state):
                    init_dist = distance_between_points(self.frame['waypoints'][0], self.frame['endpoint_frame'])
                    curr_dist = distance_between_points(self.curr_state[:2], self.frame['endpoint_frame'])
                    if curr_dist < init_dist*(1-(percentage/100.)):
                        # if already covered 'percentage' of the distance 
                        valid = False
                        return valid
                    else:
                        valid = True
                        return valid
            else:  # keep frame, until 'percentage' of the distance covered or arrived at last frame
                valid = True
                return valid
            
        if frame_type == 'corridor':
            print 'check_frame for corridor not yet implemented'
                #close enough/in next corridor?        

    def update_frame(self):
        
        # Update global path from current position,
        # since there may be a deviation from original global path
        # and since you moved over the path so a part needs to be removed.
        
        if self.problem_options['fixed_example']:
            #================================================================
            print 'For now global path is not yet updated, just shrinked'
            # for now remove a part from the path, keeping the same waypoints
            # find waypoint closest to current position, remove the rest

            # initialize distance to the maximum possible value
            dist = max(self.environment.room['shape'].width, self.environment.room['shape'].height)
            index = 0
            for idx, waypoint in enumerate(self.global_path):
                curr_dist = distance_between_points(waypoint, self.curr_state)
                if curr_dist < dist:
                    dist = curr_dist
                    index = idx
            self.global_path = self.global_path[index:]
            #================================================================
        else:
            self.global_path = self.global_planner.get_path(start=self.curr_state, goal=self.goal_state)
            self.global_path.append(self.goal_state)
        # make new frame
        self.create_frame(frame_size = self.frame_size)

        # get initial guess based on global path, get motion time
        self.init_guess, self.motion_time, _ = self.get_init_guess()

        # get moving obstacles inside frame for this time
        self.frame['moving_obstacles'] = self.get_moving_obstacles_in_frame()

    def get_init_guess(self, **kwargs):

        waypoints = self.frame['waypoints']
        # change first waypoint to current state, the startpoint of the init guess
        waypoints[0] = [self.curr_state[0], self.curr_state[1]]
        # use waypoints for prediction
        x, y = [], []
        for waypoint in waypoints:
            x.append(waypoint[0])
            y.append(waypoint[1])
        # Calculate total length in x- and y-direction
        l_x, l_y = 0., 0.
        for i in range(len(waypoints)-1):
            l_x += waypoints[i+1][0] - waypoints[i][0]
            l_y += waypoints[i+1][1] - waypoints[i][1]
        # Calculate distance in x and y between each 2 waypoints and use it as a relative measure to build time vector
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

        # Make interpolation functions
        fx = interp1d(time_x, x, kind='linear', bounds_error=False, fill_value=1.)  # kind='cubic' requires a minimum of 4 waypoints
        fy = interp1d(time_y, y, kind='linear', bounds_error=False, fill_value=1.)

        # Evaluate resulting splines to get evaluations at knots = coeffs-guess
        # Todo: this doesn't take into account the conservatism: spline value is taken instead of the knots
        #       what is the effect of this?
        coeffs_x = fx(self.vehicles[0].knots[self.vehicles[0].degree-1:-(self.vehicles[0].degree-1)])
        coeffs_y = fy(self.vehicles[0].knots[self.vehicles[0].degree-1:-(self.vehicles[0].degree-1)])
        init_guess = np.c_[coeffs_x, coeffs_y]

        # Todo: suppose moving at half of vmax, then you don't need to solve another problem
        # Todo: or solve problem once, but this requires building a problem
        
        # Pass on initial guess
        # Todo: make more general, this only works for Holonomic vehicle
        self.vehicles[0].set_init_spline_value(init_guess)

        # Set start and goal
        if hasattr (self.vehicles[0], 'signals'):
            # use current vehicle velocity as starting velocity for next frame
            self.vehicles[0].set_initial_conditions(waypoints[0], input=self.vehicles[0].signals['input'][:,-1])
        else: 
            self.vehicles[0].set_initial_conditions(waypoints[0])
        self.vehicles[0].set_terminal_conditions(waypoints[-1])

        # Solve one time
        environment = Environment(room={'shape': self.frame['border']['shape'],
                                        'position': self.frame['border']['position'], 'orientation': self.frame['border']['orientation']})
        for obstacle in self.frame['stationary_obstacles']:
            environment.add_obstacle(obstacle)

        problem = Point2point(self.vehicles[0], environment, freeT=self.problem_options['freeT'])
        problem.init()
        problem.solve(current_time=0, update_time=0.1)

        if problem.problem.stats()['return_status'] != 'Solve_Succeeded':
            feasible = False
            print 'Problem was infeasible, did not find an initial guess'
        else:
            feasible = True

        # Retreive motion time
        if self.options['freeT']:
            # freeT: there is a time variable
            motion_time = problem.father.get_variables(problem, 'T',)[0][0]
        else:
            # fixedT: the remaining motion time is always the horizon time
            motion_time = problem.options['horizon_time']
        # Save trajectories for all frames, as initial guesses
        splines = problem.father.get_variables()[self.vehicles[0].label, 'splines0']
        splines = np.array(splines)  # convert DM to array
        return splines, motion_time, feasible

    def make_border(self, xmin, ymin, xmax, ymax):
        width = xmax - xmin
        height = ymax - ymin
        angle = 0.
        # add angle to position
        position = [(xmax - xmin)*0.5+xmin,(ymax-ymin)*0.5+ymin, angle]
        limits = [xmin, ymin, xmax, ymax]
        return {'shape': Rectangle(width=width, height=height),
         'position': position, 'orientation': angle, 'limits': limits}

    def get_moving_obstacles_in_frame(self):
        # determine which moving obstacles are in self.frame for self.motion_time
        moving_obstacles = []
        for obstacle in self.environment.obstacles:
            # check if obstacle is stationary, this is when:
            # there is no entry trajectories, there are trajectories but no velocity,
            # all velocities are 0.
            if (('trajectories' in obstacle.simulation) and ('velocity' in obstacle.simulation['trajectories'])
               and not (all(vel == [0.]*obstacle.n_dim for vel in obstacle.simulation['trajectories']['velocity']['values']))):
                # get obstacle checkpoints
                obs_chck = obstacle.shape.get_checkpoints()[0]  # element [0] gives vertices, not the corresponding radii
                obs_pos = obstacle.signals['position'][:,-1]
                obs_vel = obstacle.signals['velocity'][:,-1]
                # add all moving obstacles, but only avoid those that matter
                moving_obstacles.append(obstacle)
                # by default don't avoid the obstacle, only if it inside the frame
                obstacle.set_options({'avoid': False})
                # only check points for which the obstacle velocity is not all zero
                if any(obs_vel != np.zeros(obstacle.n_dim)):
                    for chck in obs_chck:
                        # if it is not a circle, rotate the vertices
                        if hasattr(obstacle.shape, 'orientation'):
                            vertex = obstacle.shape.rotate(obstacle.shape.orientation, chck)
                        # move to correct position
                        vertex += obs_pos
                        # check if vertex is in frame during movement
                        if self.point_in_frame(vertex, time=self.motion_time, velocity=obs_vel):
                            # avoid corresponding obstacle                            
                            obstacle.set_options({'avoid': True})
                            # break from for chck in obs_chck, move on to next obstacle, since
                            # obstacle is added to the frame if any of its vertices is in the frame
                            break
        return moving_obstacles

    def generate_problem(self):
        environment = Environment(room={'shape': self.frame['border']['shape'], 'position': self.frame['border']['position'], 'draw':True})
        for obstacle in self.frame['stationary_obstacles'] + self.frame['moving_obstacles']:
            environment.add_obstacle(obstacle)
        # create a point-to-point problem
        problem = Point2point(self.vehicles, environment, freeT=self.problem_options['freeT'])
        problem.set_options(self.options['solver_options'])
        if 'horizon_time' in self.problem_options:
            problem.set_options({'horizon_time' : self.problem_options['horizon_time']})
        problem.init() 
        problem.initialize(current_time=0.)  # reset the current_time, to ensure that predict uses the provided
                                             # last input of previous problem and vehicle velocity is kept from one frame to another
        return problem