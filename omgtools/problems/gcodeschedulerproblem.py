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
from gcodeproblem import GCodeProblem
from ..basics.shape import Rectangle, Square, Circle
from ..environment.environment import Environment
from ..basics.shape import Rectangle, Ring
from ..basics.geometry import distance_between_points, point_in_polyhedron
from ..basics.spline import BSplineBasis, BSpline
from ..basics.spline_extra import concat_splines, running_integral

from scipy.interpolate import interp1d
import scipy.linalg as la
import numpy as np
import pickle
import time
import warnings

class GCodeSchedulerProblem(Problem):

    def __init__(self, tool, GCode, options=None, **kwargs):
        options = options or {}
        # split large circle segments in multiple segments
        self.split_circle = kwargs['split_circle'] if 'split_circle' in kwargs else False
        self.variable_tolerance = kwargs['variable_tolerance'] if 'variable_tolerance' in kwargs else False
        environment = self.get_environment(GCode, tool)
        # pass on environment and tool to Problem constructor,
        # generates self.vehicles and self.environment
        # self.vehicles[0] = tool
        Problem.__init__(self, tool, environment, options, label='schedulerproblem')
        self.n_current_block = 0  # number of the block that the tool will follow next/now
        self.curr_state = self.vehicles[0].prediction['state'] # initial vehicle position
        self.goal_state = self.vehicles[0].poseT # overall goal
        self.problem_options = options  # e.g. selection of problem type (freeT, fixedT)
        self.problem_options['freeT'] = True  # only this one is available
        self.start_time = 0.
        self.update_times=[]
        self.motion_time_log = []  # save the required motion times
        self.n_segments = kwargs['n_segments'] if 'n_segments' in kwargs else 1  # amount of segments to combine
        self._n_segments = self.n_segments  # save original value (for plotting)
        self.segments = []
        # are we running over the GCode using the deployer
        self.with_deployer = kwargs['with_deployer'] if 'with_deployer' in kwargs else False

        if not isinstance(self.vehicles[0].shapes[0], Circle):
            raise RuntimeError('Vehicle shape can only be a Circle when solving a GCodeSchedulerProblem')

    def init(self):
        # otherwise the init of Problem is called, which is not desirable
        pass

    def initialize(self, current_time):
        self.local_problem.initialize(current_time)

    def reinitialize(self):
        # this function is called at the start and creates the first local problem

        self.segments = []
        # select the next blocks of GCode that will be handled
        # if less than self.n_segments are left, only the remaining blocks
        # will be selected
        self.segments = self.environment.room[
                           self.n_current_block:self.n_current_block+self.n_segments]
        # if there is only one segment, save the next one to check when the tool enters the next segment
        if self.n_segments == 1:
            if len(self.environment.room) > 1:
                self.next_segment = self.environment.room[self.n_current_block+1]

        # total number of considered segments in the provided GCode
        self.cnt = len(self.environment.room)-1

        # get initial guess for trajectories (based on central line) and motion times, for all segments
        init_guess, self.motion_times = self.get_init_guess()

        # get a problem representation of the combination of segments
        # the gcodeschedulerproblem (self) has a local_problem (gcodeproblem) at each moment
        self.local_problem = self.generate_problem()
        # Todo: is this function doing what we want?
        self.local_problem.reset_init_guess(init_guess)

    def solve(self, current_time, update_time):
        # solve the local problem with a receding horizon,
        # and update segments if necessary

        # update current state
        if not hasattr(self.vehicles[0], 'signals'):
            # first iteration
            self.curr_state = self.vehicles[0].prediction['state']
        else:
            # all other iterations
            self.curr_state = self.vehicles[0].signals['state'][:,-1]

        # did we move far enough over the current segment yet?
        print self.n_current_block
        segments_valid = self.check_segments()
        if not segments_valid:
            # add new segment and remove first one
            if hasattr(self, 'no_update') and self.no_update:
                # don't update number or segments, because the deployer wants to
                # re-compute the same segment, that e.g. was infeasible
                # this boolean is set by the deployer in deployer.update_segment()
                # self.local_problem = self.generate_problem()
                pass
            else:
                self.n_current_block += 1
                self.update_segments()

                # transform segments into local_problem
                self.local_problem = self.generate_problem()
                # self.init_guess is filled in by update_segments()
                # this also updates self.motion_time
                self.local_problem.reset_init_guess(self.init_guess)

        # solve local problem
        self.local_problem.solve(current_time, update_time)

        # update motion time variables (remaining time)
        for k in range(self.n_segments):
            self.motion_times[k] = self.local_problem.father.get_variables(
                                   self.local_problem, 'T'+str(k),)[0][0]
        # save motion time for current segment
        self.motion_time_log.append(self.motion_times[0])

        # save solving time
        self.update_times.append(self.local_problem.update_times[-1])

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def store(self, current_time, update_time, sample_time):
        # call store of local problem
        self.local_problem.store(current_time, update_time, sample_time)

    def simulate(self, current_time, simulation_time, sample_time):
        # save segment
        # store trajectories
        if not hasattr(self, 'segment_storage'):
            self.segment_storage = []

        # normally simulate one segment at a time
        # simulation_time = self.motion_times[0]
        # simulate in receding horizon with small steps
        # if simulation_time == np.inf:  # when calling run_once
        #     simulation_time = sum(self.motion_times)
        repeat = int(simulation_time/sample_time)

        # copy segments, to avoid problems when removing elements from self.segments
        segments_to_save = self.segments[:]
        for k in range(repeat):
            self._add_to_memory(self.segment_storage, segments_to_save)

        # simulate the multiframe problem
        Problem.simulate(self, current_time, simulation_time, sample_time)

    def _add_to_memory(self, memory, data_to_add, repeat=1):
        memory.extend([data_to_add for k in range(repeat)])

    def stop_criterium(self, current_time, update_time):
        # check if the current segment is the last one
        if self.segments[0]['end'] == self.goal_state:
            # if we now reach the goal, the tool has arrived
            if self.local_problem.stop_criterium(current_time, update_time):
                return True
        else:
            return False

    def final(self):
        print 'The tool has reached its goal!'
        print self.cnt, ' GCode commands were executed.'
        print 'Total machining time when considering standstill-standstill segments: ', np.round(self.get_init_guess_total_motion_time(),3), ' s'
        print 'Total machining time for computed trajectories: ', np.round(sum(self.motion_time_log),3), ' s'
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
            for k in range(self._n_segments):
                # initialize segment plot, always use segments[0]
                pose_2d = self.segments[0]['pose'][:2] + [0.]  # shape was already rotated
                # Todo: generalize to 3d later
                s, l = self.segments[0]['shape'].draw(pose_2d)
                surfaces = [{'facecolor': 'none', 'edgecolor': 'red', 'linestyle' : '--', 'linewidth': 1.2} for _ in s]
                info[0][0]['surfaces'] += surfaces
                # initialize global path plot
                info[0][0]['lines'] += [{'color': 'red', 'linestyle' : '--', 'linewidth': 1.2}]
        return info

    def update_plot(self, argument, t, **kwargs):
        # plot environment
        data = Problem.update_plot(self, argument, t)
        if data is not None:
            for k in range(len(self.segment_storage[t])):
                # for every frame at this point in time
                # plot frame border
                # Todo: generalize to 3d later
                pose_2d = self.segment_storage[t][k]['pose'][:2] + [0.]  # shape was already rotated
                s, l = self.segment_storage[t][k]['shape'].draw(pose_2d)
                data[0][0]['surfaces'] += s
        return data

    # ========================================================================
    # GCodeSchedulerProblem specific functions
    # ========================================================================

    def get_environment(self, GCode, tool):
        # convert the list of GCode blocks into an environment object
        # each GCode block is represented as a room in which the trajectory
        # has to stay

        number = 0  # each room has a number
        room = []
        tolerance = tool.tolerance

        for block in GCode:
            # convert block to room
            if block.type in ['G00', 'G01']:
                if not self.variable_tolerance:
                    # add tolerance to width to obtain the complete reachable region
                    width = distance_between_points(block.start, block.end) + 2*tolerance
                    height = 2*tolerance
                    orientation = np.arctan2(block.end[1]-block.start[1], block.end[0]-block.start[0])
                    shape = Rectangle(width = width,  height = height, orientation = orientation)
                    pose = [block.start[0] + (block.end[0]-block.start[0])*0.5,
                            block.start[1] + (block.end[1]-block.start[1])*0.5,
                            block.start[2] + (block.end[2]-block.start[2])*0.5,
                            orientation,0.,0.]
                    # Todo: for now orientation is only taken into account as if it were a 2D segment
                    new_room = [{'shape': shape, 'pose': pose, 'position': pose[:2], 'draw':True,
                                'start': block.start, 'end': block.end, 'number':number}]
                else:
                    # divide segment in three parts, with variable tolerance:
                    # large tolerances in the first and last parts, tight tolerance in the middle part

                    # Todo: for now always divided in 0.1,0.8,0.1*length and 0.2*tol and 2*tol -->
                    # make it a parameter?

                    orientation = np.arctan2(block.end[1]-block.start[1], block.end[0]-block.start[0])
                    width = distance_between_points(block.start, block.end)  # default width

                    ## part 1
                    l1 = 0.1*width
                    width1 = l1 + 2*tolerance
                    height1 = 2*tolerance
                    shape1 = Rectangle(width = width1,  height = height1, orientation = orientation)
                    pose1 = [block.start[0] + 0.5*l1*np.cos(orientation),
                             block.start[1] + 0.5*l1*np.sin(orientation),
                             block.start[2] + (block.end[2]-block.start[2])*0.5,
                             orientation,0.,0.]
                    end1 = [block.start[0] + l1*np.cos(orientation),
                            block.start[1] + l1*np.sin(orientation),
                            block.start[2] + (block.end[2]-block.start[2])]

                    ## part 2, tolerance = 10 times tighter
                    l2 = 0.8*width
                    width2 = l2 + 0.2*tolerance
                    height2 = 0.2*tolerance
                    shape2 = Rectangle(width = width2,  height = height2, orientation = orientation)
                    pose2 = [block.start[0] + (l1+0.5*l2)*np.cos(orientation),
                             block.start[1] + (l1+0.5*l2)*np.sin(orientation),
                             block.start[2] + (block.end[2]-block.start[2])*0.5,
                             orientation,0.,0.]
                    start2 = end1
                    end2 = [block.start[0] + (l1+l2)*np.cos(orientation),
                            block.start[1] + (l1+l2)*np.sin(orientation),
                            block.start[2] + (block.end[2]-block.start[2])]

                    ## part 3
                    l3 = l1
                    width3 = width1
                    height3 = height1
                    shape3 = Rectangle(width = width3,  height = height3, orientation = orientation)
                    pose3 = [block.end[0] - 0.5*l3*np.cos(orientation),
                             block.end[1] - 0.5*l3*np.sin(orientation),
                             block.start[2] + (block.end[2]-block.start[2])*0.5,
                             orientation,0.,0.]
                    start3 = end2

                    new_room = [{'shape': shape1, 'pose': pose1, 'position': pose1[:2], 'draw':True,
                                'start': block.start, 'end': end1, 'number':number},
                                {'shape': shape2, 'pose': pose2, 'position': pose2[:2], 'draw':True,
                                'start': start2 , 'end': end2, 'number':number+1, 'tolerance': 0.1*tolerance},
                                {'shape': shape3, 'pose': pose3, 'position': pose3[:2], 'draw':True,
                                'start': start3, 'end': block.end, 'number':number+2}]

            elif block.type in ['G02', 'G03']:
                radius_in = block.radius - tolerance
                radius_out = block.radius + tolerance
                # move to origin
                start = np.array(block.start) - np.array(block.center)
                end = np.array(block.end) - np.array(block.center)

                # adapt start and end to include tolerance, i.e. make ring a little wider, such that
                # perpendicular distance from start (and end) to border of ring = tolerance
                theta = np.arctan2(tolerance,((radius_in+radius_out)*0.5))  # angle over which to rotate
                # provide two turning directions
                R1 = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])  # rotation matrix
                R2 = np.array([[np.cos(-theta), -np.sin(-theta)],[np.sin(-theta), np.cos(-theta)]])  # rotation matrix

                # Todo: rotation only works for 2D XY arcs for now

                if block.type == 'G02':
                    direction = 'CW'
                    start[:2] = np.dot(R1, start[:2])  # slightly rotated start point
                    end[:2] = np.dot(R2, end[:2])  # slightly rotated end point
                else:
                    direction = 'CCW'
                    start[:2] = np.dot(R2, start[:2])  # slightly rotated start point
                    end[:2] = np.dot(R1, end[:2])  # slightly rotated end point

                # split ring segments of more than 135 degrees in two equal parts
                # use extended version of the ring
                angle1 = np.arctan2(start[1], start[0])
                angle2 = np.arctan2(end[1], end[0])
                if block.type == 'G02':
                    if angle1 < angle2:
                        # clockwise so angle2 must be < angle1
                        # probably angle2 is smaller, but arctan2 returned a negative angle
                        angle1 += 2*np.pi
                    arc_angle = angle1 - angle2
                elif block.type == 'G03':
                    if angle1 > angle2:
                        # counter-clockwise so angle2 must be > angle1
                        # probably angle2 is bigger, but arctan2 returned a negative angle
                        angle2 += 2*np.pi
                    arc_angle = angle2 - angle1
                else:
                    raise RuntimeError('Invalid block type: ', block.type)
                new_room = self.split_ring_segment(block, arc_angle, start, end, radius_in, radius_out, direction, tolerance, number)

                if self.variable_tolerance:
                    divided_rooms = []
                    for r in new_room:
                        # following parameters are the same for all segments
                        pose = r['pose']
                        radius_in = r['shape'].radius_in
                        radius_out = r['shape'].radius_out
                        direction = r['shape'].direction

                        # arc angle of ring, without including tolerance
                        # must be positive, sign of angle is determined by CW or CCW

                        start = np.array(r['start'])-np.array(r['pose'][:3])
                        end = np.array(r['end'])-np.array(r['pose'][:3])
                        angle1 = np.arctan2(start[1], start[0])
                        angle2 = np.arctan2(end[1], end[0])

                        if direction == 'CW':
                            if angle1 < angle2:
                                # clockwise so angle2 must be < angle1
                                # probably angle2 is smaller, but arctan2 returned a negative angle
                                angle1 += 2*np.pi
                            arc_angle = angle1 - angle2
                        else:
                            if angle1 > angle2:
                                # counter-clockwise so angle2 must be > angle1
                                # probably angle2 is bigger, but arctan2 returned a negative angle
                                angle2 += 2*np.pi
                            arc_angle = angle2 - angle1
                        arc_angle = np.abs(arc_angle)
                        # arc_angle = np.abs(r['shape'].end_angle - r['shape'].start_angle)

                        arc1 = 0.1*arc_angle  # = arc3
                        arc2 = 0.8*arc_angle

                        ## part 1
                        # adapt start to include tolerance, i.e. make ring start a little earlier, such that
                        # perpendicular distance from start to border of ring = tolerance
                        seg_start1 = np.array(r['start'])-np.array(r['pose'][:3])  # without including tolerance = start of segment
                        start1 = np.array(seg_start1)  # with including tolerance = start of shape
                        end1 = np.array(seg_start1)  # initialize
                        # angle over which to rotate to account for large tolerance
                        delta_arc_big = np.arctan2(tolerance,((radius_in+radius_out)*0.5))
                        if direction == 'CW':
                            R_delta = np.array([[np.cos(delta_arc_big), -np.sin(delta_arc_big)],[np.sin(delta_arc_big), np.cos(delta_arc_big)]])
                            start1[:2] = np.dot(R_delta, seg_start1[:2])  # slightly rotated start point

                            theta = arc1 + delta_arc_big
                            R = np.array([[np.cos(-theta), -np.sin(-theta)],[np.sin(-theta), np.cos(-theta)]])
                            end1[:2] = np.dot(R, seg_start1[:2])  # rotate start point to end + a bit further
                        else:
                            R_delta = np.array([[np.cos(-delta_arc_big), -np.sin(-delta_arc_big)],[np.sin(-delta_arc_big), np.cos(-delta_arc_big)]])
                            start1[:2] = np.dot(R_delta, seg_start1[:2])  # slightly rotated start point

                            theta = arc1 + delta_arc_big
                            R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
                            end1[:2] = np.dot(R, seg_start1[:2])  # rotate start point to end

                        shape1 = Ring(radius_in = radius_in, radius_out = radius_out,
                                      start = start1, end = end1, direction = direction)

                        seg_end1 = np.array(seg_start1)  # initialize
                        if direction == 'CW':
                            R = np.array([[np.cos(-arc1), -np.sin(-arc1)],[np.sin(-arc1), np.cos(-arc1)]])
                            seg_end1[:2] = np.dot(R, seg_start1[:2])  # slightly rotated start point
                        else:
                            R = np.array([[np.cos(arc1), -np.sin(arc1)],[np.sin(arc1), np.cos(arc1)]])
                            seg_end1[:2] = np.dot(R, seg_start1[:2])  # slightly rotated start point

                        ## part 2, tolerance = 10 times tighter

                        # adapt start to include tolerance, i.e. make ring start a little earlier, such that
                        # perpendicular distance from start to border of ring = tolerance
                        seg_start2 = np.array(seg_end1)
                        start2 = np.array(seg_end1)
                        end2 = np.array(seg_end1)  # initialize
                        # angle over which to rotate to account for large tolerance
                        delta_arc_small = np.arctan2(0.1*tolerance,((radius_in+radius_out)*0.5))
                        if direction == 'CW':
                            R_delta = np.array([[np.cos(delta_arc_small), -np.sin(delta_arc_small)],[np.sin(delta_arc_small), np.cos(delta_arc_small)]])
                            start2[:2] = np.dot(R_delta, seg_start2[:2])  # slightly rotated start point

                            theta = arc2 + delta_arc_small
                            R = np.array([[np.cos(-theta), -np.sin(-theta)],[np.sin(-theta), np.cos(-theta)]])
                            end2[:2] = np.dot(R, seg_start2[:2])  # rotate start point to end
                        else:
                            R_delta = np.array([[np.cos(-delta_arc_small), -np.sin(-delta_arc_small)],[np.sin(-delta_arc_small), np.cos(-delta_arc_small)]])
                            start2[:2] = np.dot(R_delta, seg_start2[:2])  # slightly rotated start point

                            theta = arc2 + delta_arc_small
                            R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
                            end2[:2] = np.dot(R, seg_start2[:2])  # rotate start point to end

                        shape2 = Ring(radius_in = radius_in + 0.9*tolerance, radius_out = radius_out-0.9*tolerance,
                                      start = start2, end = end2, direction = direction)

                        seg_end2 = np.array(seg_start2)  # initialize
                        if direction == 'CW':
                            R = np.array([[np.cos(-arc2), -np.sin(-arc2)],[np.sin(-arc2), np.cos(-arc2)]])
                            seg_end2[:2] = np.dot(R, seg_start2[:2])  # slightly rotated start point
                        else:
                            R = np.array([[np.cos(arc2), -np.sin(arc2)],[np.sin(arc2), np.cos(arc2)]])
                            seg_end2[:2] = np.dot(R, seg_start2[:2])  # slightly rotated start point

                        ## part 3
                        # adapt start to include tolerance, i.e. make ring start a little earlier, such that
                        # perpendicular distance from start to border of ring = tolerance
                        seg_start3 = np.array(seg_end2)
                        start3 = np.array(seg_start3)
                        end3 = np.array(seg_start3)  # initialize
                        # angle over which to rotate to account for large tolerance
                        delta_arc_big = np.arctan2(tolerance,((radius_in+radius_out)*0.5))
                        if direction == 'CW':
                            R_delta = np.array([[np.cos(delta_arc_big), -np.sin(delta_arc_big)],[np.sin(delta_arc_big), np.cos(delta_arc_big)]])
                            start3[:2] = np.dot(R_delta, seg_start3[:2])  # slightly rotated start point

                            theta = arc1 + delta_arc_big
                            R = np.array([[np.cos(-theta), -np.sin(-theta)],[np.sin(-theta), np.cos(-theta)]])
                            end3[:2] = np.dot(R, seg_start3[:2])  # rotate start point to end
                        else:
                            R_delta = np.array([[np.cos(-delta_arc_big), -np.sin(-delta_arc_big)],[np.sin(-delta_arc_big), np.cos(-delta_arc_big)]])
                            start3[:2] = np.dot(R_delta, seg_start3[:2])  # slightly rotated start point

                            theta = arc1 + delta_arc_big
                            R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
                            end3[:2] = np.dot(R, seg_start3[:2])  # rotate start point to end

                        shape3 = Ring(radius_in = radius_in, radius_out = radius_out,
                                      start = start3, end = end3, direction = direction)

                        # start and end of ring shape is for a shape centered in the origin,
                        # room start and end are shifted away from origin
                        seg_end1 = (seg_end1 + np.array(r['pose'][:3])).tolist()  # move from origin to real position
                        seg_start2 = seg_end1
                        seg_end2 = (seg_end2 + np.array(r['pose'][:3])).tolist()
                        seg_start3 = seg_end2

                        # bla = False some segment is added double at the top?
                        if bla:
                            divided_rooms.extend([
                                            {'shape': shape2, 'pose': pose, 'position': pose[:2], 'draw':True,
                                            'start': seg_start2 , 'end': seg_end2, 'number':number+1},
                                            {'shape': shape3, 'pose': pose, 'position': pose[:2], 'draw':True,
                                            'start': seg_start3, 'end': block.end, 'number':number+2}])
                        else:
                            divided_rooms.extend([
                                            {'shape': shape1, 'pose': pose, 'position': pose[:2], 'draw':True,
                                            'start': block.start, 'end': seg_end1, 'number':number},
                                            {'shape': shape2, 'pose': pose, 'position': pose[:2], 'draw':True,
                                            'start': seg_start2 , 'end': seg_end2, 'number':number+1},
                                            {'shape': shape3, 'pose': pose, 'position': pose[:2], 'draw':True,
                                            'start': seg_start3, 'end': block.end, 'number':number+2}])
                    # assign divided rooms to old variable
                    new_room = divided_rooms
            # save original GCode block in the room description
            for r in new_room:
                room.append(r)
                number += 1
        return Environment(room=room)

    def split_ring_segment(self, block, arc_angle, start, end, radius_in, radius_out, direction, tolerance, number):
        if (self.split_circle and arc_angle > 3*np.pi/4):
            # compute middle of ring segment
            arc = arc_angle*0.5
            # adapt start and end to include tolerance, i.e. make ring a little wider, such that
            # perpendicular distance from start (and end) to border of ring = tolerance
            theta = np.arctan2(tolerance,((radius_in+radius_out)*0.5))  # angle over which to rotate
            mid1 = np.array(start)  # use np.array() to get a copy of the object
            mid2 = np.array(start)  # mid of second part of the segment
            if block.type == 'G02':
                R1 = np.array([[np.cos(-arc-theta), -np.sin(-arc-theta)],[np.sin(-arc-theta), np.cos(-arc-theta)]])  # rotation matrix
                R2 = np.array([[np.cos(-arc+theta), -np.sin(-arc+theta)],[np.sin(-arc+theta), np.cos(-arc+theta)]])  # rotation matrix
                # create overlap region between the two new segments
                mid1[:2] = np.dot(R1, mid1[:2])  # rotate start point over half arc, and a bit further
                mid2[:2] = np.dot(R2, mid2[:2])  # rotate start point over half arc, a bit less far
            else:
                R1 = np.array([[np.cos(arc+theta), -np.sin(arc+theta)],[np.sin(arc+theta), np.cos(arc+theta)]])  # rotation matrix
                R2 = np.array([[np.cos(arc-theta), -np.sin(arc-theta)],[np.sin(arc-theta), np.cos(arc-theta)]])  # rotation matrix
                # create overlap region between the two new segments
                mid1[:2] = np.dot(R1, mid1[:2])  # rotate start point over half arc, and a bit further
                mid2[:2] = np.dot(R2, mid2[:2])  # rotate start point over half arc, a bit less far
            # segment1
            start1 = np.array(start)  # keep start of segment1
            end1 = mid1
            # segment2
            start2 = mid2
            end2 = np.array(end)  # keep end of segment2

            # shape is located in the origin
            shape1 = Ring(radius_in = radius_in, radius_out = radius_out,
                         start = start1, end = end1, direction = direction)
            shape2 = Ring(radius_in = radius_in, radius_out = radius_out,
                         start = start2, end = end2, direction = direction)
            pose = list(block.center)
            pose.extend([0.,0.,0.])  # [x,y,z,orientation], ring always has orientation 0
            # room start and end is shifted away from origin
            mid1_shift = list(mid1 + np.array(block.center))  # move from origin to real position
            mid2_shift = list(mid2 + np.array(block.center))
            new_room = [{'shape': shape1, 'pose': pose, 'position': pose[:2], 'draw':True,
                         'start': block.start, 'end': mid1_shift, 'number':number},
                        {'shape': shape2, 'pose': pose, 'position': pose[:2], 'draw':True,
                         'start': mid2_shift, 'end': block.end, 'number':number+1}]
        else:
            shape = Ring(radius_in = radius_in, radius_out = radius_out,
                         start = start, end = end, direction = direction)
            pose = block.center
            pose.extend([0.,0.,0.])  # [x,y,z,orientation], ring always has orientation 0
            new_room = [{'shape': shape, 'pose': pose, 'position': pose[:2], 'draw':True,
                        'start': block.start, 'end': block.end, 'number':number}]
        return new_room

    def check_segments(self):

        # check if the tool still has to move over the first element of
        # self.segments, if so this means no movement is made in this iteration yet
        # if tool has already moved (i.e. the tool position is inside the overlap region
        # between the two segments), we will add an extra segment and drop the first one

        # if final goal is not on the current segment, check if current state overlaps with the next segment
        if (self.segments[0]['end'] == self.goal_state and
            self.segments[0]['start'] == self.environment.room[-1]['start']):
            # this is the last segment, keep it until arrival
            valid = True
            return valid
        else:
            if (self.n_segments == 1 and hasattr(self, 'next_segment')):
                if self.point_in_extended_shape(self.next_segment, self.curr_state[:2], distance=self.vehicles[0].shapes[0].radius):
                    # if point in extended shape of next segment (=complete ring, or segment with infinite length),
                    # we can move to this next segment
                    # only called if self.n_segments = 1,
                    # then self.segments[1] doesn't exist and self.next_segment does exist
                    valid = False
                else:
                    valid = True
                return valid
            elif self.point_in_extended_shape(self.segments[1], self.curr_state[:2], distance=self.vehicles[0].shapes[0].radius):
                # if point in extended shape of next segment (=complete ring, or segment with infinite length),
                # we can move to this next segment
                valid = False
                return valid
            else:
                valid = True
                return valid

        if (np.array(self.curr_state) == np.array(self.segments[0]['end'])).all():
            # current state is equal to end of segment 0
            return False
        else:
            # current state is not yet equal to the end of segment 0
            return True

    def update_segments(self):

        # update the considered segments: remove first one, and add a new one

        self.segments = self.segments[1:]  # drop first segment
        if self.segments[-1]['number'] < self.cnt:
            # last segment is not yet in self.segments, so there are some segments left,
            # create segment for next block
            # self.n_segments -= 1
            new_segment = self.environment.room[self.n_current_block+(self.n_segments-1)]
            self.segments.append(new_segment)  # add next segment

            if self.n_segments == 1:
                self.next_segment = self.environment.room[self.n_current_block+1]
        else:
            # all segments are currently in self.segments, don't add a new one
            # and lower the amount of segments that are combined
            self.n_segments -= 1

        # self.get_init_guess() uses previous solution to get an initial guess for
        # all segments except the last one,
        # for this one get initial guess based on the center line
        # analogously for the motion_times
        self.init_guess, self.motion_times = self.get_init_guess()

    def point_in_segment(self, segment, point, distance=0):
        # check if the provided point is inside segment
        # distance is the margin to take into account (due to the tool size)

        # for the check, re-use the collision avoidance constraints of tool.py

        if (isinstance(segment['shape'], (Rectangle, Square))):
            # we have a diagonal line segment
            if point_in_polyhedron(point, segment['shape'], segment['position'], margin=distance):
                return True
            else:
                return False

        elif (isinstance(segment['shape'], (Ring))):
            # we have a ring/circle segment

            # use polar coordinates to go from point(x,y) to point(r,theta)
            # then check if r and theta are inside the ring

            center = segment['pose']
            angle1 = np.arctan2(point[1] - center[1], point[0] - center[0])
            angle2 = angle1 + 2*np.pi
            r = np.sqrt((point[0]-center[0])**2+(point[1]-center[1])**2)

            if (r >= segment['shape'].radius_in+distance and r <= segment['shape'].radius_out-distance):
                # Todo: shift start and end_angle according to distance (i.e. make ring a little smaller) to
                # account for the tolerance (tool point may not lie infinitely close to the border)
                if segment['shape'].direction == 'CW':
                    if (angle1 <= segment['shape'].start_angle and angle1 >= segment['shape'].end_angle):
                        return True
                    if (angle2 <= segment['shape'].start_angle and angle2 >= segment['shape'].end_angle):
                        return True
                elif segment['shape'].direction == 'CCW':
                    if (angle1 >= segment['shape'].start_angle and angle1 <= segment['shape'].end_angle):
                        return True
                    if (angle2 >= segment['shape'].start_angle and angle2 <= segment['shape'].end_angle):
                        return True
                return False
            else:
                return False

    def point_in_extended_shape(self, segment, point, distance=0):
        # check if the provided point is inside the extended/infinite version of the shape, meaning
        # that we check if the point is in the complete ring (instead of in the ring segment), or if
        # the point is inside the rectangle with infinite width (meaning that it is inside the GCode segment
        # with infinite length)
        # this is to check if the current state (probably = the connection point between spline segments),
        # is valid to continue to the next segment (= the segment provided to this function)

        # difference with point_in_segment: checks if point is in the finite/normal version of the shape

        # distance is the margin to take into account (due to the tool size)

        if (isinstance(segment['shape'], (Rectangle, Square))):
            if (segment['shape'].orientation%(np.pi) == 0):
                # horizontal line segment
                if (point[1] < max(segment['shape'].vertices[1,:]+segment['position'][1]) and
                    point[1] > min(segment['shape'].vertices[1,:]+segment['position'][1])):
                    return True
                else:
                    return False
            elif (segment['shape'].orientation%(np.pi/2.) == 0):
                # vertical line segment
                # note: also a shape with orientation 0 would pass this test, but this was
                # already captured in first if-test
                if (point[0] < max(segment['shape'].vertices[0,:]+segment['position'][0]) and
                    point[0] > min(segment['shape'].vertices[0,:]+segment['position'][0])):
                    return True
                else:
                    return False
            else:
                # we have a diagonal line GCode segment
                # find the lines of the rectangle representing the line GCode segment with tolerances,
                # that have the length of the segment length
                couples = []
                for k in range(len(segment['shape'].vertices[0])-1):
                    point1 = segment['shape'].vertices[:,k]+segment['position']
                    point2 = segment['shape'].vertices[:,k+1]+segment['position']
                    dist = distance_between_points(point1,point2)
                    if abs(dist - segment['shape'].width) < 1e-3:
                        # the connection between the points gives a side of length = width of the shape
                        couples.append([point1,point2])
                if len(couples) != 2:
                    # not yet found two couples, so the distance between last vertex and first must also be = width
                    couples.append([segment['shape'].vertices[:,-1]+segment['position'],segment['shape'].vertices[:,0]+segment['position']])
                # compute the equations for these two lines, to check if the point is at the right side of them,
                # i.e. inside the rectangle with infinite width = the segment with infinite length
                # note: suppose that the vertices are stored in clockwise order here

                side = []
                for couple in couples:
                    x1, y1 = couple[0]  # point1
                    x2, y2 = couple[1]  # point2
                    vector = [x2-x1, y2-y1]  # vector from point2 to point1
                    a = np.array([-vector[1],vector[0]])*(1/np.sqrt(vector[0]**2+vector[1]**2))  # normal vector
                    b = np.dot(a,np.array([x1,y1]))  # offset
                    side.append(np.dot(a, point) - b)  # fill in point
                if all(s<-distance for s in side):
                    # point is inside the shape and a distance tolerance away from border
                    return True
                else:
                    return False
        elif (isinstance(segment['shape'], (Ring))):
            # we have a ring/circle segment, check if distance from point to center lies between
            # the inner and outer radius

            center = segment['pose']
            r = np.sqrt((point[0]-center[0])**2+(point[1]-center[1])**2)

            if (r >= segment['shape'].radius_in+distance and r <= segment['shape'].radius_out-distance):
                return True
            else:
                return False

    def generate_problem(self):

        local_rooms = self.environment.room[self.n_current_block:self.n_current_block+self.n_segments]
        local_environment = Environment(room=local_rooms)
        problem = GCodeProblem(self.vehicles[0], local_environment, self.n_segments, motion_time_guess=self.motion_times)

        problem.set_options({'solver_options': self.options['solver_options']})
        problem.init()
        # reset the current_time, to ensure that predict uses the provided
        # last input of previous problem and vehicle velocity is kept from one frame to another
        problem.initialize(current_time=0.)
        return problem

    def get_init_guess(self, **kwargs):
        # if first iteration, compute init_guess based on center line (i.e. connection between start and end) for all segments
        # else, use previous solutions to build a new initial guess:
        #   if combining 2 segments: combine splines in segment 1 and 2 to form a new spline in a single segment = new segment1
        #   if combining 3 segments or more: combine segment1 and 2 and keep splines of segment 3 and next as new splines of segment2 and next
        start_time = time.time()

        # initialize variables to hold guesses
        init_splines = []
        motion_times = []

        if hasattr(self, 'guess_coeffs'):
            # guess provided from a previous run
            for k in range(self.n_segments):
                init_splines.append(np.transpose(self.guess_coeffs[self.n_current_block + k]))
                motion_times.append(np.transpose(self.guess_times[self.n_current_block + k]))
        else:
            if hasattr(self, 'local_problem') and hasattr(self.local_problem.father, '_var_result'):
                # local_problem was already solved, re-use previous solutions to form initial guess
                if self.n_segments > 1:
                    # if updating in receding horizon with small steps:
                    # combine first two spline segments into a new spline = guess for new current segment
                    # init_spl, motion_time = self.get_init_guess_combined_segment()

                    # if updating per segment:
                    # the first segment disappears and the guess is given by data of next segment
                    # spline through next segment and its motion time
                    init_splines.append(np.array(self.local_problem.father.get_variables()[self.vehicles[0].label,'splines_seg1']))
                    motion_times.append(self.local_problem.father.get_variables(self.local_problem, 'T1',)[0][0])

                if self.n_segments > 2:
                    # use old solutions for segment 2 until second last segment, these don't change
                    for k in range(2, self.n_segments):
                        # Todo: strange notation required, why not the same as in schedulerproblem.py?
                        init_splines.append(np.array(self.local_problem.father.get_variables()[self.vehicles[0].label,'splines_seg'+str(k)]))
                        motion_times.append(self.local_problem.father.get_variables(self.local_problem, 'T'+str(k),)[0][0])
                # only make guess using center line for last segment
                guess_idx = [self.n_segments-1]
            else:
                # local_problem was not solved yet, make guess using center line for all segments
                guess_idx = range(self.n_segments)

            # make guesses based on center line of GCode
            for k in guess_idx:
                init_spl, motion_time = self.get_init_guess_new_segment(self.segments[k])
                init_splines.append(init_spl)
                motion_times.append(motion_time)

        # pass on initial guess
        self.vehicles[0].set_init_spline_values(init_splines, n_seg = self.n_segments)

        if hasattr (self.vehicles[0], 'signals'):
            # use current vehicle velocity as starting velocity for next frame
            self.vehicles[0].set_initial_conditions(self.curr_state, input = self.vehicles[0].signals['input'][:,-1],
                                                                     dinput = self.vehicles[0].signals['dinput'][:,-1])
        elif self.with_deployer:
            # initial conditions are already set by calling problem.predict()
            # which calls vehicle.predict in deployer.run_segment(),
            # so don't repeat here since this would erase the input and dinput values
            pass
        else:
            self.vehicles[0].set_initial_conditions(self.curr_state)
        self.vehicles[0].set_terminal_conditions(self.segments[-1]['end'])

        end_time = time.time()
        if self.options['verbose'] >= 2:
            print 'elapsed time in get_init_guess ', end_time - start_time

        return init_splines, motion_times

    def get_init_guess_new_segment(self, segment):

        option = 1

        if option == 1:
            # generate a feasible initial guess with a bang-bang jerk profile
            init_guess, motion_time = self.get_init_guess_bangbang_jerk(segment)

            testx = BSpline(self.vehicles[0].basis, init_guess[:,0])
            testy = BSpline(self.vehicles[0].basis, init_guess[:,1])

            dtestx = testx.derivative(1)
            ddtestx = testx.derivative(2)
            dddtestx = testx.derivative(3)
            dtesty = testy.derivative(1)
            ddtesty = testy.derivative(2)
            dddtesty = testy.derivative(3)

            eval = np.linspace(0,1,100)

            maxvx = max(dtestx(eval)/motion_time)
            maxvy = max(dtesty(eval)/motion_time)
            maxax = max(ddtestx(eval)/motion_time**2)
            maxay = max(ddtesty(eval)/motion_time**2)
            maxjx = max(dddtestx(eval)/motion_time**3)
            maxjy = max(dddtesty(eval)/motion_time**3)

            if maxvx > self.vehicles[0].vxmax:
                print maxvx
                raise RuntimeError('Velx guess too high')
            if maxvy > self.vehicles[0].vymax:
                print maxvy
                raise RuntimeError('Vely guess too high')
            if maxax > self.vehicles[0].axmax:
                print maxax
                raise RuntimeError('Accx guess too high')
            if maxay > self.vehicles[0].aymax:
                print maxay
                raise RuntimeError('Accy guess too high')
            if maxjx > self.vehicles[0].jxmax:
                print maxjx
                raise RuntimeError('Jerkx guess too high')
            if maxjy > self.vehicles[0].jymax:
                print maxjy
                raise RuntimeError('Jerky guess too high')

        elif option == 2:
            # generate initial guess for new segment, based on center line

            if isinstance(segment['shape'], Rectangle):
                points = np.c_[segment['start'], segment['end']]
            elif isinstance(segment['shape'], Ring):
                # start_angle and end_angle are defined based on shape.start and shape.end, which is moved
                # to make the ring a little larger to take into account the tolerance, so better use the segment start
                # and end points to make a guess

                # part of a ring, placed in the origin
                start_angle = np.arctan2(segment['start'][1]-segment['position'][1],segment['start'][0]-segment['position'][0])
                end_angle = np.arctan2(segment['end'][1]-segment['position'][1],segment['end'][0]-segment['position'][0])
                if segment['shape'].direction == 'CW':
                    if start_angle < end_angle:
                        start_angle += 2*np.pi  # arctan2 returned a negative start_angle, make positive
                elif segment['shape'] == 'CCW':
                    if start_angle > end_angle:  # arctan2 returned a negative end_angle, make positive
                        end_angle += 2*np.pi

                s = np.linspace(start_angle, end_angle, 50)
                # instead of:
                # s = np.linspace(segment['shape'].start_angle, segment['shape'].end_angle, 50)


                # calculate radius
                radius = (segment['shape'].radius_in+segment['shape'].radius_out)*0.5
                points = np.vstack((segment['pose'][0] + radius*np.cos(s), segment['pose'][1] + radius*np.sin(s)))
                points = np.vstack((points, 0*points[0,:]))  # add guess of all 0 in z-direction
                # Todo: for now only arcs in the XY-plane are considered

            # construct x and y vectors
            x, y, z = [], [], []
            x = np.r_[x, points[0,:]]
            y = np.r_[y, points[1,:]]
            z = np.r_[z, points[2,:]]
            # calculate total length in x-, y- and z-direction
            l_x, l_y, l_z = 0., 0., 0.
            for i in range(len(points[0])-1):
                l_x += points[0,i+1] - points[0,i]
                l_y += points[1,i+1] - points[1,i]
                l_z += points[2,i+1] - points[2,i]
            # calculate distance in x, y and z between each 2 waypoints
            # and use it as a relative measure to build time vector
            time_x, time_y, time_z = [0.], [0.], [0.]

            for i in range(len(points[0])-1):
                if l_x != 0:
                    time_x.append(time_x[-1] + float(points[0,i+1] - points[0,i])/l_x)
                else:
                    time_x.append(0.)
                if l_y != 0:
                    time_y.append(time_y[-1] + float(points[1,i+1] - points[1,i])/l_y)
                else:
                    time_y.append(0.)
                if l_z != 0:
                    time_z.append(time_z[-1] + float(points[2,i+1] - points[2,i])/l_z)
                else:
                    time_z.append(0.)
                # gives time 0...1

            # make approximate one an exact one
            # otherwise fx(1) = 1
            for idx, t in enumerate(time_x):
                if (1 - t < 1e-5):
                    time_x[idx] = 1
            for idx, t in enumerate(time_y):
                if (1 - t < 1e-5):
                    time_y[idx] = 1
            for idx, t in enumerate(time_z):
                if (1 - t < 1e-5):
                    time_z[idx] = 1

            # make interpolation functions
            if (all( t == 0 for t in time_x) and all(t == 0 for t in time_y) and all(t == 0 for t in time_z)):
                # motion_times.append(0.1)
                # coeffs_x = x[0]*np.ones(len(self.vehicles[0].knots[self.vehicles[0].degree-1:-(self.vehicles[0].degree-1)]))
                # coeffs_y = y[0]*np.ones(len(self.vehicles[0].knots[self.vehicles[0].degree-1:-(self.vehicles[0].degree-1)]))
                # init_splines.append(np.c_[coeffs_x, coeffs_y])
                # break
                raise RuntimeError('Trying to make a prediction for goal = current position.')
            if all(t == 0 for t in time_x):
                # if you don't do this, f evaluates to NaN for f(0)
                if not all(t == 0 for t in time_y):
                    time_x = time_y
                else:
                    time_x = time_z
            if all(t == 0 for t in time_y):
                # if you don't do this, f evaluates to NaN for f(0)
                if not all(t == 0 for t in time_x):
                    time_y = time_x
                else:
                    time_y = time_z
            if all(t == 0 for t in time_z):
                # if you don't do this, f evaluates to NaN for f(0)
                if not all(t == 0 for t in time_x):
                    time_z = time_x
                else:
                    time_z = time_y
            # kind='cubic' requires a minimum of 4 waypoints
            fx = interp1d(time_x, x, kind='linear', bounds_error=False, fill_value=1.)
            fy = interp1d(time_y, y, kind='linear', bounds_error=False, fill_value=1.)
            fz = interp1d(time_z, z, kind='linear', bounds_error=False, fill_value=1.)

            # evaluate resulting splines to get evaluations at knots = coeffs-guess
            # Note: conservatism is neglected here (spline value = coeff value)
            coeffs_x = fx(self.vehicles[0].basis.greville())
            coeffs_y = fy(self.vehicles[0].basis.greville())
            coeffs_z = fz(self.vehicles[0].basis.greville())
            init_guess = np.c_[coeffs_x, coeffs_y, coeffs_z]

            # suppose vehicle is moving at half of vmax to calculate motion time
            length_to_travel = np.sqrt((l_x**2+l_y**2+l_z**2))
            max_vel = min(self.vehicles[0].vxmax,self.vehicles[0].vymax)
            motion_time = length_to_travel/(max_vel*0.5)
            init_guess[0] = 0  # initial velocity is zero
            init_guess[1] = 0
            init_guess[2] = 0
            init_guess[3] = 0
            init_guess[-2] = init_guess[-1]  # final velocity is zero
            init_guess[-3] = init_guess[-1]  # final acceleration is also 0 normally
            init_guess[-4] = init_guess[-1]  # final acceleration is also 0 normally

        return init_guess, motion_time

    def get_init_guess_bangbang_jerk(self, segment):
        x0 = segment['start'][0]
        y0 = segment['start'][1]
        x1 = segment['end'][0]
        y1 = segment['end'][1]

        j_lim = self.vehicles[0].jxmax  # jerk limit
        if j_lim != self.vehicles[0].jymax:
            raise RuntimeError('Generating initial guess only possible for x-value = y-value')
        if j_lim != -self.vehicles[0].jxmin:
            raise RuntimeError('Generating initial guess only possible for upper and lower bounds of equal size')

        # self.vehicles[0].basis is for position, take third derivative to obtain the basis for the jerk spline
        # length of this basis determines amount of coeffs that are required
        n_coeffs = len(self.vehicles[0].basis.derivative(3)[0])
        multiple, rest = divmod(n_coeffs, 4)
        # check on amount of coeffs that are required to make desired jerk profile
        # the basic options to obtain a profile with an average of zero are:
        # 4 coeffs: [1, -1, -1, 1] * j_lim
        # 5 coeffs: [1, -1, 0 -1, 1] * j_lim
        # 6 coeffs: [1, 0, -1, -1, 0, 1] * j_lim
        # 7 coeffs: [1, 0, -1, 0, -1, 0, 1] * j_lim
        # for 8 coeffs or more, all non-zero values are copied, with 'multiple'
        # 8 coeffs: [1, 1, -1, -1, -1, -1, 1, 1] * j_lim
        if rest == 0:
            coeffs_j = np.r_[j_lim*np.ones((multiple,1)),-j_lim*np.ones((2*multiple,1)),j_lim*np.ones((multiple,1))]
        elif rest == 1:
            coeffs_j = np.r_[j_lim*np.ones((multiple,1)),-j_lim*np.ones((multiple,1)),0*np.ones((1,1)),-j_lim*np.ones((multiple,1)),j_lim*np.ones((multiple,1))]
        elif rest == 2:
            coeffs_j = np.r_[j_lim*np.ones((multiple,1)), 0*np.ones((1,1)),-j_lim*np.ones((2*multiple,1)), 0*np.ones((1,1)),j_lim*np.ones((multiple,1))]
        elif rest == 3:
            coeffs_j = np.r_[j_lim*np.ones((multiple,1)), 0*np.ones((1,1)),-j_lim*np.ones((multiple,1)), 0*np.ones((1,1)),-j_lim*np.ones((multiple,1)),0*np.ones((1,1)),j_lim*np.ones((multiple,1))]
        else:
            raise RuntimeError('Something wrong with n_coeffs, it was not an int: ', n_coeffs)

        # make jerk spline and integrate to obtain corresponding position spline
        jerk = BSpline(self.vehicles[0].basis.derivative(3)[0], coeffs_j)
        acc = running_integral(jerk)
        vel = running_integral(acc)
        pos = running_integral(vel)
        guess = pos.coeffs  # coefficients guess

        if isinstance(segment['shape'], Rectangle):
            # shift and scale to obtain trajectory from x0 to x1
            guess_x = [g/pos.coeffs[-1]*(x1-x0)+x0 for g in guess]
            guess_y = [g/pos.coeffs[-1]*(y1-y0)+y0 for g in guess]
        elif isinstance(segment['shape'], Ring):
            # Note: guess contains angle (theta) values for this case

            # don't use these ones, the start and end angle include the extension of the shape
            # theta0 = segment['shape'].start_angle
            # theta1 = segment['shape'].end_angle
            center = segment['position']

            theta0 = np.arctan2(y0 - center[1], x0 - center[0])  # start angle
            theta1 = np.arctan2(y1 - center[1], x1 - center[0])  # end angle
            if segment['shape'].direction == 'CW':  # theta must decrease
                if theta0 < theta1:  # theta0 needs to be bigger
                    theta0 += 2*np.pi
            else:  # counter-clockwise, theta must increase
                if theta0 > theta1:  # theta1 needs to be bigger
                    theta1 += 2*np.pi

            # calculate circle radius
            r = np.sqrt((center[0] - x0)**2 + (center[1] - y0)**2)
            # r += self.vehicles[0].tolerance  # shift outwards
            # scale with theta range, and shift with start angle
            guess = [g*(theta1-theta0)/pos.coeffs[-1]+theta0 for g in guess]
            # convert angles to position
            guess_x = r*np.cos(guess) + center[0]
            guess_y = r*np.sin(guess) + center[1]
        else:
            raise RuntimeError('Segment with invalid (not Rectangle or Ring) shape: ', segment['shape'])

        # initial and final velocity and acceleration are 0
        guess_x[0] = x0
        guess_x[1] = x0
        guess_x[2] = x0
        guess_x[-3] = x1
        guess_x[-2] = x1
        guess_x[-1] = x1

        # initial and final velocity and acceleration are 0
        guess_y[0] = y0
        guess_y[1] = y0
        guess_y[2] = y0
        guess_y[-3] = y1
        guess_y[-2] = y1
        guess_y[-1] = y1

        guess_z = 0*np.array(guess_x)
        init_guess = np.c_[guess_x, guess_y, guess_z]

        # construct corresponding splines
        pos_x = BSpline(self.vehicles[0].basis, guess_x)
        vel_x = pos_x.derivative(1)
        acc_x = pos_x.derivative(2)
        jerk_x = pos_x.derivative(3)

        pos_y = BSpline(self.vehicles[0].basis, guess_y)
        vel_y = pos_y.derivative(1)
        acc_y = pos_y.derivative(2)
        jerk_y = pos_y.derivative(3)

        # determine which limit is the most strict, and therefore determines the motion_time
        eval = np.linspace(0,1,100)
        # take into account scaling factor, with appropriate power
        motion_time_j = (max(np.r_[abs(jerk_x(eval)), abs(jerk_y(eval))])/float(j_lim))**(1/3.)
        a_lim = self.vehicles[0].axmax  # jerk limit
        motion_time_a = np.sqrt(max(np.r_[abs(acc_x(eval)), abs(acc_y(eval))])/float(a_lim))
        v_lim = self.vehicles[0].vxmax  # jerk limit
        motion_time_v = max(np.r_[abs(vel_x(eval)), abs(vel_y(eval))])/float(v_lim)
        motion_time = max(motion_time_j, motion_time_a, motion_time_v)
        motion_time = 1.05*motion_time  # take some margin to avoid numerical errors

        return init_guess, motion_time

    def get_init_guess_combined_segment(self):
        # combines the splines of the first two segments into a single one, forming the guess
        # for the new current segment

        # remaining spline through current segment
        spl1 = self.local_problem.father.get_variables(self.vehicles[0], 'splines_seg0')
        # spline through next segment
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

    def get_init_guess_motion_time(self, segment):
        # predict the time of each of the 8 phases of the guess:
        # 1: j_lim
        # 2: a_lim
        # 3: -j_lim
        # 4 & 5: v_lim
        # 6: -j_lim
        # 7: -a_lim
        # 8: j_lim
        j_lim = self.vehicles[0].jxmax
        a_lim = self.vehicles[0].axmax
        v_lim = self.vehicles[0].vxmax

        if isinstance(segment['shape'], Rectangle):
            distance = 0
            for l in range(len(segment['start'])):
                distance += (segment['end'][l] - segment['start'][l])**2
        elif isinstance(segment['shape'], Ring):
            # split arc in two lines going from start point, respectively end point to half of the ring
            radius = (segment['shape'].radius_in + segment['shape'].radius_out)*0.5
            # arc length
            distance = radius * abs(segment['shape'].end_angle - segment['shape'].start_angle)
        else:
            raise RuntimeError('Invalid shape of segment given in get_init_guess_motion_time: ', segment['shape'])

        # determine what the limiting factor is when applying max jerk in phase 1
        # this factor determines the selected T1
        T1_acc = (a_lim/j_lim)  # apply max jerk, when is amax reached
        T1_vel = np.sqrt(v_lim/j_lim)  # apply max jerk, when is vmax reached
        T1_pos = (32 * distance/j_lim)**(1/3.)/4  # apply max jerk, when is distance reached
        T1 = min([T1_acc, T1_vel, T1_pos])
        T3 = T1
        if T1 == T1_pos:  # apply max jerk, until half distance is reached
            T2 = 0.
            T4 = 0.
        elif T1 == T1_vel:  # apply max jerk until vmax is reached and keep vmax until d/2 reached
            T2 = 0.
            T4 = float(distance/2.-(j_lim*T1**3))/v_lim
        else:
            T2_pos = (2*np.sqrt((a_lim*(a_lim**3 + 4*distance*j_lim**2))/4.) - 3*a_lim**2)/(2.*a_lim*j_lim)  # distance limit
            T2_vel = (float(-a_lim**2)/j_lim + v_lim)/a_lim
            T2 = min([T2_vel, T2_pos])
            if T2 == T2_vel:
                T4 = -(a_lim**2*v_lim - j_lim*distance*a_lim + j_lim*v_lim**2)/float(2*a_lim*j_lim*v_lim)
            else:
                T4 = 0.
        T = [T1, T2, T3, T4, T4, T3, T2, T1]
        T_tot = sum(T)
        return T_tot

    def get_init_guess_total_motion_time(self):
        guess_total_time = 0
        for segment in self.environment.room:
            time = self.get_init_guess_motion_time(segment)
            guess_total_time += time
        return guess_total_time