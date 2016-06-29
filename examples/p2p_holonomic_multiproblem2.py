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
import sys
sys.path.insert(0, "/home/tim/Dropbox/EigenDocumenten/Doctoraat/MotionPlanning/omg-tools") 
from omgtools import *

import numpy as np
from scipy.interpolate import interp1d

def verticesToRectangle(vertices):
    # syntax: vertices = (xmin, ymin, xmax, ymax)
    # Transforms two vertices into a rectangle representation
    # Note: this function supposes angle of rectangle to be 0
    # vertex0 is the bottom left vertex, vertex 1 is the top right

    vertex0 = np.array([vertices[0], vertices[1]])
    vertex1 = np.array([vertices[2], vertices[3]])
    w, h = vertex1-vertex0  # w = x-direction, h = y-direction
    pos = vertex0 + (vertex1-vertex0)/2.
    # todo: remove
    vel = [0., 0.]
    angle = 0.
    rectangle = {'type': 'rectangle', 'position': pos,
                            'velocity': vel, 'angle': angle, 'height': h, 'width': w}
    return rectangle

# frames = {1: {'border': [xbmin,ybmin,xbmax,ybmax], 'waypoints': [[x0,y0],[x1,y1],[x2,y2],[x3,y3]], 
#               'obstacles': [{'type': 'rectangle', 'position': [posx, posy],
#                             'velocity': [0.,0.], 'angle': 0., 'height': h, 'width': w}},
#                             {'type': 'circle', 'position': [posx, posy],
#                             'velocity': [0.,0.], 'radius': r}],
#           2: {}}


# practical example:
# {'endpointFrame': [(53.0, 211.14530524812568)], 'obstacles': {
#                 0: {'position': (48.80859375, 186.26953125, 53.0, 193.2421875), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 1: {'position': (34.86328125, 202.20703125, 53.0, 229.1015625), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 2: {'position': (48.80859375, 234.08203125, 49.8046875, 236.0), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 3: {'position': (38.84765625, 230.09765625, 47.8125, 236.0), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 4: {'position': (12.94921875, 230.09765625, 31.875, 236.0), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 5: {'position': (3.0, 196.23046875, 9.9609375, 203.203125), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 6: {'position': (3.0, 188.26171875, 23.90625, 205.1953125), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 7: {'position': (10.95703125, 186.0, 13.9453125, 187.265625), 'angle': 0, 'type': 'rectangle', 'velocity': 0},
#                 8: {'position': (18.92578125, 198.22265625, 35.859375, 217.1484375), 'angle': 0, 'type': 'rectangle', 'velocity': 0}},
#                 'border': (3.0, 186.0, 53.0, 236.0)}


#Syntax for calling OMG-tools multiple times
frames = {0: {'endpointFrame': [(2.5, 2.5)], 'border': (0., 0., 5., 5.), 'waypoints': [[1.,1.],[2.,2.],[3.,3.]], 
              'obstacles': [{'type': 'rectangle', 'position': (0.5, 3.5, 1.5, 4.5), 'velocity': [0.,0.], 'angle': 0.}]},
          1: {'endpointFrame': [(9., 9.)], 'border': (2.5, 2.5, 10., 10.), 'waypoints': [[3.,3.],[6.,6.],[9.,9.]], 
              'obstacles': [{'type': 'rectangle', 'position': (7.5, 5.5, 8.5, 6.5), 'velocity': [0.,0.], 'angle': 0.}]}}
nFrames = len(frames)
start = frames[0]['waypoints'][0]
goal = frames[nFrames-1]['endpointFrame'][0]  # returns a tuple

splines = {}  # holds all initial guesses of spline variables

# create vehicle
vehicle = Holonomic(Circle(radius=0.4))
# vehicle.set_options({'safety_distance': 0.1})
vehicle.define_knots(knot_intervals=10)

# Initial guess step
for number, frame in frames.iteritems():
	# Use waypoints for prediction
    x, y = [], []
    for waypoint in frame['waypoints']:
        x.append(waypoint[0])
        y.append(waypoint[1])
    # Calculate total length in x- and y-direction
    l_x, l_y = 0., 0.
    for i in range(len(frame['waypoints'])-1):
        waypoints = frame['waypoints']
        l_x += waypoints[i+1][0] - waypoints[i][0]
        l_y += waypoints[i+1][1] - waypoints[i][1]
    # Calculate distance in x and y between each 2 waypoints and use it as a relative measure to build time vector
    time_x = [0.]
    time_y = [0.]
    for i in range(len(frame['waypoints'])-1):
        waypoints = frame['waypoints']
        time_x.append(time_x[-1] + float(waypoints[i+1][0] - waypoints[i][0])/l_x)
        time_y.append(time_y[-1] + float(waypoints[i+1][1] - waypoints[i][1])/l_y)  # gives time 0...1

    # Make interpolation functions
    fx = interp1d(time_x, x, kind='linear')  # kind='cubic' requires a minimum of 4 waypoints
    fy = interp1d(time_y, y, kind='linear')

    # Evaluate resulting splines to get evaluations at knots = coeffs-guess
    coeffs_x = fx(vehicle.knots[vehicle.degree-1:-(vehicle.degree-1)])
    coeffs_y = fy(vehicle.knots[vehicle.degree-1:-(vehicle.degree-1)])
    init_guess = np.c_[coeffs_x, coeffs_y]
    # Pass on initial guess
    vehicle.set_init_spline_value(init_guess)

    # Set start and goal
    vehicle.set_initial_conditions([waypoints[0]])
    vehicle.set_terminal_conditions([waypoints[-1]])

    # Solve one time
    border = verticesToRectangle(frame['border'])
    environment = Environment(room={'shape': Rectangle(width=border['width'], height=border['height']),
                                    'position': border['position'], 'orientation': border['angle'] })

    obstacles = []
    for obstacle in frame['obstacles']:
        if obstacle['type'] == 'circle':  # [x,y,dx,dy,r]
            circle = Circle(radius=obstacle['radius'])
            traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
            obstacles.append(Obstacle({'position': obstacle['position']}, shape=circle,
                                                simulation={'trajectories': traj}))
        elif obstacle['type'] == 'rectangle':  # [x,y,theta,dx,dy,w,h]
            rectangle = verticesToRectangle(obstacle['position'])
            rect = Rectangle(width=rectangle['width'], height= rectangle['height'], orientation= obstacle['angle'])
            traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
            obstacles.append(Obstacle({'position': rectangle['position']}, shape=rect,
                                                simulation={'trajectories': traj}))
        else:
            raise RuntimeError('Obstacle received which was not a circle or a rectangle')  
    environment.add_obstacle(obstacles)

    problem = Point2point(vehicle, environment, freeT=True)
    problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
    problem.init()
    simulator = Simulator(problem)
    # problem.plot('scene')
    # vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

    simulator.run_once()

    motion_time = simulator.problem.father.get_variables(simulator.problem, 'T',)[0][0]

    # Call object tracker for relevant moving obstacles
    # Give start and end time
    # obstacles = moving_obstacles(frame['border'], T)
    movingObstacles = None

	# If changes, update problem
    if movingObstacles is not None:
        for obstacle in obstacles:
            if obstacle['type'] == 'circle':  # [x,y,dx,dy,r]
                circle = Circle(radius=obstacle['radius'])
                traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
                environment.add_obstacle(Obstacle({'position': obstacle['position']}, shape=circle,
                                                    simulation={'trajectories': traj}))
            elif obstacle['type'] == 'rectangle':  # [x,y,theta,dx,dy,w,h]
                rectangle = Rectangle(width=obstacle['width'], height= obstacle['height'], orientation= obstacle['angle'])
                traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
                environment.add_obstacle(Obstacle({'position': obstacle['position']}, shape=rectangle,
                                                    simulation={'trajectories': traj}))
            else:
                raise RuntimeError('Obstacle received which was not a circle or a rectangle')  
            # Add moving obstacles to original set of obstacles
            frame['obstacles'].append(obstacle) 
        # Make new, complete, problem
        problem = Point2point(vehicle, environment, freeT=True)
        problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
        problem.init()
        simulator = Simulator(problem)
        # problem.plot('scene')
        # vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])
        simulator.run_once()
    # else: no extra obstacles, go on

    # Save trajectories for all frames, as initial guesses
    splines[number] = problem.father.get_variables()[vehicle.label, 'splines0']


# Iteration step
curr_pos = frames[0]['waypoints'][0]
# create vehicle
# todo: why can't I take the old vehicle here?
vehicle = Holonomic(Circle(radius=0.4))
# vehicle.set_options({'safety_distance': 0.1})
vehicle.define_knots(knot_intervals=10)
for number, frame in frames.iteritems():
    
    # ask for new moving obstacles here, but not in each iteration. 
    # check if I got a new frame: based on the distance to the goal within the frame, after each iteration
    # make a class of this script

    # Get initial guess from first step
    # todo: how good is this? since starting point of frame 2!= waypoint 0 of frame 2 evt.
    vehicle.set_init_spline_value(splines[number])

    vehicle.set_initial_conditions(curr_pos)
    vehicle.set_terminal_conditions(frame['waypoints'][-1])

    # create environment
    border = verticesToRectangle(frame['border'])
    environment = Environment(room={'shape': Rectangle(width=border['width'], height=border['height']),
                                    'position': border['position'], 'orientation': border['angle']})

    obstacles = []
    for obstacle in frame['obstacles']:
        if obstacle['type'] == 'circle':  # [x,y,dx,dy,r]
            circle = Circle(radius=obstacle['radius'])
            traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
            obstacles.append(Obstacle({'position': obstacle['position']}, shape=circle,
                                                simulation={'trajectories': traj}))
        elif obstacle['type'] == 'rectangle':  # [x,y,theta,dx,dy,w,h]
            rectangle = verticesToRectangle(obstacle['position'])
            rect = Rectangle(width=rectangle['width'], height= rectangle['height'], orientation= rectangle['angle'])
            traj = {'velocity': {'time': [0.], 'values': [rectangle['velocity']]}}
            obstacles.append(Obstacle({'position': rectangle['position']}, shape=rect,
                                                simulation={'trajectories': traj}))
        else:
            raise RuntimeError('Obstacle received which was not a circle or a rectangle')  
    environment.add_obstacle(obstacles)

    problem = Point2point(vehicle, environment, freeT=True)
    problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
    problem.init()
    simulator = Simulator(problem)

    border_tot = frames[0]['border'][:2] + frames[len(frames)-1]['border'][2:]
    total_border = verticesToRectangle(border_tot)
    total_environment = Environment(room={'shape': Rectangle(width=total_border['width'], height=total_border['height']),
                                    'position': total_border['position'], 'orientation': total_border['angle']})    

    problem.plot('scene')
    vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

    try:
        region = frames[number+1]['border']
    except:  # last frame reached, region = goal state
        region = (frames[number]['waypoints'][-1] + frames[number]['waypoints'][-1])

    simulator.set_problem(problem)
    vehicle.set_terminal_conditions(frame['waypoints'][-1])
    vehicle.reinit_splines(problem)

    curr_pos = simulator.run_iterative(region)  # run until vehicle reached region

# plot movie
problem.plot_movie('scene', repeat=False)

    # return current_pos