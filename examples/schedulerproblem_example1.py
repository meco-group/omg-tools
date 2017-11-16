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



# In this example we load a big environment, and solve the motion planning
# problem by splitting it over different subproblems. First we use a global planner, which
# is an A-star planner in this case, to find a rough path through the environment. The global
# planner only takes into account the stationary obstacles. Afterwards, we make a MultiFrameProblem,
# which consists of several subproblems. Each subproblem is solved inside a so-called frame
# around the vehicle. Inside the frame, all obstacles, including the moving ones, are taken into
# account. A trajectory which is parameterized as a spline is computed to move through the local frame.
# In the beginning, the first two frames are computed. When the vehicles enters the second frame (which
# overlaps with the first one), the third frame is computed, and so the method continues.

# When computing the trajectory, only one frame is taken into account. This means that sometimes problems
# can arise when obstacles are moving behind the corner.

from omgtools import *

# create vehicle
vehicle = Holonomic(shapes = Circle(radius=2), options={'syslimit': 'norm_2', 'stop_tol': 1.e-2},
                    bounds={'vmax': 10, 'vmin':-10, 'amax':10, 'amin':-10})
veh_size = vehicle.shapes[0].radius

# create environment
start = [192.5,32]
goal = [25,375]
vehicle.set_initial_conditions(start)
vehicle.set_terminal_conditions(goal)

envWidth = 300
envHeight = 400
environment = Environment(room={'shape': Rectangle(width=envWidth, height=envHeight),
                                'position': [150, 200], 'draw':True})

# create stationary obstacles
smallSize = 2
distBetween = 40  # parameter to set space between obstacles
heightObstacles = (envHeight - (4 * distBetween)) / 3.
widthObstacles = (envWidth/2.) - (2 * distBetween)

smallRectangle = Rectangle(width=2*smallSize, height=2*smallSize)
mediumRectangle = Rectangle(width=2*smallSize, height=envHeight/5.-smallSize)
bigRectangle = Rectangle(width=widthObstacles, height=heightObstacles)

environment.add_obstacle(Obstacle({'position': [envWidth/2., envHeight/2.]}, shape=smallRectangle))
environment.add_obstacle(Obstacle({'position': [envWidth/2., distBetween+heightObstacles/2.]}, shape=mediumRectangle))
environment.add_obstacle(Obstacle({'position': [envWidth/2., distBetween+heightObstacles/2.+2*(distBetween+heightObstacles)]}, shape=mediumRectangle))
environment.add_obstacle(Obstacle({'position': [distBetween+widthObstacles/2., distBetween+heightObstacles/2.]}, shape=bigRectangle))
environment.add_obstacle(Obstacle({'position': [distBetween+widthObstacles/2., distBetween+heightObstacles/2.+(distBetween+heightObstacles)]}, shape=bigRectangle))
environment.add_obstacle(Obstacle({'position': [distBetween+widthObstacles/2., distBetween+heightObstacles/2.+2*(distBetween+heightObstacles)]}, shape=bigRectangle))
environment.add_obstacle(Obstacle({'position': [distBetween+widthObstacles/2.+envWidth/2., distBetween+heightObstacles/2.]}, shape=bigRectangle))
environment.add_obstacle(Obstacle({'position': [distBetween+widthObstacles/2.+envWidth/2., distBetween+heightObstacles/2.+(distBetween+heightObstacles)]}, shape=bigRectangle))
environment.add_obstacle(Obstacle({'position': [distBetween+widthObstacles/2.+envWidth/2., distBetween+heightObstacles/2.+2*(distBetween+heightObstacles)]}, shape=bigRectangle))

# create moving obstacles
trajectories1 = {'velocity': {'time': [0],
                             'values': [[-1.6, 0.0]]}}
trajectories2 = {'velocity': {'time': [0],
                             'values': [[-1.5, 0.0]]}}
# the option bounce determines if a moving obstacles bounces back off other obstacles of off the walls
environment.add_obstacle(Obstacle({'position': [230, 270]}, shape=Rectangle(width=14,height=14), options={'bounce': True},
                                  simulation={'trajectories': trajectories2}))
environment.add_obstacle(Obstacle({'position': [150, 210]}, shape=Rectangle(width=14,height=14), options={'bounce': True},
                                  simulation={'trajectories': trajectories1}))

# make global planner
# [25,25] = number of cells in vertical and horizonal direction
globalplanner = AStarPlanner(environment, [25,25], start, goal, options={'veh_size': veh_size})

# make problem
# one extra setting is the frame_type:
## 'corridor': use frame which is as big as possible, without containing stationary obstacles
## 'shift' use frame of fixed size, which is moved when the vehicle comes close to the end point,
## this requires an extra setting 'frame_size': select the size of the shifted (square) frame

# Note: When 'corridor' is selected and your vehicle size is larger than the cell size,
# shifting frames sometimes causes problems
# 'n_frames': number of frames to combine when searching for a trajectory
# 'check_moving_obs_ts': check in steps of ts seconds if a moving obstacle is inside the frame
# 'frame_type': 'corridor': creates corridors
	# 'scale_up_fine': tries to scale up the frame in small steps, leading to the largest possible corridor
	# 'l_shape': cuts off corridors, to obtain L-shapes, and minimize the influence of moving obstacles
# 'frame_type': 'shift': creates frames of fixed size, around the vehicle
	# 'frame_size': size of the shifted frame
options={'freeT': True, 'frame_type': 'corridor', 'scale_up_fine': True}
# options={'freeT': True, 'frame_type': 'shift', 'frame_size': 150}
schedulerproblem=SchedulerProblem(vehicle, environment, globalplanner,options=options)

# simulate the problem
simulator = Simulator(schedulerproblem)

# define what you want to plot
schedulerproblem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()

# save plots
schedulerproblem.save_movie('scene', format='gif', name='multiproblem', number_of_frames=150, movie_time=15, axis=False)
# schedulerproblem.save_movie('scene', format='tikz', name='multiproblemtikz', number_of_frames=100, movie_time=5, axis=False)