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

# This example simulates a bus which can only drive in a straight line. It tries
# to reach its goal position but the journey is not without obstacles.

from omgtools import *

# create vehicle
vehicle = Holonomic(shapes=Circle(1.), bounds={'vxmax': 0.5, 'vxmin': -0.5, 'vymax':0, 'vymin': 0,
                                               'axmax': 1, 'aymax': 0, 'axmin':-1, 'aymin':0},
									   options={'syslimit':'norm_inf', 'safety_distance':0.5, 'safety_weight':10})
vehicle.define_knots(knot_intervals=10)

# create environment
environment = Environment(room={'shape': Rectangle(width=14, height=6), 'position': [7,1]})
rectangle = Rectangle(width=1., height=1)

scenario = 2
if scenario==1:
	vehicle.set_initial_conditions([1, 1])
	vehicle.set_terminal_conditions([10., 1.])
	trajectories1 = {'velocity': {'time': [2.],
	                             'values': [[0,0.3]]}}
	trajectories2 = {'position': {'time': [16.],
	                              'values': [[17, 11]]},
	                 'velocity': {'time': [18.],
	                             'values': [[0, 0.3]]}}
	trajectories3 = {'velocity': {'time': [9., 23.],
	                             'values': [[0.25,-0.25], [-0.5,0.5]]}}
	environment.add_obstacle(Obstacle({'position': [5,-1 ]}, shape=rectangle,
	                                  simulation={'trajectories': trajectories1}))

	environment.add_obstacle(Obstacle({'position': [-10,-10]}, shape=rectangle,
	                                  simulation={'trajectories': trajectories2}))

	environment.add_obstacle(Obstacle({'position': [7,3]}, shape=rectangle,
	                                  simulation={'trajectories': trajectories3}))
if scenario==2:
	vehicle.set_initial_conditions([1, 1])
	vehicle.set_terminal_conditions([11., 1.])
	trajectories1 = {'velocity': {'time': [2.],
	                             'values': [[0,0.3]]}}
	# trajectories2 = {'position': {'time': [16.],
	#                               'values': [[17, 11]]},
	#                  'velocity': {'time': [18.],
	#                              'values': [[0, 0.3]]}}
	trajectories3 = {'velocity': {'time': [15., 40.],
	                             'values': [[0.25,-0.25], [-0.5,0.5]]}}
	environment.add_obstacle(Obstacle({'position': [5,-1 ]}, shape=rectangle,
	                                  simulation={'trajectories': trajectories1}))

	# environment.add_obstacle(Obstacle({'position': [-10,-10]}, shape=rectangle,
	#                                   simulation={'trajectories': trajectories2}))

	environment.add_obstacle(Obstacle({'position': [7,3]}, shape=rectangle,
	                                  simulation={'trajectories': trajectories3}))

# create a point-to-point problem
options= {'horizon_time': 10.}
problem = Point2point(vehicle, environment, options, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
vehicle.plot('a', knots=True, labels=['a_x (m/s)', 'a_y (m/s)'])

# run it!
simulator.run()

problem.save_movie('scene', number_of_frames=150, name='bus', axis=False)
