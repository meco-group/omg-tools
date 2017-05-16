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
import os, sys
sys.path.insert(0,os.getcwd()+'/..')
from omgtools import *

# create vehicle
vehicle = Holonomic()
vehicle.set_options({'safety_distance': 0.1})
vehicle.set_options({'ideal_prediction': False})

vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

environment.add_obstacle(Obstacle({'position': [-100.1, -0.5]}, shape=rectangle))
# environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
# trajectories = {'velocity': {'time': [3., 4.],
#                              'values': [[-0.15, 0.0], [0., 0.15]]}}
# environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
#                                   simulation={'trajectories': trajectories}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57','ipopt.print_level': 4, 'ipopt.tol': 1e-12,"ipopt.fixed_variable_treatment":"make_constraint"}}}) #for gridding comparison: ,'ipopt.tol': 1e-12
# problem.init()

# create simulator
simulator = Simulator(problem, sample_time=0.01, update_time=0.1, options={'debugging': False})
problem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
