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

import sys, os
sys.path.insert(0, os.getcwd()+'/..')
from omgtools import *

# create vehicle
vehicle = Holonomic()
vehicle.set_options({'safety_distance': 0.1})

vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
# trajectories = {'velocity': {'time': [3., 4.],
#                              'values': [[-0.15, 0.0], [0., 0.15]]}}
# environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
#                                   simulation={'trajectories': trajectories}))

# initial guess via ipopt
problem0 = Point2point(vehicle, environment, freeT=False)
problem0.set_options({'solver': 'ipopt', 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})

problem0.init()
simulator = Simulator(problem0)
simulator.run_once(simulate=False)

# online mp via blocksqp
problem1 = Point2point(vehicle, environment, freeT=False)
problem1.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'warmstart': True, 'hess_lim_mem': 0, 'print_header': False}}})
problem1.init()

problem1.father._var_result = problem0.father._var_result
problem1.father._dual_var_result = problem0.father._dual_var_result
simulator = Simulator(problem1)
simulator.run_once(simulate=False, init_reset=False)

problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'warmstart': True, 'hess_lim_mem': 0, 'print_header': False}}})
problem.init()

problem.father._var_result = problem1.father._var_result
problem.father._dual_var_result = problem1.father._dual_var_result
simulator = Simulator(problem)
simulator.run_once(simulate=False, init_reset=False)


# problem.plot('scene')
# vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
# simulator.run(init_reset=False)

