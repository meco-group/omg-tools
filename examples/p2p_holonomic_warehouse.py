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

from omgtools import *

# create vehicle
vehicle = Holonomic(options={'syslimit': 'norm_2'})

vehicle.set_initial_conditions([0., 0.])
vehicle.set_terminal_conditions([6., 3.5])

# create environment
environment = Environment(room={'shape': Rectangle(width=7., height=4.5), 'position': [3., 1.75]})
rectangle = Rectangle(width=1., height=1.)

environment.add_obstacle(Obstacle({'position': [1., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [5., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1., 2.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3., 2.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [5., 2.5]}, shape=rectangle))
trajectories1 = {'velocity': {'time': [1., 2.],
                             'values': [[0., 0.0], [0., 0.15]]}}
trajectories2 = {'velocity': {'time': [1., 2.],
                             'values': [[0., 0.0], [0., -0.1]]}}
environment.add_obstacle(Obstacle({'position': [4., 2.5]}, shape=Circle(0.5),
                                  simulation={'trajectories': trajectories2}))
environment.add_obstacle(Obstacle({'position': [2., 1.]}, shape=Circle(0.5),
                                  simulation={'trajectories': trajectories1}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
# problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
# problem.set_options({'solver_options': {'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})
# problem.set_options({'solver_options': {'ipopt': {'ipopt.warm_start_bound_push': 1e-6}}})
# problem.set_options({'solver_options': {'ipopt': {'ipopt.warm_start_mult_bound_push': 1e-6}}})
# problem.set_options({'solver_options': {'ipopt': {'ipopt.mu_init': 1e-5}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v (m/s)', 'w (rad/s)'])

# run it!
simulator.run()

# show some results
problem.plot_movie('scene', repeat=False)
