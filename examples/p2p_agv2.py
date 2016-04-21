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
vehicle = AGV(length=0.8,
				  bounds={'vmax': 0.5, 'dmax': 30., 'dmin': -30., 'ddmax': 45, 'ddmin': -45},  # in deg
                  options={'plot_type': 'agv'})
vehicle.define_knots(knot_intervals=5)  # choose lower amount of knot intervals

vehicle.set_initial_conditions([-1., -0.5, 0.] , [0.])  # x, y, theta, delta
vehicle.set_terminal_conditions([4., 2., 0.])  # x, y, theta

# create environment
environment = Environment(room={'shape': Square(6.), 'position': [1.5, 1.5]})
rectangle = Rectangle(width=0.8, height=0.2)
# environment.add_obstacle(Obstacle({'position': [1., -0.35]}, shape=rectangle))
# environment.add_obstacle(Obstacle({'position': [3.6, -0.35]}, shape=rectangle))
# trajectories = {'velocity': {'time': [3., 4.],
#                              'values': [[0.15, 0.0], [0., 0.15]]}}
# environment.add_obstacle(Obstacle({'position': [0.5, 0.5]}, shape=Circle(0.4),
#                                   simulation={'trajectories': trajectories}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
# extra solver settings which may improve performance
problem.set_options({'solver': {'ipopt.linear_solver': 'ma57'}})
problem.set_options({'solver': {'ipopt.hessian_approximation': 'limited-memory'}})
# problem.set_options({'solver': {'ipopt.warm_start_bound_push': 1e-6}})
# problem.set_options({'solver': {'ipopt.warm_start_mult_bound_push': 1e-6}})
# problem.set_options({'solver': {'ipopt.mu_init': 1e-5}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True, 'prediction': False})
simulator.plot.show('scene')
simulator.plot.show('input')
simulator.plot.show('state')

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('scene', repeat=True)