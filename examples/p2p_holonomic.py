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
vehicle = Holonomic()
vehicle.set_options({'safety_distance': 0.1})
# vehicle.set_options({'1storder_delay': True, 'time_constant': 0.1})
# vehicle.set_options({'input_disturbance': {'fc': 0.01, 'stdev': 0.05*np.ones(2)}})

vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
trajectories = {'velocity': {'time': [3., 4.],
                             'values': [[-0.15, 0.0], [0., 0.15]]}}
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  simulation={'trajectories': trajectories}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
# problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()

# show/save some results
problem.plot_movie('scene', repeat=False)
vehicle.plot_movie('input', repeat=False, knots=True,
                   labels=['v_x (m/s)', 'v_y (m/s)'])
# problem.save_movie('scene', axis=False)
# vehicle.save_plot('input', time=3., knots=True)
