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
vehicle = Holonomic(shapes=Circle(0.1), options={'syslimit': 'norm_2'},
                    bounds={'vmin':-0.5, 'vmax': 0.5, 'amin':-5, 'amax':5})

vehicle.define_knots(knot_intervals = 6)
vehicle.set_initial_conditions([3., 0.])
vehicle.set_terminal_conditions([6., 3.5])

# create environment
environment = Environment(room={'shape': Rectangle(width=7.5, height=4.5),
                                'position': [2.75, 1.75]})

rectangle = Rectangle(width=1., height=1.)

environment.add_obstacle(Obstacle({'position': [1., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [5., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1., 2.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3., 2.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [5., 2.5]}, shape=rectangle))
trajectories1 = {'velocity': {'time': [0], 'values': [[0., 0.15]]}}
trajectories2 = {'position': {'time': [0, 2.5], 'values': [[0, 0], [14, 12.5]]}}
trajectories3 = {'position': {'time': [0, 2.5], 'values': [[0, 0], [15, 11.75]]}}
circle1 = Circle(0.5)
circle2 = Circle(0.25)
environment.add_obstacle(Obstacle({'position': [2., 0.]}, shape=circle1,
                                  simulation={'trajectories': trajectories1}))
environment.add_obstacle(Obstacle({'position': [-10., -10]}, shape=circle1,
                                  simulation={'trajectories': trajectories2}))
environment.add_obstacle(Obstacle({'position': [-10, -10]}, shape=circle2,
                                  simulation={'trajectories': trajectories3}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True,labels=['vx (m/s)', 'vy (m/s)'])

# run it!
simulator.run()
