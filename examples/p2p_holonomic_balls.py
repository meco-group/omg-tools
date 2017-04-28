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
vehicle = Holonomic(shapes=Circle(0.2), options={'syslimit': 'norm_2'})
vehicle.define_knots(knot_intervals=10)

vehicle.set_initial_conditions([-4., 0])
vehicle.set_terminal_conditions([4., 0])

# create environment
environment = Environment(room={'shape': Square(10.)})

trajectories1 = {'velocity': {'time': [0, 4.5],
                             'values': [[0., 0.0], [0., 0.35]]}}
trajectories2 = {'velocity': {'time': [0, 5.],
                             'values': [[0., 0.0], [0., 0.25]]}}

environment.add_obstacle(Obstacle({'position': [0.,-0.5]}, shape=Circle(0.75),
    simulation={'trajectories': trajectories1}))
environment.add_obstacle(Obstacle({'position': [2.,0.5]}, shape=Circle(0.75)))
environment.add_obstacle(Obstacle({'position': [-2.,0.5]}, shape=Circle(0.75)))

environment.add_obstacle(Obstacle({'position': [0.,-2.25]}, shape=Circle(0.75),
    simulation={'trajectories': trajectories2}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
problem.set_options({'solver_options':
    {'ipopt': {'ipopt.hessian_approximation': 'exact'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
