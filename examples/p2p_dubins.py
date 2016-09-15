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
vehicle = Dubins(bounds={'vmax': 0.7, 'wmax': 60., 'wmin': -60.})  # in deg
vehicle.define_knots(knot_intervals=5)  # choose lower amount of knot intervals

vehicle.set_initial_conditions([0., 0., 0.])  # input orientation in deg
vehicle.set_terminal_conditions([3., 3., 0.])

# create environment
environment = Environment(room={'shape': Square(5.), 'position': [1.5, 1.5]})

trajectories = {'velocity': {'time': [0.5],
                             'values': [[0.25, 0.0]]}}
environment.add_obstacle(Obstacle({'position': [1., 1.]}, shape=Circle(0.5),
                                  simulation={'trajectories': trajectories}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
# extra solver settings which may improve performance
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v (m/s)', 'w (rad/s)'])
vehicle.plot('state', knots=True, labels=['x (m)', 'y (m)', 'theta (rad)'])

# run it!
simulator.run()
