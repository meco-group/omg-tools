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
sys.path.insert(0, os.getcwd()+"/..")
from omgtools import *
# create vehicle
vehicle = Holonomic(shapes=Square(0.2))
vehicle.define_knots(knot_intervals=15)  # adapt amount of knot intervals

#vehicle.set_options({'safety_distance': 0.1})

vehicle.set_initial_conditions([0., 0.])
vehicle.set_terminal_conditions([3., 3.])
vehicle.define_knots(knot_intervals=15)  # adapt amount of knot intervals

# create environment
environment = Environment(room={'shape': Square(10.), 'position': [1.5, 1.5]})
rectangle = Rectangle(width=.2, height=3.)

environment.add_obstacle(Obstacle({'position': [1.5, 1.]}, shape=rectangle))


# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57','ipopt.print_level': 4}}})
#problem.set_options({'solver_options': {'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})

problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])
#problem.export()
# run it!
simulator.run()

