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
vehicle = AGV(length=0.8, options={'plot_type': 'agv'})
# vehicle.set_options({'safety_distance': 0.3})
vehicle.define_knots(knot_intervals=10)  # choose lower amount of knot intervals

vehicle.set_initial_conditions([0.8, -0.05, 0., 0.])  # x, y, theta, delta
vehicle.set_terminal_conditions([2.45, -0.35, 0.])  # x, y, theta

# create environment
environment = Environment(room={'shape': Rectangle(width=4, height=1), 'position': [2, 0.]})
rectangle = Rectangle(width=0.8, height=0.2)
environment.add_obstacle(Obstacle({'position': [1., -0.35]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3.4, -0.35]}, shape=rectangle))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
# extra solver settings which may improve performance
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57',
    'ipopt.hessian_approximation': 'limited-memory'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('acc', knots=True)

vehicle.plot('input', knots=True, labels=['v (m/s)', 'ddelta (rad/s)'])
vehicle.plot('state', knots=True, labels=[
             'x (m)', 'y (m)', 'theta (rad)', 'delta (rad)'])

# run it!
simulator.run()
