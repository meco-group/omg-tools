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
import sys
sys.path.insert(0, '/home/ruben/Documents/Work/Programs/motionplanningtoolbox/')
from omgtools import *

# create vehicle
vehicle = Holonomic3D(Cuboid(width=0.5, depth=1., height=0.2))

vehicle.set_initial_conditions([-2., -2., -2])
vehicle.set_terminal_conditions([2., 2., -2])

# create environment
environment = Environment(room={'shape': Cube(5.)})
environment.add_obstacle(Obstacle(
    {'position': [0., 0., -1.5]}, shape=Cuboid(width=0.5, depth=4., height=2.)))
trajectories = {'velocity': {4: [0.0, 0.0, 1.]}}
environment.add_obstacle(Obstacle(
    {'position': [1., 1., -2.25]}, shape=Cube(0.25), trajectories=trajectories))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver': {'ipopt.linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True, 'prediction': False})
simulator.plot.show('scene', view=[45, -45])  # elevation and azimuth of cam
simulator.plot.show('state')
simulator.plot.show('input')

# run it!
simulator.run()

# # show/save some results
simulator.plot.show_movie('scene', repeat=True)
