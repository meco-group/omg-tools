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
import numpy as np

# create vehicle
vehicle = Quadrotor3D(0.5)

vehicle.set_initial_conditions([-3, -2, -0.5, 0, 0, 0, 0, 0])
vehicle.set_terminal_conditions([3, 2, 0.5])

vehicle.set_options({'safety_distance': 0.1, 'safety_weight': 10})

# create environment
environment = Environment(room={'shape': Cuboid(8, 6, 8)})

trajectory = {'velocity': {'time': [1.5], 'values': [[0, 0, -0.6]]}}
obst1 = Obstacle({'position': [-2, 0, -2]}, shape=Plate(Rectangle(5., 8.), 0.1,
                 orientation=[0., np.pi/2, 0.]), options={'draw': True})
obst2 = Obstacle({'position': [2, 0, 3.5]}, shape=Plate(Rectangle(5., 8.), 0.1,
                 orientation=[0., np.pi/2, 0.]),
                 simulation={'trajectories': trajectory}, options={'draw': True})

environment.add_obstacle([obst1, obst2])

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False, options={'horizon_time': 5.})
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

vehicle.problem = problem
# create simulator
simulator = Simulator(problem, sample_time=0.01, update_time=0.4)
vehicle.plot('input', knots=True)
problem.plot('scene', view=[20, -80])

# run it!
simulator.run()
