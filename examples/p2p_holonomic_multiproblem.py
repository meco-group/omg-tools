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

vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

obstacle1 = Obstacle({'position': [-2.1, -0.5]}, shape=rectangle)
obstacle2 = Obstacle({'position': [1.7, -0.5]}, shape=rectangle)
trajectories = {'velocity': {'time': [3., 4.],
                             'values': [[-0.15, 0.0], [0., 0.15]]}}
obstacle3 = Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
    simulation={'trajectories': trajectories})

environment.add_obstacle([obstacle1, obstacle2, obstacle3])

# create problems
problem1 = Point2point(vehicle, environment, freeT=False)
problem1.init()
obstacle3.set_options({'avoid': False})
problem2 = Point2point(vehicle, environment, freeT=False)
problem2.init()

# create simulator & plots
simulator = Simulator(problem1)
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
problem1.plot('scene')
# 1st task
simulator.run()
# 2nd task
vehicle.set_terminal_conditions([2., 0.0])
simulator.run()
# 3th task (we can neglect circular obstacle), but first sleep 2 seconds
simulator.sleep(2.)
simulator.set_problem(problem2)
vehicle.set_terminal_conditions([0.0, 1.0])
simulator.run()
