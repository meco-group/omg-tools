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
import numpy as np

# create vehicle
vehicle = Holonomic()
vehicle.set_initial_conditions([0., -2.0])
vehicle.set_terminal_conditions([0., 2.0])

# create environment
environment = Environment(room={'shape': Square(5.)})
beam1 = Beam(width=2.2, height=0.2)
environment.add_obstacle(Obstacle({'position': [-2., 0.]}, shape=beam1))
environment.add_obstacle(Obstacle({'position': [2., 0.]}, shape=beam1))

beam2 = Beam(width=1.4, height=0.2)
horizon_time = 10.
omega = 1.5*(2*np.pi/horizon_time)
environment.add_obstacle(Obstacle({'position': [0., 0.], 'angular_velocity': omega},
                                  shape=beam2, horizon_time=horizon_time))
environment.add_obstacle(Obstacle({'position': [0., 0.], 'orientation': 0.5*np.pi, 'angular_velocity': omega},
                                  shape=beam2, horizon_time=horizon_time))

# create a point-to-point problem
problem = Point2point(
    vehicle, environment, freeT=False, options={'horizon_time': horizon_time})
problem.set_options({'solver': {'ipopt.linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True, 'prediction': False})
simulator.plot.show('scene')
simulator.plot.show('input')

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('scene', repeat=True, number_of_frames=80)
