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

# create vehicle
vehicle = Quadrotor3Dv2(0.5)

vehicle.set_initial_conditions([-5., -5., -3, 0., 0., 0., 0., 0.])
vehicle.set_terminal_conditions([5., 5., 3])
vehicle.define_knots(knot_intervals=10)
vehicle.set_options({'safety_distance': 0.1})
vehicle.set_options({'substitution': True, 'exact_substitution': False})

# create environment
environment = Environment(room={'shape': Cube(12.)})


traj1 = {'velocity': {'time': [1.6], 'values': [[0.0, 0.0, 3.]]}}
traj2 = {'velocity': {'time': [1.6], 'values': [[0.0, 0.0, -3.]]}}

# environment.add_obstacle(Obstacle({'position': [-2., 0., -4.]}, shape=Plate(Rectangle(6., 18.), 0.25, orientation=[0., np.pi/2, 0.]), simulation={'trajectories': traj1}))
# environment.add_obstacle(Obstacle({'position': [2., 0., 4.]}, shape=Plate(Rectangle(6., 18.), 0.25, orientation=[0., np.pi/2, 0.]), simulation={'trajectories': traj2}))

environment.add_obstacle(Obstacle({'position': [-2., 0., -3]}, shape=Plate(Rectangle(4.5, 14.), 0.25, orientation=[0., np.pi/2, 0.]), simulation={'trajectories': traj1}))
environment.add_obstacle(Obstacle({'position': [2., 0., 3]}, shape=Plate(Rectangle(4.5, 14.), 0.25, orientation=[0., np.pi/2, 0.]), simulation={'trajectories': traj2}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.set_options({'hard_term_con': False, 'horizon_time': 5.})
problem.init()

# create simulator
simulator = Simulator(problem)
vehicle.plot('state', knots=True)
vehicle.plot('input', knots=True)
problem.plot('scene', view=[0, -90], axis=True)  # elevation and azimuth of cam

# run it!
simulator.run()

problem.plot_movie('scene', number_of_frames=100, repeat=True, view=[30, 60])

# Save a movie as gif: you need imagemagick for this!
# problem.save_movie('scene', format='gif', name='quad2', view=[25, 60], number_of_frames=100, movie_time=5, axis=False)

import matplotlib.pyplot as plt
plt.show(block=True)
