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
vehicle = Holonomic(bounds={'vxmin': -1, 'vymin': -1, 'vxmax': 1, 'vymax': 1,
							'axmin': -1, 'aymin': -1, 'axmax': 1, 'aymax': 1}, options={'syslimit':'norm_inf', 'velocity_weight': 50.})

vehicle.set_initial_conditions([-2, -2])
vehicle.set_terminal_conditions([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})

# environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
trajectories = {'velocity': {'time': [0., 40.],
                             'values': [[-0.25, 0.25], [0., 0.15]]}}
environment.add_obstacle(Obstacle({'position': [1, -1]}, shape=Circle(0.5), options={'bounce':False},
                                  simulation={'trajectories': trajectories}))

# environment.add_danger_zone(DangerZone({'position': [0., 0]}, shape=Circle(1),
#                            bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.25}))

environment.add_danger_zone(DangerZone({'position': [1., -1]}, shape=Rectangle(width=2, height=2),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.25},
                           simulation={'trajectories': trajectories}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()

# problem.save_movie('scene', format='gif', name='problemgif', number_of_frames=80, movie_time=4, axis=False)

