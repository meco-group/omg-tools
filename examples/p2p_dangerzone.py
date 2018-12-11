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
vehicle = Holonomic(shapes = Circle(0.4), bounds={'vxmin': -1, 'vymin': -1, 'vxmax': 1, 'vymax': 1, 'vmax':1,
							'axmin': -1, 'aymin': -1, 'axmax': 1, 'aymax': 1}, options={'syslimit':'norm_2', 'velocity_weight': 50.})

vehicle.set_options({'safety_distance': 0.1, 'safety_weight': 50.})

vehicle.set_initial_conditions([3, -3])
vehicle.set_terminal_conditions([3, 4.])

# create environment
environment = Environment(room={'shape': Rectangle(width=6, height=9), 'position':[3,1]})

# environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
rect1 = Rectangle(width=2, height=4)
rect2 = Rectangle(width=2, height=2)
environment.add_obstacle(Obstacle({'position': [1,-2]}, shape=rect1))
environment.add_obstacle(Obstacle({'position': [5,-2]}, shape=rect1))
environment.add_obstacle(Obstacle({'position': [1,4]}, shape=rect1))
environment.add_obstacle(Obstacle({'position': [5,4]}, shape=rect1))


trajectories = {'velocity': {'time': [0., 40.],
                             'values': [[-0.3, 0], [0., 0.15]]}}
environment.add_obstacle(Obstacle({'position': [4.5, 1]}, shape=Circle(0.4), options={'bounce':False},
                                  simulation={'trajectories': trajectories}))

environment.add_danger_zone(DangerZone({'position': [3., 0.5]}, shape=Rectangle(width=2, height=3),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.5},
                           simulation={'trajectories': {}}))

environment.add_danger_zone(DangerZone({'position': [3., 0.5]}, shape=Rectangle(width=2, height=3),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.25},
                           simulation={'trajectories': {}}))

# environment.add_danger_zone(DangerZone({'position': [1., -1]}, shape=Rectangle(width=2, height=2),
#                            bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.25},
#                            simulation={'trajectories': trajectories}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
problem.set_options({'solver_options':
    {'ipopt': {
    		   'ipopt.hessian_approximation': 'limited-memory',
    		   # 'ipopt.linear_solver': 'ma57'
           }}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()

# problem.save_movie('scene', format='gif', name='crossroads_pos', number_of_frames=80, movie_time=4, axis=True)
# vehicle.save_movie('input', format='gif', name='crossroads_vel', number_of_frames=80, movie_time=4, axis=True)