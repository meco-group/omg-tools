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
vehicle = Quadrotor3Dv2(0.5)

vehicle.set_initial_conditions([-5., -5., -3, 0., 0., 0., 0., 0.])
vehicle.set_terminal_conditions([5., 5., 3])
vehicle.set_options({'safety_distance': 0.1})
vehicle.set_options({'substitution': True, 'exact_substitution': False})

# create environment
environment = Environment(room={'shape': Cube(12.)})
environment.add_obstacle(Obstacle({'position': [-2., 0., -3.4]}, shape=Plate(Rectangle(6., 18.), 0.25, orientation=[0., np.pi/2, 0.])))
environment.add_obstacle(Obstacle({'position': [2., 0., 3.4]}, shape=Plate(Rectangle(6., 18.), 0.25, orientation=[0., np.pi/2, 0.])))

# w = 15.
# h = 15.
# r = 2.5
# triangle = [[r*np.cos(np.pi/6), r*np.sin(np.pi/6)], [r*np.cos(5*np.pi/6), r*np.sin(5*np.pi/6)], [r*np.cos(9*np.pi/6), r*np.sin(9*np.pi/6)]]

# vertices1 = np.c_[triangle[0], [0.5*w, 0.5*h], [0.5*w, -0.5*h], [0., -0.5*h], [0., -r]]
# vertices2 = np.c_[triangle[1], [-0.5*w, 0.5*h], [-0.5*w, -0.5*h], [0., -0.5*h], [0., -r]]
# vertices3 = np.c_[triangle[0], triangle[1], [-0.5*w, 0.5*h], [0.5*w, 0.5*h]]

# shape1 = Plate(Polyhedron(vertices1), 0.01, orientation=[0., 0., 0.])
# shape2 = Plate(Polyhedron(vertices2), 0.01, orientation=[0., 0., 0.])
# shape3 = Plate(Polyhedron(vertices3), 0.01, orientation=[0., 0., 0.])

# environment.add_obstacle(Obstacle({'position': [0., 0., 0.]}, shape=shape1, options={'avoid': True}))
# environment.add_obstacle(Obstacle({'position': [0., 0., 0.]}, shape=shape2, options={'avoid': True}))
# environment.add_obstacle(Obstacle({'position': [0., 0., 0.]}, shape=shape3, options={'avoid': True}))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.set_options({'hard_term_con': False, 'horizon_time': 5.})
problem.init()

vehicle.problem = problem

# create simulator
simulator = Simulator(problem)
problem.plot('scene', view=[25, 60])  # elevation and azimuth of cam
vehicle.plot('state', knots=True)
vehicle.plot('input', knots=True)

# run it!
simulator.run()

# problem.plot_movie('scene', number_of_frames=100, repeat=False, view=[30, 60])

# Save a movie as gif: you need imagemagick for this!
# problem.save_movie('scene', format='gif', name='quad2', view=[25, 60], number_of_frames=100, movie_time=5, axis=False)
