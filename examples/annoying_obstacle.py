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
vehicle.set_options({'safety_distance': 0.2, 'safety_weight': 1.e2})

vehicle.set_initial_conditions([-4., 0.])
vehicle.set_terminal_conditions([4., 0.])

# create environment
environment = Environment(room={'shape': Rectangle(width=10., height=5.)})
rectangle = Rectangle(width=3., height=0.2)

# we provide another simulation model than the model that is used to predict
# the motion for the obstacle: the obstacle is moving sinusoidal, but our
# optimization problem uses a linear prediction.
wn = 2*np.pi/5.
a1 = np.array([[0., 1., 0.], [0., 0., 0.], [0., 0., 0.]])
a2 = np.array([[0., 1., 0.], [-wn**2, 0., 0.], [0., 0., 0.]])
s1 = np.array([[1., 0.], [0., 0.]])
s2 = np.array([[0., 0.], [0., 1.]])
A = np.kron(a1, s1) + np.kron(a2, s2)

simulation = {'model': {'A': A}}
environment.add_obstacle(Obstacle({'position': [-3.5, -1.], 'velocity': [0.4, 0.]},
                                  shape=Circle(0.4), simulation=simulation))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
# problem.set_options({'solver': {'ipopt.linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()

# show/save some results
problem.plot_movie('scene', repeat=False)
