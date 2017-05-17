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
sys.path.insert(0, os.getcwd()+'/..')
from omgtools import *


for n_knots in range(10,11,1):
	# create vehicle
	vehicle = Dubins(bounds={'vmax': 0.7, 'wmax': np.pi/1.1, 'wmin': -np.pi/1.1}, # in rad/s
	                 options={'substitution': False})
	vehicle.define_knots(knot_intervals=n_knots)

	vehicle.set_initial_conditions([1., 3., 0.])  # input orientation in rad
	vehicle.set_terminal_conditions([5., 3., 0.])

	# create environment
	environment = Environment(room={'shape': Square(6.), 'position': [3, 3], 'draw':True})

	environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=Circle(0.5),
	                                  simulation={'trajectories': {}}))

	# create a point-to-point problem
	problem = Point2point(vehicle, environment, freeT=True)
	# extra solver settings which may improve performance
	problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57','ipopt.hessian_approximation': 'limited-memory'}}})

	problem.init()

	vehicle.problem = problem  # to plot error when using substitution

	# create simulator
	simulator = Simulator(problem)
	problem.plot('scene')
	vehicle.plot('input', knots=True, labels=['v (m/s)', 'w (rad/s)'])
	vehicle.plot('acc', knots=True, labels=['a (m/s^2)'])
	vehicle.plot('state', knots=True, labels=['x (m)', 'y (m)', 'theta (rad)'])

	if vehicle.options['substitution']:
	    vehicle.plot('err_pos', knots=True)
	    vehicle.plot('err_dpos', knots=True)

	# run it!
	simulator.run_once()

	print 'internal knots: ', n_knots

	# vehicle.save_plot('state', name='dubins_state', knots=True)
	vehicle.save_plot('input', name='dubins_input'+str(n_knots), knots=True)
	vehicle.save_plot('acc', name='dubins_acc'+str(n_knots), knots=True)
	# Save a movie as multiple Tikz: you need matplotlib2tikz for this!
	# vehicle.save_movie('input', number_of_frames=20, knots=True, name='dubins_input', prediction=True, axis=True)
	# problem.save_movie('scene', number_of_frames=20, name='dubins_scene', axis=True)
