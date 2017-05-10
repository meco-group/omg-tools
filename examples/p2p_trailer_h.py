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
options = {}
options['safety_distance'] = 0.1;
vehicle = Holonomic(shapes=Circle(0.3), options=options)  # in deg
vehicle.define_knots(knot_intervals=9)  # adapt amount of knot intervals was eerst 9
vehicle.set_initial_conditions([2., 5.])  # input orientation in deg
#vehicle.set_terminal_conditions([3., 3., 90.])
vehicle.set_terminal_conditions([8., 5.]) #eerste waarde naar rechts en tweede omhoog
vehicle.set_options({'ideal_prediction': False})

# create trailer
trailer = TrailerHolonomic(lead_veh=vehicle,  shapes=Square(0.1), l_hitch = 0.6, options=options)  # limit angle between vehicle and trailer
# Note: the knot intervals of lead_veh and trailer should be the same
trailer.define_knots(knot_intervals=9)  # adapjkt amount of knot intervals was eerst 9
trailer.set_initial_conditions(0.)  # input orientation in deg
#trailer.set_terminal_conditions(0.)  # this depends on the application e.g. driving vs parking

# create environment
environment = Environment(room={'shape': Square(10.), 'position': [5.,5.]})
rectangle = Rectangle(width=.2, height=3.8)
environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [6., 7.]}, shape=rectangle))

# create a point-to-point problem
problem = Point2point(trailer, environment, freeT=True)  # pass trailer to problem
# todo: isn't there are a cleaner way?
problem.father.add(vehicle)  # add vehicle to optifather, such that it knows the trailer variables
problem.vehicles.append(vehicle)
# todo: isn't there are a cleaner way?
vehicle.to_simulate = False
# extra solver settings which may improve performance
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57','ipopt.print_level': 4}}})
problem.init()

# problem.set_options({'hard_term_con': True, 'horizon_time': 12})
# vehicle.problem = problem  # to plot error

# create simulator
simulator = Simulator(problem,sample_time=0.01, update_time=0.3)
problem.plot('scene')
trailer.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y'])
trailer.plot('state', knots=True, labels=['x_tr (m)', 'y_tr (m)', 'theta_tr (rad)', 'x_veh (m)', 'y_veh (m)', 'theta_veh (rad)'])

# run it!
simulator.run()
