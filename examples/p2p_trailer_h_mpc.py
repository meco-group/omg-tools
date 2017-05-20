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
sys.path.insert(0, os.getcwd()+"/..")
from omgtools import *

# create vehicle
vehicle = Holonomic(shapes=Circle(0.2))
number_knot_intervals = 9.
vehicle.define_knots(knot_intervals=number_knot_intervals)  # adapt amount of knot intervals
vehicle.set_initial_conditions([2., 5.])  # input orientation in deg
#vehicle.set_terminal_conditions([3., 3., 90.])
vehicle.set_terminal_conditions([8., 5.]) #eerste waarde naar rechts en tweede omhoog
# create trailer
trailer = TrailerHolonomic(lead_veh=vehicle,  shapes=Rectangle(0.3,0.2), l_hitch = 0.6)  # ldie limiet boeit niet, wordt niet in rekening gebracht.
# Note: the knot intervals of lead_veh and trailer should be the same
trailer.define_knots(knot_intervals=number_knot_intervals)  # adapt amount of knot intervals
trailer.set_initial_conditions([0.])  # input orientation in deg
#trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking

# create environment
environment = Environment(room={'shape': Square(10.), 'position': [5.,5.]})
rectangle = Rectangle(width=.2, height=4.)

environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=rectangle))
#environment.add_obstacle(Obstacle({'position': [6., 7.]}, shape=rectangle))
trajectory = {'position': {'time': [3.],
                           'values': [[-0.0 , -1.]]}}
# Here we defined the time-axis and the corresponding values for velocity.
# Note that these values should be interpreted relatively: eg. at time 3, we
# _add_ an extra velocity of [-0.15, 0.0].
# You could also change position and acceleration in a similar way.

# trajectories are put in a simulation dictionary
simulation = {'trajectories': trajectory}
# here we can give a different simulation model, by providing an A and B matrix
# simulation['model'] = {'A': A, 'B': B}
# The behaviour of an obstacle is expressed by x_dot = A*x + B*u
# The state x is composed of [position, velocity, acceleration]
# The input can be provided via a trajectory (is by default 0)
# simulation['trajectories']['input'] = {'time': time, 'values': values}
# simulation['trajectories']['input'] = {'time': time, 'values': values}
# Here the values should be interpreted as absolute.

environment.add_obstacle(Obstacle({'position': [6., 8.]}, shape=rectangle,
                                  simulation=simulation))


# environment.add_obstacle(Obstacle({'position': [2., 1.5]}, shape=rectangle))
# create a point-to-point problem
problem = Point2point(trailer, environment, freeT=True)  # pass trailer to problem
# todo: isn't there are a cleaner way?
problem.father.add(vehicle)  # add vehicle to optifather, such that it knows the trailer variables
# extra solver settings which may improve performance https://www.coin-or.org/Ipopt/documentation/node53.html#SECTION0001113010000000000000
#problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57','ipopt.print_level': 4}}})
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
#problem.set_options({'solver_options': {'ipopt': {'ipopt.print_level': 4}}})
#problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()
#problem.export()

# create simulator
simulator = Simulator(problem, update_time = 0.1)
problem.plot('scene')
trailer.plot('input', knots=True, labels=['v_x (m/s)','v_y (m/s)'])
trailer.plot('state', knots=True, labels=['x_tr (m)', 'y_tr (m)', 'theta_tr (rad)', 'x_veh (m)', 'y_veh (m)', 'theta_veh (rad)'])

# run it!
simulator.run()
problem.save_movie('scene', format='gif', name='trailer_h9', number_of_frames=100, movie_time=5, axis=False)