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
#source /opt/ros/indigo/setup.bash
#export TUE_ENV=
#source ~/.tue/setup.bash
# export LD_LIBRARY_PATH=/home/joland/coinhsl-2014.01.10:$LD_LIBRARY_PATH

import sys, os


sys.path.insert(0, os.getcwd()+"/..")
from omgtools import *

knots= 9
# create vehicle
vehicle = Dubins(shapes=Circle(0.2), bounds={'vmax': 0.8, 'wmax': 60., 'wmin': -60.})  # in deg
vehicle.define_knots(knot_intervals = knots)  # adapt amount of knot intervals was eerst 9
vehicle.set_initial_conditions([2., 5., 0.])  # input orientation in deg
#vehicle.set_terminal_conditions([3., 3., 90.])
vehicle.set_terminal_conditions([8., 5., 0.]) #eerste waarde naar rechts en tweede omhoog

# create trailer
trailer = TrailerKnik(lead_veh=vehicle,  shapes=Rectangle(0.3,0.2), l_hitch = 0.215, l_hitch1 = 0.2075,bounds={'tmax': 90., 'tmin': -90.})  # limit angle between vehicle and trailer
# Note: the knot intervals of lead_veh and trailer should be the same
trailer.define_knots(knot_intervals=knots) # adapt amount of knot intervals was eerst 9
trailer.set_initial_conditions(0.)  # input orientation in rad
#trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking

 # create environment
environment = Environment(room={'shape': Square(10.), 'position': [5.,5.]})
rectangle = Rectangle(width=.2, height=3.8)

environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [6., 7.]}, shape=rectangle))

# Let's also add a circular obstacle which will suddenly start to move.
# create a point-to-point problem
problem = Point2point(trailer, environment, freeT=True)  # pass trailer to problem
# todo: isn't there are a cleaner way?
problem.father.add(vehicle)  # add vehicle to optifather, such that it knows the trailer variables
# extra solver settings which may improve performance https://www.coin-or.org/Ipopt/documentation/node53.html#SECTION0001113010000000000000
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57',    'ipopt.hessian_approximation': 'limited-memory'}}})
#problem.set_options({'solver_options': {'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})

problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
trailer.plot('input', knots=True, labels=['v (m/s)', 'ddelta (rad/s)'])
trailer.plot('state', knots=True, labels=['x_tr (m)', 'y_tr (m)', 'theta_tr (rad)', 'x_veh (m)', 'y_veh (m)', 'theta_veh (rad)'])

# run it!
simulator.run_once()
problem.save_movie('scene', format='gif', name='trailer_joland_nieuwe_ode_recht_obstakel2', number_of_frames=100, movie_time=5, axis=False)
