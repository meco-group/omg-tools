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
import numpy as np

# create vehicle
vehicle = Dubins(bounds={'vmax': 0.7, 'wmin': -30., 'wmax': 30.})
vehicle.define_knots(knot_intervals=6)
vehicle.set_initial_conditions([0., -2.0, np.pi/2])
vehicle.set_terminal_conditions([-1.5, 2.0, np.pi/2])

# create environment
environment = Environment(room={'shape': Square(5.)})
beam1 = Beam(width=2.2, height=0.2)
environment.add_obstacle(Obstacle({'position': [-2., 0.]}, shape=beam1))
environment.add_obstacle(Obstacle({'position': [2., 0.]}, shape=beam1))

beam2 = Beam(width=1.4, height=0.2)
horizon_time = 15.
omega = 0.1*1.*(2*np.pi/horizon_time)
velocity = [0., 0.]
# velocity = [0., -0.2] # crazy revolving door
environment.add_obstacle(Obstacle({'position': [0., 0.], 'velocity': velocity,
    'orientation': 0.+np.pi/4., 'angular_velocity': omega},
     shape=beam2, simulation={}, options={'horizon_time': horizon_time}))
environment.add_obstacle(Obstacle({'position': [0., 0.], 'velocity': velocity,
    'orientation': 0.5*np.pi+np.pi/4., 'angular_velocity': omega},
    shape=beam2, simulation={}, options={'horizon_time': horizon_time}))

# create a point-to-point problem
solver = 'ipopt'
if solver is 'knitro':
    options={'solver': solver, 'horizon_time': horizon_time, 'hard_term_con': True}
    problem = Point2point(vehicle, environment, options, freeT=False)
    problem.set_options(
    	# {'solver_options':{'knitro':{'knitro.bar_initpt': 2, 'knitro.honorbnds': 0, 'knitro.scale': 1, 'knitro.linsolver':4, 'knitro.bar_murule':5, 'knitro.bar_directinterval':0, 'knitro.algorithm':1}}})
        # {'solver_options': {'knitro': {'knitro.tuner':1}}})
        # {'solver_options': {'knitro': {'knitro.ms_enable': 1, 'knitro.ms_maxsolves':3}}})
        {'solver_options': {'knitro': {}}})
if solver is 'ipopt':
    options={'solver': solver, 'horizon_time': horizon_time, 'hard_term_con': True}
    problem = Point2point(vehicle, environment, options, freeT=False)
    problem.set_options(
        {'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57',
        							  'ipopt.hessian_approximation': 'limited-memory',
        							  'ipopt.warm_start_mult_bound_push': 1e-6}}}) 
problem.init()

# problem.export2AMPL()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
