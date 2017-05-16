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

# create vehicle
vehicle = AGV(length=0.8, options={'plot_type': 'agv'})
# vehicle.set_options({'safety_distance': 0.3})
vehicle.define_knots(knot_intervals=5)  # choose lower amount of knot intervals

vehicle.set_initial_conditions([0.8, -0.05, 0., 0.])  # x, y, theta, delta
vehicle.set_terminal_conditions([2.45, -0.35, 0.])  # x, y, theta

# create environment
environment = Environment(room={'shape': Rectangle(width=4, height=1), 'position': [2, 0.]})
rectangle = Rectangle(width=0.8, height=0.2)
environment.add_obstacle(Obstacle({'position': [1., -0.35]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3.4, -0.35]}, shape=rectangle))

# create a point-to-point problem
problem0 = Point2point(vehicle, environment, freeT=True)
# extra solver settings which may improve performance
problem0.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57',
    'ipopt.hessian_approximation': 'limited-memory','ipopt.print_level': 4, 'ipopt.tol': 1e-12,"ipopt.fixed_variable_treatment":"make_constraint"}}}) #for gridding comparison: ,'ipopt.tol': 1e-12
problem0.init()

#####ipopt


# # create simulator
# simulator = Simulator(problem0, sample_time=0.01, update_time=0.1)
# vehicle.plot('input', knots=True)
# problem0.plot('scene', view=[20, -80])
# simulator.run()


#####blocksqp


# create simulator
simulator = Simulator(problem0, sample_time=0.01, update_time=0.1)
simulator.run_once(simulate=False)

options={}
# options['codegen'] = {'build': 'shared', 'flags': '-O2'} # just-in-time compilation
problem = Point2point(vehicle, environment, options, freeT=True)
problem.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': 
	{'verbose':True, 'warmstart': True, 'qp_init' : False, 'print_header': True,
	 'block_hess':1, 'hess_update':2, 'hess_lim_mem':0}}})  #1
	 # 'block_hess':1, 'hess_update':2, 'hess_lim_mem':1 }}}) #2
	 # 'block_hess':1, 'hess_update':1, 'fallback_update':2, 'hess_lim_mem':0 }}}) #3
	 # 'block_hess':1, 'hess_update':1, 'fallback_update':2, 'hess_lim_mem':1 }}}) #4
     # 'block_hess':0, 'hess_update':2, 'hess_lim_mem':0 }}}) #5
	 # 'block_hess':0, 'hess_update':1, 'fallback_update':2, 'hess_lim_mem':0 }}}) #6

# options['codegen'] = {'build': 'jit', 'flags': '-O2'} # just-in-time compilation

# problem.set_options({'hard_term_con': True, 'horizon_time': 12})
vehicle.problem = problem
problem.init()
problem.father._var_result = problem0.father._var_result
problem.father._dual_var_result = problem0.father._dual_var_result
simulator = Simulator(problem, sample_time=0.01, update_time=0.1)

vehicle.plot('input', knots=True)
problem.plot('scene', view=[20, -80])

# run it!
simulator.run(init_reset=False)

