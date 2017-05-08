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

# create vehicle
vehicle = Dubins(bounds={'vmax': 0.7, 'wmax': np.pi/3., 'wmin': -np.pi/3.})  # in rad/s
vehicle.define_knots(knot_intervals=5)  # choose lower amount of knot intervals

vehicle.set_initial_conditions([0., 0., 0.])  # input orientation in rad
vehicle.set_terminal_conditions([3., 3., 0.])

# create environment
environment = Environment(room={'shape': Square(5.), 'position': [1.5, 1.5], 'draw':True})

trajectories = {'velocity': {'time': [0.],
                             'values': [[0.25, 0.0]]}}
environment.add_obstacle(Obstacle({'position': [1., 1.]}, shape=Circle(0.5)))

# create a point-to-point problem
problem0 = Point2point(vehicle, environment, freeT=True)
problem0.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.hessian_approximation': 'limited-memory'}}})
# problem0.set_options({'hard_term_con': True, 'horizon_time': 12})
problem0.init()

# create simulator
simulator = Simulator(problem0, sample_time=0.01, update_time=0.1)
simulator.run_once(simulate=False)

options={}
# options['codegen'] = {'build': 'shared', 'flags': '-O2'} # just-in-time compilation
problem = Point2point(vehicle, environment, options, freeT=True)
problem.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'verbose':True, 'warmstart': True, 'qp_init' : False, 'hess_lim_mem': 0, 'print_header': False}}})
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




#### ipopt

# simulator = Simulator(problem0, sample_time=0.01, update_time=0.1)


# vehicle.plot('input', knots=True)
# problem0.plot('scene', view=[20, -80])

# simulator.run()

# # # create simulator
# # simulator = Simulator(problem0, sample_time=0.01, update_time=0.1)
# # simulator.run_once(simulate=False)

# # problem = Point2point(vehicle, environment, freeT=True)
# # problem.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'verbose':True, 'warmstart': True, 'qp_init' : False, 'hess_lim_mem': 1, 'print_header': False}}})
# # # problem.set_options({'hard_term_con': True, 'horizon_time': 12})
# # vehicle.problem = problem
# # problem.init()
# # problem.father._var_result = problem0.father._var_result
# # problem.father._dual_var_result = problem0.father._dual_var_result
# # simulator = Simulator(problem, sample_time=0.01, update_time=0.1)

# # vehicle.plot('input', knots=True)
# # problem.plot('scene', view=[20, -80])

# # # run it!
# # simulator.run(init_reset=False)
