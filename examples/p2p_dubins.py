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
vehicle = Dubins(bounds={'vmax': 0.7, 'wmax': np.pi/3., 'wmin': -np.pi/3.})  # in rad/s
vehicle.define_knots(knot_intervals=5)  # choose lower amount of knot intervals

vehicle.set_initial_conditions([0., 0., 0.])  # input orientation in rad
vehicle.set_terminal_conditions([3., 3., 0.])

# create environment
environment = Environment(room={'shape': Square(5.), 'position': [1.5, 1.5], 'draw':True})

trajectories = {'velocity': {'time': [0.5],
                             'values': [[0.25, 0.0]]}}
environment.add_obstacle(Obstacle({'position': [1., 1.]}, shape=Circle(0.5),
                                  simulation={'trajectories': trajectories}))

# create a point-to-point problem
# problem = Point2point(vehicle, environment, freeT=True)
# extra solver settings which may improve performance
# problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})


solver = 'ipopt'
if solver is 'ipopt':
    options = {'solver': solver}
    problem = Point2point(vehicle, environment, options, freeT=True)
    problem.set_options(
        {'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.hessian_approximation': 'limited-memory',
        'ipopt.print_level': 4, 'ipopt.tol': 1e-12,"ipopt.fixed_variable_treatment":"make_constraint"
                                      }}}) #'ipopt.hessian_approximation': 'limited-memory'
if solver is 'knitro':
    options = {'solver': solver}
    problem = Point2point(vehicle, environment, options, freeT=True)
    problem.set_options(
        {'solver_options': {'knitro': {'knitro.linsolver': 2, 'knitro.bar_murule':5, 'knitro.algorithm':1}}}) #'knitro.bar_initpt': 2, 'knitro.honorbnds': 0, 'knitro.scale': 1

problem.set_options({'hard_term_con': True, 'horizon_time': 12})

problem.init()

vehicle.problem = problem  # to plot error 

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v (m/s)', 'w (rad/s)'])
vehicle.plot('state', knots=True, labels=['x (m)', 'y (m)', 'theta (rad)'])
# vehicle.plot('err_pos', knots=True)
# vehicle.plot('err_dpos', knots=True)

# run it!
simulator.run()
