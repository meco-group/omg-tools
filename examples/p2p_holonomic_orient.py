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
vehicle = HolonomicOrient()
# Regularization on change in orientation may be required, since the orientation
# of the holonomic vehicle is free. The weight has to be tuned for the application.
# By default no regularization is added.
vehicle.set_options({'reg_type': 'norm_1', 'reg_weight': 10})

vehicle.set_initial_conditions([-1.5, -1.5, np.pi/4.])  # input orientation in deg
vehicle.set_terminal_conditions([2., 2., np.pi/2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3.15, height=0.2)

environment.add_obstacle(Obstacle({'position': [-1.8, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
trajectories = {'velocity': {'time': [3., 4.],
                             'values': [[-0.15, 0.0], [0., 0.15]]}}
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  simulation={'trajectories': trajectories}))



solver = 'ipopt'
if solver is 'ipopt':
    options = {'solver': solver}
    options['solver_options'] = {'ipopt': {'ipopt.tol': 1e-5,
                                                          # 'ipopt.linear_solver': 'ma57',
                                                           'ipopt.warm_start_bound_push': 1e-6,
                                                           'ipopt.warm_start_mult_bound_push': 1e-6,
                                                           'ipopt.warm_start_mult_bound_push': 1e-6,
                                                           'ipopt.mu_init': 1e-5,
                                                           # 'ipopt.hessian_approximation': 'limited-memory',
                                                           # 'ipopt.max_iter': 20000
                                                           }}
    problem = Point2point(vehicle, environment, options, freeT=True)
elif solver is 'worhp':
    options = {'solver': solver}
    worhp_options = {  # 'worhp.qp_ipLsMethod': 'MA57',  # todo: option not found?
        'worhp.MaxIter': 200,
        'worhp.TolOpti': 1e-6,
        # False = warm start
        'worhp.InitialLMest': False,
        'worhp.UserHM': True}  # True = exact Hessian
    options['solver_options'] = {'worhp': worhp_options}
    problem = Point2point(vehicle, environment, options, freeT=True)

# create a point-to-point problem
# problem = Point2point(vehicle, environment, freeT=True)
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)', 'w (rad/s)'])
vehicle.plot('state', knots=True, labels=['x (m)', 'y (m)', 'theta (rad)'])

# run it!
simulator.run()
