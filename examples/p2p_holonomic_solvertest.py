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
vehicle.set_options({'safety_distance': 0.1})

vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

# environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
trajectories = {'velocity': {'time': [3., 4.],
                             'values': [[-0.15, 0.0], [0., 0.15]]}}
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  simulation={'trajectories': trajectories}))

# create a point-to-point problem
# select solver
solver = 'ipopt'
if solver is 'ipopt':
    options = {'solver': solver}
    problem = Point2point(vehicle, environment, options, freeT=False)
elif solver is 'worhp':
    options = {'solver': solver}
    worhp_options = {  # 'worhp.qp_ipLsMethod': 'MA57',  # todo: option not found?
        'worhp.MaxIter': 200,
        'worhp.TolOpti': 1e-6,
        # False = warm start
        'worhp.InitialLMest': False,
        'worhp.UserHM': True}  # True = exact Hessian
    options['solver_options'] = {'worhp': worhp_options}
    problem = Point2point(vehicle, environment, options, freeT=False)
elif solver is 'snopt':
    options = {'solver': solver}  # todo: plugin snopt not found?
    problem = Point2point(vehicle, environment, options, freeT=False)
    problem.set_options({'solver_options':
                         {'snopt': {'snopt.Hessian': 'limited memory',
                                    'start': 'warm'}}})
elif solver is 'blocksqp':
    options = {'solver': solver}
    problem = Point2point(vehicle, environment, options, freeT=False)
    problem.set_options({'solver_options':
                         {'blocksqp': {'warmstart': True, 'hess_lim_mem': 0}}})
elif solver is 'knitro':
    options = {'solver': solver}
    problem = Point2point(vehicle, environment, options, freeT=True)
    problem.set_options({'solver_options':
      {'knitro': {'knitro.bar_initpt': 2, 'knitro.honorbnds': 0, 'knitro.scale': 1}}})
      # other possible options: 'knitro.linsolver': 2, 'knitro.bar_murule':5, 'knitro.algorithm':1
else:
    print('You selected solver: ' + solver +
          ' but this solver is not supported. ' +
          'Choose between ipopt, worhp, snopt or blocksqp.')
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
