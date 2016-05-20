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
import matplotlib.pyplot as plt
import numpy as np

"""
This script compares variants on the ADMM method applied on the distributed
motion planning for a formation of holonomic vehicles.
"""

# residuals
residuals = {'ocp': {'cvx': {}, 'non-cvx': {}},
             'mh': {'cvx': {}, 'non-cvx': {}}}

"""
Case 1: cvx optimal control problem
"""
# # ADMM
# create fleet
N = 4
vehicles_1a = [Holonomic() for l in range(N)]
fleet_1a = Fleet(vehicles_1a)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_1a.set_configuration(configuration.tolist())
fleet_1a.set_initial_conditions(init_positions.tolist())
fleet_1a.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_1a = Environment(room={'shape': Square(5.)})
# create problem
options = {'admm': {'rho': 2., 'init_iter': 300}, 'horizon_time': 10}
problem_1a = FormationPoint2point(fleet_1a, environment_1a, options=options)
problem_1a.init()
# run simulation
simulator_1a = Simulator(problem_1a)
simulator_1a.run_once()
# save residuals
residuals['ocp']['cvx']['admm'] = problem_1a.residuals

# # Fast-ADMM
# create fleet
N = 4
vehicles_1b = [Holonomic() for l in range(N)]
fleet_1b = Fleet(vehicles_1b)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_1b.set_configuration(configuration.tolist())
fleet_1b.set_initial_conditions(init_positions.tolist())
fleet_1b.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_1b = Environment(room={'shape': Square(5.)})
# create problem
options = {'admm': {'rho': 2., 'nesterov_acceleration': True, 'init_iter': 300},
           'horizon_time': 10, 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem_1b = FormationPoint2point(fleet_1b, environment_1b, options=options)
problem_1b.init()
# run simulation
simulator_1b = Simulator(problem_1b)
simulator_1b.run_once()
# save residuals
residuals['ocp']['cvx']['fast-admm'] = problem_1b.residuals

"""
Case 2: non-cvx optimal control problem
"""
# # ADMM
# create fleet
N = 4
vehicles_2a = [Holonomic() for l in range(N)]
fleet_2a = Fleet(vehicles_2a)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_2a.set_configuration(configuration.tolist())
fleet_2a.set_initial_conditions(init_positions.tolist())
fleet_2a.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_2a = Environment(room={'shape': Square(5.)})
environment_2a.add_obstacle(
    Obstacle({'position': [-2.1, -0.5]}, shape=Rectangle(width=3., height=0.2)))
environment_2a.add_obstacle(
    Obstacle({'position': [1.7, -0.5]}, shape=Rectangle(width=3., height=0.2)))
# create problem
options = {'admm': {'rho': 2., 'init_iter': 300}, 'horizon_time': 10,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem_2a = FormationPoint2point(fleet_2a, environment_2a, options=options)
problem_2a.init()
# run simulation
simulator_2a = Simulator(problem_2a)
simulator_2a.run_once()
# save residuals
residuals['ocp']['non-cvx']['admm'] = problem_2a.residuals

# # Fast-ADMM
# create fleet
N = 4
vehicles_2b = [Holonomic() for l in range(N)]
fleet_2b = Fleet(vehicles_2b)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_2b.set_configuration(configuration.tolist())
fleet_2b.set_initial_conditions(init_positions.tolist())
fleet_2b.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_2b = Environment(room={'shape': Square(5.)})
environment_2b.add_obstacle(
    Obstacle({'position': [-2.1, -0.5]}, shape=Rectangle(width=3., height=0.2)))
environment_2b.add_obstacle(
    Obstacle({'position': [1.7, -0.5]}, shape=Rectangle(width=3., height=0.2)))
# create problem
options = {'admm': {'rho': 2., 'nesterov_acceleration': True, 'init_iter': 300},
           'horizon_time': 10, 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem_2b = FormationPoint2point(fleet_2b, environment_2b, options=options)
problem_2b.init()
# run simulation
simulator_2b = Simulator(problem_2b)
simulator_2b.run_once()
# save residuals
residuals['ocp']['non-cvx']['fast-admm'] = problem_2b.residuals

"""
Case 3: cvx moving horizon problem
"""
# # ADMM
# create fleet
N = 4
vehicles_3a = [Holonomic() for l in range(N)]
fleet_3a = Fleet(vehicles_3a)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_3a.set_configuration(configuration.tolist())
fleet_3a.set_initial_conditions(init_positions.tolist())
fleet_3a.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_3a = Environment(room={'shape': Square(5.)})
# create problem
options = {'admm': {'rho': 2., 'max_iter': 300}, 'horizon_time': 10,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem_3a = FormationPoint2point(fleet_3a, environment_3a, options=options)
problem_3a.init()
# run simulation
simulator_3a = Simulator(problem_3a)
simulator_3a.run()
# save residuals
residuals['mh']['cvx']['admm'] = problem_3a.residuals

# # Fast-ADMM
# create fleet
N = 4
vehicles_3b = [Holonomic() for l in range(N)]
fleet_3b = Fleet(vehicles_3b)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_3b.set_configuration(configuration.tolist())
fleet_3b.set_initial_conditions(init_positions.tolist())
fleet_3b.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_3b = Environment(room={'shape': Square(5.)})
# create problem
options = {'admm': {'rho': 2., 'nesterov_acceleration': True, 'max_iter': 300},
           'horizon_time': 10, 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem_3b = FormationPoint2point(fleet_3b, environment_3b, options=options)
problem_3b.init()
# run simulation
simulator_3b = Simulator(problem_3b)
simulator_3b.run_once()
# save residuals
residuals['mh']['cvx']['fast-admm'] = problem_3b.residuals

"""
Case 4: non-cvx moving horizon problem
"""
# # ADMM
# create fleet
N = 4
vehicles_4a = [Holonomic() for l in range(N)]
fleet_4a = Fleet(vehicles_4a)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_4a.set_configuration(configuration.tolist())
fleet_4a.set_initial_conditions(init_positions.tolist())
fleet_4a.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_4a = Environment(room={'shape': Square(5.)})
environment_4a.add_obstacle(
    Obstacle({'position': [-2.1, -0.5]}, shape=Rectangle(width=3., height=0.2)))
environment_4a.add_obstacle(
    Obstacle({'position': [1.7, -0.5]}, shape=Rectangle(width=3., height=0.2)))
# create problem
options = {'admm': {'rho': 2., 'init_iter': 300}, 'horizon_time': 10,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem_4a = FormationPoint2point(fleet_4a, environment_4a, options=options)
problem_4a.init()
# run simulation
simulator_4a = Simulator(problem_4a)
simulator_4a.run_once()
# save residuals
residuals['mh']['non-cvx']['admm'] = problem_4a.residuals

# # Fast-ADMM
# create fleet
N = 4
vehicles_4b = [Holonomic() for l in range(N)]
fleet_4b = Fleet(vehicles_4b)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet_4b.set_configuration(configuration.tolist())
fleet_4b.set_initial_conditions(init_positions.tolist())
fleet_4b.set_terminal_conditions(terminal_positions.tolist())
# create environment
environment_4b = Environment(room={'shape': Square(5.)})
environment_4b.add_obstacle(
    Obstacle({'position': [-2.1, -0.5]}, shape=Rectangle(width=3., height=0.2)))
environment_4b.add_obstacle(
    Obstacle({'position': [1.7, -0.5]}, shape=Rectangle(width=3., height=0.2)))
# create problem
options = {'admm': {'rho': 2., 'nesterov_acceleration': True, 'init_iter': 300},
           'horizon_time': 10, 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem_4b = FormationPoint2point(fleet_4b, environment_4b, options=options)
problem_4b.init()
# run simulation
simulator_4b = Simulator(problem_4b)
simulator_4b.run_once()
# save residuals
residuals['mh']['non-cvx']['fast-admm'] = problem_4b.residuals

"""
Plot residuals
"""
# color definition (rgb)
blue = [17., 110., 138.]
red = [138.,  31.,  17.]
green = [17., 138.,  19.]
lightblue = [106., 194., 238.]
colors = [blue, green, red, lightblue]
colors = [[c/255. for c in color] for color in colors]
n_colors = len(colors)

for key, value in residuals.items():
    for k, v in value.items():
        figure, axes = plt.subplots(3, 1)
        for _k, res in enumerate(v.values()):
            lbl = v.keys()[_k]
            n_it = len(res['primal'])
            iterations = np.linspace(1, n_it, n_it)
            axes[0].semilogy(iterations, res['primal'], '*',
                             color=colors[_k % n_colors], label=lbl)
            axes[1].semilogy(iterations, res['dual'], '*',
                             color=colors[_k % n_colors], label=lbl)
            axes[2].semilogy(iterations, res['combined'], '*',
                             color=colors[_k % n_colors], label=lbl)
        for k in range(3):
            axes[k].legend()
            axes[k].set_xlabel('Iteration')
        axes[0].set_ylabel('Primal residual')
        axes[1].set_ylabel('Dual residual')
        axes[2].set_ylabel('Combined residual')
