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
import pickle

# fleet_sizes = range(2, 22, 2)
fleet_sizes = [2, 4]

obj_admm = {}
t_it_admm = {}
t_b_admm = {}
obj_central = {}
t_it_central = {}
t_b_central = {}
form_err = {}

for N in fleet_sizes:
    fleet = Fleet([Quadrotor(0.2) for k in range(N)])
    configuration = RegularPolyhedron(0.4, N, orientation=np.pi/2).vertices.T
    if N == 2:
        configuration = np.array([[-0.4, 0.], [0.4, 0.]])
    init_positions = [-4., -4.] + configuration
    terminal_positions = [4., 4.] + configuration
    fleet.set_configuration(configuration.tolist())
    fleet.set_initial_conditions(init_positions.tolist())
    fleet.set_terminal_conditions(terminal_positions.tolist())
    environment = Environment(room={'shape': Square(9.3)})
    environment.add_obstacle(Obstacle({'position': [0., 3.7]},
                                      shape=Rectangle(width=0.2, height=3.)))
    environment.add_obstacle(Obstacle({'position': [0., -5.4]},
                                      shape=Rectangle(width=0.2, height=10.)))
    options = {'rho': 0.03, 'horizon_time': 5.,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
    problem_admm = FormationPoint2point(fleet, environment, options=options)
    # problem_admm.set_options({'codegen': {'build': 'existing'}})
    t_b_admm[N] = problem_admm.init()
    problem_admm.plot('scene')
    simulator = Simulator(problem_admm)
    simulator.run()
    obj_admm[N] = problem_admm.compute_objective()
    t_it_admm[N] = sum(problem_admm.update_times)*1000./len(problem_admm.update_times)
    form_err[N] = problem_admm.get_interaction_error()

# central problem
for N in fleet_sizes:
    fleet = Fleet([Quadrotor(0.2) for k in range(N)])
    configuration = RegularPolyhedron(0.4, N, orientation=np.pi/2).vertices.T
    if N == 2:
        configuration = np.array([[-0.4, 0.], [0.4, 0.]])
    init_positions = [-4., -4.] + configuration
    terminal_positions = [4., 4.] + configuration
    environment = Environment(room={'shape': Square(9.3)})
    environment.add_obstacle(Obstacle({'position': [0., 3.7]},
                                      shape=Rectangle(width=0.2, height=3.)))
    environment.add_obstacle(Obstacle({'position': [0., -5.4]},
                                      shape=Rectangle(width=0.2, height=10.)))
    fleet.set_configuration(configuration.tolist())
    fleet.set_initial_conditions(init_positions.tolist())
    fleet.set_terminal_conditions(terminal_positions.tolist())
    options = {'horizon_time': 5.,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
    problem_central = FormationPoint2pointCentral(fleet, environment, options=options)
    # problem_central.set_options({'codegen': {'build': 'shared', 'flags': '-O0'}})
    t_b_central[N] = problem_central.init()
    problem_central.plot('scene')
    simulator = Simulator(problem_central)
    simulator.run()
    obj_central[N] = problem_central.compute_objective()
    t_it_central[N] = sum(problem_central.update_times)*1000./len(problem_central.update_times)

# data = {'obj_admm': obj_admm, 'obj_central': obj_central, 't_it_admm': t_it_admm, 't_it_central': t_it_central, 't_b_admm': t_b_admm, 't_b_central': t_b_central, 'form_err': form_err}
# pickle.dump(data, open('compare_distr_centr.p', 'wb'))
