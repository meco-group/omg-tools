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
import numpy as np
import matplotlib.pyplot as plt

# settings
number_of_iterations = 100

# create fleet
N = 3
vehicles = [Quadrotor(0.2) for l in range(N)]

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.4, N, orientation=np.pi/2).vertices.T
init_positions = [-4., -4.] + configuration
terminal_positions = [4., 4.] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(9.3)})
environment.add_obstacle(Obstacle({'position': [0., 3.7]},
                                  shape=Rectangle(width=0.2, height=3.)))
environment.add_obstacle(Obstacle({'position': [0., -5.4]},
                                  shape=Rectangle(width=0.2, height=10.)))

# create & solve central problem
options = {'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.tol': 1e-8}}, 'horizon_time': 5}
problem = FormationPoint2pointCentral(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once(update=False)
var_central = np.zeros((0, 1))
for vehicle in vehicles:
    splines = problem.father.get_variables(vehicle, 'splines0')
    pos_c = vehicle.get_fleet_center(splines, vehicle.rel_pos_c, substitute=False)
    pos_c = np.hstack([c.coeffs for c in pos_c])
    var_central = np.vstack((var_central, np.c_[pos_c]))

# create & solve ADMM problem
options = {'rho': 0.03, 'horizon_time': 5., 'init_iter': number_of_iterations-1,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.tol': 1e-8}}}
problem = FormationPoint2point(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once(update=False)
var_admm = problem.get_stacked_x()

# create & solve Fast ADMM problem
options = {'rho': 0.03, 'horizon_time': 5., 'init_iter': number_of_iterations-1,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.tol': 1e-8}},
           'nesterov_acceleration': True, 'nesterov_reset': False}
problem = FormationPoint2point(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once(update=False)
var_fastadmm = problem.get_stacked_x()


# create & solve Dual decomposition problem
options = {'rho': 0.002, 'horizon_time': 5., 'init_iter': number_of_iterations-1,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.tol': 1e-8}}}
problem = FormationPoint2pointDualDecomposition(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once()
var_dualdec = problem.get_stacked_x()

# compare convergence

err_admm = [np.linalg.norm(v - var_central)/np.linalg.norm(var_central) for v in var_admm]
err_fastadmm = [np.linalg.norm(v - var_central)/np.linalg.norm(var_central) for v in var_fastadmm]
err_dualdec = [np.linalg.norm(v - var_central)/np.linalg.norm(var_central) for v in var_dualdec]
iterations = np.linspace(0, number_of_iterations, number_of_iterations+1)

plt.figure()
plt.hold(True)
plt.semilogy(iterations, err_dualdec, label='Dual decomposition')
plt.semilogy(iterations, err_admm, label='ADMM')
plt.semilogy(iterations, err_fastadmm, label='Fast ADMM')
plt.legend()
plt.show(block=True)
