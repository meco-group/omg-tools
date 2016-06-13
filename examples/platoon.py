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
number_of_iterations = 500


# create fleet of vehicles
vehicle1 = Holonomic1D(width=4., height=1., options={}, bounds={'vmax': 30., 'vmin': -30., 'amax': 100., 'amin': -100.})
vehicle2 = Holonomic1D(width=4., height=1., options={}, bounds={'vmax': 20., 'vmin': -20., 'amax': 100., 'amin': -100.})
vehicle3 = Holonomic1D(width=4., height=1., options={}, bounds={'vmax': 25., 'vmin': -25., 'amax': 100., 'amin': -100.})
vehicle4 = Holonomic1D(width=4., height=1., options={}, bounds={'vmax': 30., 'vmin': -30., 'amax': 100., 'amin': -100.})
vehicles = [vehicle1, vehicle2, vehicle3, vehicle4]
fleet = Fleet(vehicles)
fleet.nghb_list[vehicle1] = [vehicle2]
fleet.nghb_list[vehicle4] = [vehicle3]
rel_pos = 6.
configuration = np.array([[k*rel_pos] for k, veh in enumerate(vehicles)])
init_positions = [0.] + configuration
terminal_positions = [150.] + configuration
fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Rectangle(width=222., height=20.), 'position': [109., 0.]})

# create & solve central problem
options = {'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}}
problem = FormationPoint2pointCentral(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once(update=False)
var_central = np.zeros((0, 1))
for vehicle in vehicles:
    iet = vehicle.get_variable('splines0', solution=True, spline=False)
    var_central = np.vstack((var_central, iet))

# create & solve ADMM problem
options = {'rho': 0.01, 'horizon_time': 10., 'init_iter': number_of_iterations-1,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.tol': 1e-8}}}
problem = FormationPoint2point(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once(update=False)
var_admm = problem.get_stacked_x()

# create & solve Fast ADMM problem
options = {'rho': 0.01, 'horizon_time': 10., 'init_iter': number_of_iterations-1,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.tol': 1e-8}}, 'nesterov_acceleration': True}
problem = FormationPoint2point(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once(update=False)
var_fastadmm = problem.get_stacked_x()

# create & solve Dual decomposition problem
options = {'rho': 0.000005, 'horizon_time': 10., 'init_iter': number_of_iterations-1,
           'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.tol': 1e-8}}}
problem = FormationPoint2pointDualDecomposition(fleet, environment, options=options)
problem.init()
simulator = Simulator(problem)
simulator.run_once(update=False)
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
# from matplotlib2tikz import save as tikz_save
# tikz_save('comparison.tikz')
