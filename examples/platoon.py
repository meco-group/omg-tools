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
import numpy as np

# create fleet of vehicles
vehicle1 = Holonomic1D(width=0.7, height=0.1, options={}, bounds={'vmax': 0.5, 'vmin': -0.5})
vehicle2 = Holonomic1D(width=0.7, height=0.1, options={}, bounds={'vmax': 0.3, 'vmin': -0.3})
vehicle3 = Holonomic1D(width=0.7, height=0.1, options={}, bounds={'vmax': 0.5, 'vmin': -0.5})
vehicle4 = Holonomic1D(width=0.7, height=0.1, options={}, bounds={'vmax': 0.5, 'vmin': -0.5})
vehicles = [vehicle1, vehicle2, vehicle3, vehicle4]
# vehicles = [Holonomic1D() for k in range(4)]
# for vehicle in vehicles:
#     vehicle.define_knots(knot_intervals=10)
fleet = Fleet(vehicles)
fleet.nghb_list[vehicle1] = [vehicle2]
fleet.nghb_list[vehicle4] = [vehicle3]

rel_pos = 1.
configuration = np.array([[k*rel_pos] for k, veh in enumerate(vehicles)])
init_positions = [-3.] + configuration
# init_positions = np.zeros((4, 1))
terminal_positions = [-2.] + configuration
fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Rectangle(width=8., height=0.3)})
rectangle = Rectangle(width=3., height=0.2)

# create a point-to-point problem
# ADMM
options = {'rho': 2., 'horizon_time': 6., 'nesterov_acceleration': False, 'init_iter': 100}
# Dual decomp
# options = {'rho': 0.001, 'horizon_time': 6., 'init_iter': 500}
problem = FormationPoint2point(fleet, environment, options=options)
# problem = FormationPoint2pointCentral(fleet, environment, options=options)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
# problem.plot('scene')
fleet.plot('input', knots=True, prediction=True)
# fleet.plot('state', knots=True, prediction=True)
# problem.plot('residuals')
# run it!
# simulator.run()
simulator.run_once()

# show/save some results
problem.plot_movie('scene', repeat=True)
# vehicle.plot_movie('input', repeat=False, knots=True,
#                    labels=['v_x (m/s)', 'v_y (m/s)'])
# problem.save_movie('scene', axis=False)
# vehicle.save_plot('input', time=3., knots=True)
import matplotlib.pyplot as plt
plt.show(block=True)
