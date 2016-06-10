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
vehicle1 = Vehicle1D(width=0.7, height=0.1, time_constant=0.4, options={}, bounds={'vmax': 33., 'vmin': -33.})
vehicle2 = Vehicle1D(width=0.7, height=0.1, time_constant=0.4, options={}, bounds={'vmax': 25., 'vmin': -25.})
vehicle3 = Vehicle1D(width=0.7, height=0.1, time_constant=0.4, options={}, bounds={'vmax': 25., 'vmin': -25.})
vehicle4 = Vehicle1D(width=0.7, height=0.1, time_constant=0.4, options={}, bounds={'vmax': 33., 'vmin': -33.})
vehicles = [vehicle1, vehicle2, vehicle3, vehicle4]
for vehicle in vehicles:
    vehicle.define_knots(knot_intervals=10)
fleet = Fleet(vehicles)

rel_pos = 2.
configuration = np.array([[k*rel_pos] for k, veh in enumerate(vehicles)])
init_positions = [0.] + configuration
terminal_positions = [200.] + configuration
fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Rectangle(width=140., height=20.)})
rectangle = Rectangle(width=3., height=0.2)

# create a point-to-point problem
options = {'admm': {'rho': 0.01}, 'horizon_time': 10}
problem = FormationPoint2point(fleet, environment, options=options)
problem.set_options({'solver': {'ipopt.linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
# fleet.plot('input', knots=True, prediction=True, labels=['u'])
fleet.plot('state', knots=True, prediction=True, labels=['position (m)', 'velocity (m/s)', 'acceleration (m/s2)'])

# run it!
simulator.run()
# simulator.run_once()

# show/save some results
# problem.plot_movie('scene', repeat=False)
# vehicle.plot_movie('input', repeat=False, knots=True,
#                    labels=['v_x (m/s)', 'v_y (m/s)'])
# problem.save_movie('scene', axis=False)
# vehicle.save_plot('input', time=3., knots=True)
import matplotlib.pyplot as plt
plt.show(block=True)
