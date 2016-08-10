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
import os
import csv

"""
This file demonstrates how to export a point2point problem to c++. It generates
some source and header files which can be compiled with your own source code or
which can be compiled to a shared library and included in your own project.
"""

# create fleet
N = 4
options = {'room_constraints': None}
vehicles = [Holonomic(shapes=Circle(0.1), options=options) for l in range(N)]

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [0.0, 0.0] + configuration
terminal_positions = [3.5, 3.5] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(5.), 'position': [1.5, 1.5]})
rectangle = Rectangle(width=3., height=0.2)

environment.add_obstacle(Obstacle({'position': [-0.6, 1.0]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3.2, 1.0]}, shape=rectangle))

# create a formation point-to-point problem
options = {'rho': 2., 'horizon_time': 10, 'init_iter': 5}
problem = FormationPoint2point(fleet, environment, options=options)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

options = {}
casadi_path = os.path.join(os.getenv('HOME'), 'casadi-py27-np1.9.1-v3.0.0')
# casadi_path = os.path.join(os.getenv('HOME'), 'Documents/Work/Repositories/casadi_binary')
options['directory'] = os.path.join(os.getcwd(), 'export_f/')
# path to object files of your exported optimization problem
options['casadiobj'] = os.path.join(options['directory'], 'bin/')
# your casadi include path
options['casadiinc'] = os.path.join(casadi_path, 'include/')
# your casadi library path
options['casadilib'] = os.path.join(casadi_path, 'casadi/')

# export the problem
problem.export(options)
simulator = Simulator(problem)
trajectories, signals = simulator.run()

# save results for check in c++
testdir = os.path.join(options['directory'], 'test')
if not os.path.isdir(testdir):
    os.makedirs(os.path.join(options['directory'], 'test'))
jump = int(simulator.update_time/simulator.sample_time)
with open(os.path.join(testdir, 'data_state.csv'), 'wb') as f:
    w = csv.writer(f)
    for v in range(len(vehicles)):
        for i in range(0, 50*jump, jump):
            for k in range(trajectories['vehicle'+str(v)]['state'][i].shape[0]):
                w.writerow(trajectories['vehicle'+str(v)]['state'][i][k, :])
with open(os.path.join(testdir, 'data_input.csv'), 'wb') as f:
    w = csv.writer(f)
    for v in range(len(vehicles)):
        for i in range(0, 50*jump, jump):
            for k in range(trajectories['vehicle'+str(v)]['input'][i].shape[0]):
                w.writerow(trajectories['vehicle'+str(v)]['input'][i][k, :])

# note: you need to implement your vehicle type in c++. Take a look at
# Holonomic.cpp and Holonomic.hpp which are also exported as an example.
