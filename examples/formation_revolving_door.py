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

# create fleet
N = 4
vehicles = [Holonomic() for l in range(N)]

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.2, N, np.pi/4).vertices.T
# configuration = np.array([[0., 0.2], [0., -0.2]])
init_positions = [0., -2.0] + configuration
terminal_positions = [0., 2.0] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(5.)})
beam1 = Beam(width=2.2, height=0.2)
environment.add_obstacle(Obstacle({'position': [-2., 0.]}, shape=beam1))
environment.add_obstacle(Obstacle({'position': [2., 0.]}, shape=beam1))

beam2 = Beam(width=1.4, height=0.2)
horizon_time = 10.
omega = 1.5*(2*np.pi/horizon_time)
velocity = [0., 0.]
# velocity = [0., -0.2] # crazy revolving door
environment.add_obstacle(Obstacle({'position': [0., 0.], 'velocity': velocity,
    'angular_velocity': omega}, shape=beam2, simulation={},
    options={'horizon_time': horizon_time}))
environment.add_obstacle(Obstacle({'position': [0., 0.], 'velocity': velocity,
    'orientation': 0.5*np.pi, 'angular_velocity': omega},
    shape=beam2, simulation={}, options={'horizon_time': horizon_time}))

# create a formation point-to-point problem
options = {'rho': 2., 'horizon_time': 10}
problem = FormationPoint2point(fleet, environment, options=options)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
fleet.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
