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

# create fleet
N = 2
vehicles = [Holonomic() for l in range(N)]

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
configuration = np.array([[-0.2, 0], [0.2, 0.]])

# init_positions = [-1.5, -1.5] + configuration
init_positions = [-1.5, -1.5] + np.array([[0, 0.3], [0, 0.1], [0, -0.1], [0, -0.3]])
# init_positions = [-1.5, -1.5] + np.array([[-0.4*l - 0.2*N, 0] for l in range(N)])
init_positions = [-1.5, -1.5] + np.array([[-0.1 , -0.1], [0.1, 0.1]])
terminal_positions = [2., 2.] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())

fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(5.)})

# create a formation point-to-point problem
options = {'rho': 0.5, 'horizon_time': 10, 'init_iter': 20}
problem = FormationPoint2point(fleet, environment, options=options)
problem.init()

# create simulator
simulator = Simulator(problem)
fleet.plot('input', knots=True, predict=True, labels=['v_x (m/s)', 'v_y (m/s)'])
# problem.plot('scene')
# problem.plot('residuals')

# run it!
simulator.run_once()
problem.plot_movie('scene')
