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
from casadi import inf

# create fleet
N = 4
vehicles = [Holonomic() for l in range(N)]

for k, vehicle in enumerate(vehicles):
    vehicle.set_initial_conditions([-1. -0.5*N*0.5 + 0.5*k, -1.5])

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
terminal_positions = [2., 2.] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-1.8, 0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, 0.5]}, shape=rectangle))

# create a formation point-to-point problem
options = {'rho': 1., 'horizon_time': 15}
problem = FormationPoint2pointCentral(fleet, environment, options=options)
problem.set_options({'soft_formation': True})
problem.set_options({'soft_formation_weight': 100})
problem.set_options({'max_formation_deviation': inf})
problem.set_options({'inter_vehicle_avoidance': True})
problem.init()


# create simulator
simulator = Simulator(problem)
fleet.plot('input', knots=True, predict=True, labels=['v_x (m/s)', 'v_y (m/s)'])
problem.plot('scene')

# run it!
simulator.run()
