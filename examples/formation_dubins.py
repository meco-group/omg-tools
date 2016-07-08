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
N = 3
vehicles = [Dubins(options={'degree': 2}, bounds={'vmax': 1., 'wmax': 30., 'wmin': -30.}) for l in range(N)]
for vehicle in vehicles:
    vehicle.define_knots(knot_intervals=10)

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.2, N, np.pi).vertices.T
init_positions = [-0.5, -1.5] + configuration
terminal_positions = [0.5, 1.5] + configuration
init_pose = np.c_[init_positions, 90.*np.ones(N)]
terminal_pose = np.c_[terminal_positions, 90.*np.ones(N)]

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_pose.tolist())
fleet.set_terminal_conditions(terminal_pose.tolist())

# create environment
environment = Environment(room={'shape': Square(4.)})
beam1 = Beam(width=3., height=0.2, orientation=np.pi/2)
environment.add_obstacle(Obstacle({'position': [0., -2.2]}, shape=beam1))
environment.add_obstacle(Obstacle({'position': [0., 2.2]}, shape=beam1))

# create a formation point-to-point problem
options = {'rho': 5., 'horizon_time': 5., 'hard_term_con': True}
problem = FormationPoint2point(fleet, environment, options=options)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
fleet.plot('input', knots=True, labels=['v (m/s)', 'w (rad/s)'])
fleet.plot('state', knots=True, labels=['x', 'y', 'theta'])
fleet.plot('fleet_center')

# run it!
simulator.run()
problem.plot_movie('scene', number_of_frames=100)
