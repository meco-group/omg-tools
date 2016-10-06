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
sys.path.insert(0, os.getcwd()+"/..")
from omgtools import *

#create fleet
N = 4
vehicles = [Quadrotor3D(shapes=RegularPrisma(0.4, 0.15, 8)) for l in range(N)]

for vehicle in vehicles:
    vehicle.set_options({'stop_tol': 5e-1})
fleet = Fleet(vehicles)
configuration = Plate(Rectangle(1., 1.), height=0.1).vertices.T
init_positions = [-2., -2., -2.] + configuration
terminal_positions = [3., 3.,-3.] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Cube(10.)})
environment.add_obstacle(Obstacle(
    {'position': [1., 0, -2.5]}, shape=Cuboid(width=0.25, depth=10, height=5.)))

# create a point-to-point problem
options = {'horizon_time':12}
problem = FormationPoint2pointCentral(fleet, environment, options=options)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene', view=[20, -35])  # elevation and azimuth of cam
fleet.plot('state', knots=True)
fleet.plot('input', knots=True)
# run it!
simulator.run()

# Save a movie as gif: you need imagemagick for this!
#problem.save_movie('scene', format='gif', name='QUAD_FORM_3D', number_of_frames=100, movie_time=5, axis=False)
