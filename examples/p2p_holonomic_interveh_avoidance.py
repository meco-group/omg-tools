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

# create vehicle
N = 2
vehicles = [Holonomic() for k in range(N)]

for k, vehicle in enumerate(vehicles):
    vehicle.set_initial_conditions([1.5*np.cos((k*2.*np.pi)/N), 1.5*np.sin((k*2.*np.pi)/N)])
    vehicle.set_terminal_conditions([-1.5*np.cos((k*2.*np.pi)/N), -1.5*np.sin((k*2.*np.pi)/N)])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

# create a point-to-point problem
problem = Point2point(vehicles, environment, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.set_options({'inter_vehicle_avoidance': True})
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicles[0].plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
vehicles[1].plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
