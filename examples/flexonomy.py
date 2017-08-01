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

# create vehicle
vehicle = Holonomic()
vehicle.set_options({'safety_distance': 0.1})
vehicle.set_options({'ideal_prediction': False})

vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# extract spline parameters
coeffs = np.zeros([len(vehicle.basis.knots)-vehicle.basis.degree-1,2])
splineParams = {'knots':vehicle.basis.knots,'degree':vehicle.basis.degree,'coeffs':coeffs}

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
obstacles = Obstacle({'position': [1.5, 0.5]},shape=Circle(0.4))
obstacles.set_options({'splineTraj': True})
obstacles.set_options({'splineParams': splineParams})
environment.add_obstacle(obstacles)

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.init()

options = {}
options['directory'] = os.path.join(os.getcwd(), 'export/')
# path to object files of your exported optimization problem
options['casadiobj'] = os.path.join(options['directory'], 'bin/')
options['obstTraj'] = obstacles.options['splineTraj']
problem.export(options)

print 'Exported! :)'

