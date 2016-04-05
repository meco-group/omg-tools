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

"""
This file demonstrates how to export a point2point problem to c++. It generates
some source and header files which can be compiled with your own source code or
which can be compiled to a shared library and included in your own project.
"""

# create vehicle
options = {'room_constraint': None}
vehicle = Holonomic(shapes=Circle(0.1), options=options)

vehicle.set_initial_conditions([0.0, 0.0])
vehicle.set_terminal_conditions([4.0, 4.0])

# create environment
environment = Environment(room={'shape': Square(6.)})
rectangle = Rectangle(width=3., height=0.2)

environment.add_obstacle(Obstacle({'position': [0.5, 2.0]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [4.2, 2.0]}, shape=rectangle))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

options = {}
casadi_path = '/home/ruben/Documents/Work/Repositories/casadi_binary'
options['directory'] = os.path.join(os.getcwd(), 'export/')
# path to object files of your exported optimization problem
options['casadiobj'] = os.path.join(options['directory'], 'bin/')
# your casadi include path
options['casadiinc'] = os.path.join(casadi_path, 'include/')
# your casadi library path
options['casadilib'] = os.path.join(casadi_path, 'casadi/')

# export the problem
problem.export(options)

# note: you need to implement your vehicle type in c++. Take a look at
# Holonomic.cpp and Holonomic.hpp which are also exported as an example.
