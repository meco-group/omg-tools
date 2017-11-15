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


# This example tests a separate multiframeproblem. Normally, a schedulerproblem creates
# a multiframeproblem, and solves it with a receding horizon. This example allows testing
# the behavior of a multiframeproblem on itself. Beware that this problem will not solve
# completely if using simulator.run(), this is because the amount of frames that are coupled
# stays the same. E.g. when coupling two frames, normally the first frame is dropped
# (by the scheduler) as soon as the vehicle enters the second frame, this is not possible
# when considering only a single multiframeproblem. Therefore, the vehicle cannot leave
# the first frame.

# The frames that are considered here are of the type 'corridor', i.e. frames that are as
# large as possible, without containing any stationary obstacle.

from omgtools import *

# create vehicle
vehicle = Holonomic(shapes = Circle(radius=0.5), options={'syslimit': 'norm_2', 'stop_tol': 1.e-2},
                    bounds={'vmax': 2, 'vmin':-2, 'amax':4, 'amin':-4})
# create environment
start = [5,0]
goal = [25,20]
vehicle.set_initial_conditions(start)
vehicle.set_terminal_conditions(goal)

room1 = {'shape': Rectangle(width=30, height=10), 'position': [15, 0], 'draw':True}
room2 = {'shape': Rectangle(width=10, height=30), 'position': [25, 10], 'draw':True}
room3 = {'shape': Rectangle(width=40, height=10), 'position': [40, 20], 'draw':True}

environment = Environment(room=[room1, room2])

# create stationary obstacles

rectangle = Rectangle(width=2., height=2)
rectangle1 = Rectangle(width=0.2, height=0.2)

obstacle1 = Obstacle({'position': [10,0]}, shape=rectangle)
trajectories = {'velocity': {'time': [0.], 'values': [[0, -0.]]}}
obstacle2 = Obstacle({'position': [22.5,12.5]}, shape=rectangle, simulation={'trajectories': trajectories})
# environment.add_obstacle([obstacle1, obstacle2])

# Warning: if you use fill room, then also fill the empty rooms with []
environment.fill_room(room1, [])#[obstacle1])
environment.fill_room(room2, [])#[obstacle2])
# environment.fill_room(room3, [])

# create moving obstacles
# make global planner

# make problem
multiframeproblem=MultiFrameProblem(vehicle, environment, n_frames=2)
multiframeproblem.init()

# simulate the problem
simulator = Simulator(multiframeproblem)

# define what you want to plot
multiframeproblem.plot('scene')
# vehicle.plot('state')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
vehicle.plot('dinput')

# run it!
simulator.run_once()

multiframeproblem.plot_movie('scene', number_of_frames=100, repeat=False)