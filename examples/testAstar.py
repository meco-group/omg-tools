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

start = [2,2]
goal = [8,8]
width = 10
height = 10
square_size = 0.5

environment = Environment(room={'shape': Rectangle(width=width, height=height), 'position':[5,5]})
environment.add_obstacle(Obstacle({'position': [5, 5]}, shape=Rectangle(width=2,height=2)))

planner = AStarPlanner(environment, [10,10], start, goal)
waypoints = planner.get_path()
planner.grid.draw()
planner.plot_path(waypoints)
# put breakpoint here to see the plot
print waypoints