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

class GlobalPlanner:
    def __init__(self, environment):
        print 'bla'

    def get_path(self, environment, curr_state, goal_state):
        # Implement this in the child classes
        print 'bla'
        return []

    def movePoint2Grid(x,y):
        # move a point in woorld coordinates to the closest grid point

        return x_grid, y_grid

class QuadmapPlanner(GlobalPlanner):

    def __init__(self,environment):
        GlobalPlanner.__init__(self, environment)
        self.environment = environment
    
        # # initialization of the quad map
        # bb = qmap.Bbox((width, height))
        # self.quad = qmap.Quad(bb, depth=2)
        # self.quad.insert(bb, 0)
        # self.insertFactoryWalls()

        # # initialization of the viewer
        # self.v = viewer.Viewer(self.quad, 1000, 1000, self.quad.G)
        # self.v.update()
        # self.path = {}  # contain all current paths
        # self.pathLineStrings = {}  # contain all paths converted to lineString
        # self.dist = {}  # current travel distance of paths
        # self.obstacles = {}  # random obstacles traveling in the virtual world
        # self.robotgoals = {}  # endpoints of the robots
        # self.robotstates = {}  # current position of the robots (x,y,th,vx,vy,w) 
        # self.splineplannerdict = {}  # spline planners for the robots

    def get_path(self, environment, curr_state, goal_state):
        print 'bla'
        return []