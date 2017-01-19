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

from ..basics.shape import Rectangle, Square, Circle

from matplotlib import pyplot as plt
import numpy as np

class GlobalPlanner:
    def __init__(self, environment):
        print 'bla'

    def get_path(self, environment, curr_state, goal_state):
        # Implement this in the child classes
        print 'bla'
        return []

    def movePoint2Grid(x,y):
        # move a point in world coordinates to the closest grid point

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

class AStarPlanner(GlobalPlanner):
    def __init__(self, environment, cell_size, start, goal):

        if isinstance(environment.room['shape'], (Rectangle,Square)):
            grid_width = environment.room['shape'].width
            grid_height = environment.room['shape'].height
            if 'position' in environment.room:
                grid_position = environment.room['position']
            else:
                grid_position = [0, 0]
        else:
            raise RuntimeError('Environment has invalid room shape, only Rectangle or Square is supported')

        # make grid        
        if not isinstance(cell_size, list):
            cell_size = [cell_size]
        if (len(cell_size) == 1 or len(cell_size == 2) and cell_size[0] == cell_size[1]):
            self.grid = SquareGrid(width=grid_width, height=grid_height, position=grid_position, square_size=cell_size[0])
        elif len(grid_size == 2):
            self.grid = Grid(width=grid_width, height=grid_height, position=grid_position, cell_width=cell_size[0], cell_height=cell_size[1])
        else:
            raise ValueError('Invalid grid_size entered, use maximum two numbers (width and height)')

        # only grid points are reachable
        self.start = self.grid.move_to_gridpoint(start)
        self.goal = self.grid.move_to_gridpoint(goal)

        # initialize A*-algorithm
        self.current_node = self.create_node(self.start)
        self.open_list = []
        self.closed_list = [self.current_node]

        # diagonal moving cost for grid
        theta = np.arctan(self.grid.cell_height/self.grid.cell_width)
        self.diag_cost = self.grid.cell_width / np.cos(theta)

        # occupy grid cells based on environment
        blocked = self.grid.get_occupied_cells(environment, cell_size)
        self.grid.block(blocked)

    def set_start(self, start):
        self.start = start

    def set_goal(self, goal):
        self.goal = goal

    def calculate_g_cost(self, node):
        if node.parent is not None:
            # get movement direction from parent to node
            x,y = node.pos
            par_x, par_y = node.parent.pos
            if x == par_x and y == par_y:
                raise ValueError('Parent and node have the same position, something is wrong!')
            if x != par_x and y == par_y:
                # horizontal movement
                g_cost = self.grid.cell_width
            elif x == par_x and y != par_y: 
                # verical movement
                g_cost = self.grid.cell_height
            elif x != par_x and y != par_y:
                # diagonal movement
                g_cost = self.diag_cost
            g_cost += node.parent.g_cost
        return g_cost

    def calculate_h_cost(self, node):
        # h_cost is determined by horizontal and vertical distance from current node to goal node
        # this is called the Manhattan way of determining the h cost
        h_cost_x = abs(self.goal[0] - node.pos[0])
        h_cost_y = abs(self.goal[1] - node.pos[1])
        h_cost = h_cost_x + h_cost_y

        return h_cost

    def calculate_f_cost(self, node):
        return node.g_cost + node.h_cost

    def get_lowest_f_cost_node(self):
        cost = np.inf
        cheapest_node = None
        for node in self.open_list:
            if node.f_cost < cost:
                cost = node.f_cost
                cheapest_node = node

        return cheapest_node

    def remove_from_open_list(self, node):
        for n in self.open_list:
            if n == node:
                self.open_list.remove(n)
                break

    def create_node(self, point, parent=None):
        # creates a node on the location of point
        return Node(point, parent)

    def get_path(self):
        while not self.current_node.pos == self.goal:
            # get positions of current node neighbours
            neighbors = self.grid.get_neighbors(self.current_node.pos)
            for point in neighbors:
                # suppose that the gridpoint is not yet seen
                new_point = True
                for n in self.closed_list:
                    if point == n.pos:
                        # point is already in closed list
                        new_point = False
                        break
                for n in self.open_list:
                    if point == n.pos:
                        # point is already in open list
                        dummy_node = Node(point, parent=self.current_node)
                        new_g_cost = self.calculate_g_cost(dummy_node)
                        if new_g_cost <= n.g_cost:
                            n.parent = self.current_node
                            n.g_cost = new_g_cost
                            n.h_cost = self.calculate_h_cost(n)
                            n.f_cost = self.calculate_f_cost(n)
                        new_point = False
                        break
                if new_point:
                    # make a node for the point
                    new_node = self.create_node(point)
                    new_node.parent = self.current_node
                    new_node.g_cost = self.calculate_g_cost(new_node)
                    new_node.h_cost = self.calculate_h_cost(new_node)
                    new_node.f_cost = self.calculate_f_cost(new_node)
                    self.open_list.append(new_node)

            self.current_node = self.get_lowest_f_cost_node()
            self.remove_from_open_list(self.current_node)
            self.closed_list.append(self.current_node)
            print self.current_node.pos

            if not self.open_list:
                # open list was empty, meaning that no path could be found
                raise RuntimeError('There is no path from the desired start to the desired end node!')

        path = self.closed_list_to_path()
        waypoints = []
        for node in path:
            waypoints.append(node.pos)
        waypoints.reverse()
        # temporary plotting
        self.grid.draw()
        self.plot_path(path)
        import pdb; pdb.set_trace()  # breakpoint b6d486eb //
        return waypoints

    def closed_list_to_path(self):
        current_node = self.closed_list[-1]
        path = [current_node]
        while current_node != self.closed_list[0]:
            next_node = current_node.parent
            path.append(next_node)
            current_node = next_node
        return path

    def plot_path(self, path):
        posx = []
        posy = []
        for node in path:
            posx.append(node.pos[0])
            posy.append(node.pos[1])
        plt.figure(1)
        plt.plot(posx,posy)
        plt.show()

class Node:
    def __init__(self, position, parent=None):
        self.pos = position
        self.parent = parent
        self.f_cost = 0
        self.g_cost = 0
        self.h_cost = 0

    def get_parent(self):
        return self.parent

    def get_pos(self):
        return self.pos

    def get_g_cost(self):
        return self.g_cost

    def get_h_cost(self):
        return self.h_cost

    def get_f_cost(self):
        return self.f_cost

class Grid:
    # based on: http://www.redblobgames.com/pathfinding/a-star/implementation.html
    def __init__(self, width, height, position, cell_width, cell_height):
        self.occupied = []  # initialize grid as empty
        self.width = width
        self.height = height
        self.position = position
        if width % cell_width != 0:
            raise ValueError('Selected width must be a multiple of square size')
        elif height % cell_height != 0:
            raise ValueError('Selected height must be a multiple of square size')
        else:
            self.cell_width = cell_width  # width of a cell
            self.cell_height = cell_height  # height of a cell
    
    def in_bounds(self, point):
        x, y = point
        return self.position[0] - 0.5*self.width <= x < self.position[0] + 0.5*self.width and self.position[1] - 0.5*self.height <= y < self.position[1] + 0.5*self.height
    
    def block(self, points):
        # block cells given by points
        if len(points) == 2 and isinstance(points[0], (int, float)):
            points = [points]
        for point in points:
            if self.in_bounds(point):
                # only add points which are in the bounds
                self.occupied.append(point)                

    def free(self, point):
        # check if a point is free
        # i.e.: not occupied and in bounds

        free = False
        if ((not point in self.occupied) and (self.in_bounds(point))):
           free = True
        return free

    def is_accessible(self, point1, point2):
        # Check if you can reach point2 from point1. Diagonal movement along
        # an edge of an occupied point is not allowed
        # graphical illustration:
        #  1----2----3
        #  |    |    |
        #  4----x----6 
        #  |    |    |
        #  7----8----9
        # if x represents an occupied point, then moving from e.g. 8 to 6 is not possible

        accessible = True
        # only possible to be free but not accessible if diagonal movement
        if (point1[0] != point2[0] and point1[1] != point2[1]):
            # if diagonal movement accessible may be False
            accessible = False
            # diagonal up right
            if (point1[0] + self.cell_width == point2[0] and point1[1] + self.cell_height == point2[1]):
                if (self.free([point1[0], point1[1] + self.cell_height]) and self.free([point1[0] + self.cell_width,point1[1]])):
                    accessible = True
            # diagonal up left
            elif (point1[0] - self.cell_width == point2[0] and point1[1] + self.cell_height == point2[1]):
                if (self.free([point1[0], point1[1] + self.cell_height]) and self.free([point1[0] - self.cell_width,point1[1]])):
                    accessible = True
            # diagonal down right
            elif (point1[0] + self.cell_width == point2[0] and point1[1] - self.cell_height == point2[1]):
                if (self.free([point1[0], point1[1] - self.cell_height]) and self.free([point1[0] + self.cell_width,point1[1]])):
                    accessible = True
            # diagonal down left
            elif (point1[0] - self.cell_width == point2[0] and point1[1] - self.cell_height == point2[1]):
                if (self.free([point1[0], point1[1] - self.cell_height]) and self.free([point1[0] - self.cell_width,point1[1]])):
                    accessible = True
        return accessible
    
    def move_to_gridpoint(self, point):
        # start on first grid point and add the amount of cell widths/heights
        # which fits in the point, do this -1 because you start on the first grid point
        moved_point = [0, 0]
        moved_point[0] = self.position[0] - 0.5*self.width + 0.5*self.cell_width +  self.cell_width * (round(float(point[0])/self.cell_width)-1)
        moved_point[1] = self.position[1] - 0.5*self.height + 0.5*self.cell_height +  self.cell_height * (round(float(point[1])/self.cell_height)-1)
        return moved_point

    def get_neighbors(self, point):
        x, y = point
        results = [[x+self.cell_width, y], [x-self.cell_width, y],
                   [x, y+self.cell_height],[x, y-self.cell_height],
                   [x-self.cell_width, y+self.cell_height], [x+self.cell_width, y+self.cell_height],
                   [x-self.cell_width, y-self.cell_height], [x+self.cell_width, y-self.cell_height]]
        results = filter(self.in_bounds, results)
        results = filter(self.free, results)
        results = filter(lambda x: self.is_accessible(point, x), results)
        # results = filter(self.is_accessible, results, point)

        return results

    def get_occupied_cells(self, environment, cell_size):
        occupied_cells = []
        cells = []
        centers_x = np.arange(self.position[0]-self.width*0.5 + 0.5*self.cell_width,self.position[0]+self.width*0.5 + 0.5*self.cell_width,self.cell_width)
        centers_y = np.arange(self.position[1]-self.height*0.5 + 0.5*self.cell_height, self.position[1]+self.height*0.5 + 0.5*self.cell_height, self.cell_height)
        for x in centers_x:
            for y in centers_y:
                cells.append([x,y])
        
        for obstacle in environment.obstacles:
            # only look at stationary obstacles
            if ((not 'trajectories' in obstacle.simulation) or (not 'velocity' in obstacle.simulation['trajectories'])
               or (all(vel == [0.]*obstacle.n_dim for vel in obstacle.simulation['trajectories']['velocity']['values']))):
                pos = obstacle.signals['position'][:,-1]
                if isinstance(obstacle.shape, (Rectangle, Square)):
                    vertices = []
                    vertex_x = obstacle.shape.vertices[0]
                    vertex_y = obstacle.shape.vertices[1]
                    for k in range(len(vertex_x)):
                        v_x = vertex_x[k] + pos[0]
                        v_y = vertex_y[k] + pos[1]
                        vertices.append([v_x, v_y])
                if isinstance(obstacle.shape, Circle):
                    r = obstacle.radius
                    # approximate circle by a square and add these vertices
                    vertices = [[pos[0]+radius, pos[1]+radius],
                                [pos[0]+radius, pos[1]-radius],
                                [pos[0]-radius, pos[1]+radius],
                                [pos[0]-radius, pos[1]-radius]]
                vertices = np.array(vertices)

                occ_cells = []
                for cell in cells:
                    # one of the vertices is inside a cell
                    cell_vertices = []
                    cell_vertices.append([cell[0] - 0.5*self.cell_width, cell[1] - 0.5*self.cell_height])
                    cell_vertices.append([cell[0] + 0.5*self.cell_width, cell[1] - 0.5*self.cell_height])
                    cell_vertices.append([cell[0] - 0.5*self.cell_width, cell[1] + 0.5*self.cell_height])
                    cell_vertices.append([cell[0] + 0.5*self.cell_width, cell[1] + 0.5*self.cell_height])
                    # cell is inside the vertices of an obstacle
                    if (min(vertices[:,0]) <= cell[0] <= max(vertices[:,0]) and 
                        min(vertices[:,1]) <= cell[1] <= max(vertices[:,1])):
                        occ_cells.append(cell)
                    else:
                        for cell_v in cell_vertices:
                            if (min(vertices[:,0]) <= cell_v[0] <= max(vertices[:,0]) and 
                                min(vertices[:,1]) <= cell_v[1] <= max(vertices[:,1])):
                                    occ_cells.append(cell)
                                    break
                    # if one of the vertices is inside the cell, add the cell and go to next cell
                    # if (cell in occ_cells):
                    #     break
                # if cell is found to be occupied, remove it, don't check again for next obstacle
                for cell in occ_cells:
                    cells.remove(cell)
                # add cells which are occupied by the obstacle to the collection of occupied cells
                occupied_cells.extend(occ_cells)

        return occupied_cells

    def draw(self):
        plt.figure()
        #plot centers
        centers_x = np.arange(self.position[0]-self.width*0.5 + 0.5*self.cell_width,self.position[0]+self.width*0.5 + 0.5*self.cell_width,self.cell_width)
        centers_y = np.arange(self.position[1]-self.height*0.5 + 0.5*self.cell_height, self.position[1]+self.height*0.5 + 0.5*self.cell_height, self.cell_height)
        for x in centers_x:
            for y in centers_y:
                if [x,y] not in self.occupied:
                    plt.plot(x,y,'ro')
        #plot grid lines
        x_bottom = self.position[0] - 0.5*self.width
        x_top = self.position[0] + 0.5*self.width
        y_bottom = self.position[1] - 0.5*self.height
        y_top = self.position[1] + 0.5*self.height
        for k in range(self.width/self.cell_width+1):
            x_point = x_bottom + k*self.cell_width
            plt.plot([x_point,x_point], [y_bottom,y_top], 'r-')
        for k in range(self.height/self.cell_height+1):
            y_point = y_bottom + k*self.cell_height
            plt.plot([x_bottom,x_top], [y_point,y_point], 'r-')
        plt.draw()

class SquareGrid(Grid):
    
    def __init__(self, width, height, position, square_size):
        # make a general grid, with square cell
        Grid.__init__(self, width, height, position, square_size, square_size)