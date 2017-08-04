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

import time
from matplotlib import pyplot as plt
import numpy as np

class GlobalPlanner:
    def __init__(self, environment):
        pass

    def get_path(self, environment, curr_state, goal_state):
        # Implement this in the child classes
        pass

    def move_point_to_grid(x,y):
        # move a point in world coordinates to the closest grid point
        pass

class QuadmapPlanner(GlobalPlanner):
    # global planner using a quadmap
    def __init__(self,environment):
        GlobalPlanner.__init__(self, environment)
        raise NotImplementedError('Please implement this method!')

    def get_path(self, environment, curr_state, goal_state):
        raise NotImplementedError('Please implement this method!')

class AStarPlanner(GlobalPlanner):
    # global planner using the A*-algorithm
    def __init__(self, environment, n_cells, start, goal, options={}):
        if isinstance(environment.rooms[0]['shape'], (Rectangle, Square)):
            grid_width = environment.rooms[0]['shape'].width
            grid_height = environment.rooms[0]['shape'].height
            if 'position' in environment.rooms[0]:
                grid_position = environment.rooms[0]['position']
            else:
                grid_position = [0, 0]
        else:
            raise RuntimeError('Environment has invalid room shape, only Rectangle or Square is supported')

        # check if vehicle size needs to be taken into account while searching a global path
        self.veh_size = options['veh_size'] if 'veh_size' in options else [0.,0.]

        # make grid
        if ((grid_width == grid_height) and (n_cells[0] == n_cells[1])):
            self.grid = SquareGrid(size=grid_width, position=grid_position, n_cells=n_cells, offset=self.veh_size)
        else:
            self.grid = Grid(width=grid_width, height=grid_height, position=grid_position, n_cells=n_cells, offset=self.veh_size)

        # occupy grid cells based on environment
        blocked = self.grid.get_occupied_cells(environment)
        self.grid.block(blocked)

        # only grid points are reachable so move start and goal for global planner
        self.start = self.grid.move_to_gridpoint(start)
        self.goal = self.grid.move_to_gridpoint(goal)

        # calculate diagonal moving cost for grid
        theta = np.arctan(float(self.grid.cell_height)/self.grid.cell_width)
        self.diag_cost = self.grid.cell_width / np.cos(theta)

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
        h_cost_x = abs(self.goal[0] - node.pos[0])*self.grid.cell_width
        h_cost_y = abs(self.goal[1] - node.pos[1])*self.grid.cell_height
        h_cost = h_cost_x + h_cost_y
        return h_cost

    def calculate_f_cost(self, node):
        return node.g_cost + node.h_cost

    def get_lowest_f_cost_node(self):
        # find the cheapest node to go to
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

    def get_path(self, start=None, goal=None):
        # main function of the A* algorithm
        t1 = time.time()
        if start is not None:
            # only grid points are reachable
            self.start = self.grid.move_to_gridpoint(start)
        if goal is not None:
            self.goal = self.grid.move_to_gridpoint(goal)

        # initialize A*-algorithm
        self.current_node = self.create_node(self.start)
        self.open_list = []
        self.closed_list = [self.current_node]

        while self.current_node.pos != self.goal:
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

            if not self.open_list:
                # current node is not the goal, and open list is empty
                # check if there are neigbors left which you can reach and are not yet visited
                neighbors = self.grid.get_neighbors(self.current_node.pos)
                closed_list_pos = []
                for node in self.closed_list:
                    closed_list_pos.append(node.pos)
                if not neighbors or all(item in neighbors for item in closed_list_pos):
                    # there are no neighbors which are accessible or they are all in the closed list,
                    # meaning that no path could be found
                    raise RuntimeError('There is no path from the desired start to the desired end node!')

        t2 = time.time()
        print 'Elapsed time to find a global path: ', t2-t1

        # convert a set of nodes to a set of positions
        path = self.closed_list_to_path()
        nodes_pos = []
        for node in path:
            nodes_pos.append(node.pos)
        nodes_pos.reverse()

        # convert node positions (indices) to waypoint positions (physical values)
        path = self.convert_node_to_waypoint(nodes_pos)

        return path

    def closed_list_to_path(self):
        # convert closed list to a list of nodes, by moving over the parent of each node
        current_node = self.closed_list[-1]
        path = [current_node]
        while current_node != self.closed_list[0]:
            next_node = current_node.parent
            path.append(next_node)
            current_node = next_node
        return path

    def convert_node_to_waypoint(self, nodes):
        # convert position of node (i.e. an index in a grid) to a physical position [m]
        waypoints = []
        if not isinstance (nodes[0], list):  # make something like [[0,1]]
            nodes = [nodes]
        for node in nodes:
            waypoint = [0,0]
            waypoint[0] = self.grid.position[0] - self.grid.width*0.5 + self.grid.cell_width*0.5 + node[0]*self.grid.cell_width
            waypoint[1] = self.grid.position[1] - self.grid.height*0.5 + self.grid.cell_height*0.5 + node[1]*self.grid.cell_height
            waypoints.append(waypoint)
        return waypoints

    def plot_path(self, path):
        # plot the computed path
        posx = []
        posy = []
        for waypoint in path:
            posx.append(waypoint[0])
            posy.append(waypoint[1])
        plt.plot(posx,posy)
        plt.show()

class Node:
    def __init__(self, position, parent=None):
        self.pos = position  # index of the point in the grid
        self.parent = parent  # how did you end up in this node
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
    def __init__(self, width, height, position, n_cells, offset=[0.,0.]):
        self.occupied = []  # initialize grid as empty
        self.width = width
        self.height = height
        self.position = position
        self.n_cells = n_cells  # number of cells in horizontal and vertical direction
        self.cell_width = self.width*1./self.n_cells[0]
        self.cell_height = self.height*1./self.n_cells[1]

        self.offset = offset  # blows up obstacles, e.g. to take the vehicle size into account in the grid

    def in_bounds(self, point):
        x, y = point
        # cell number starts counting at 0, until n_cells-1
        return 0 <= x < self.n_cells[0] and 0 <= y < self.n_cells[1]

    def block(self, points):
        # block cells given by indices/position in grid
        if len(points) == 2 and isinstance(points[0], (int)):
            points = [points]
        for point in points:
            if self.in_bounds(point):
                # only add points which are in the bounds
                self.occupied.append(point)

    def free(self, point):
        # check if a gridpoint is free
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
            if (point1[0] + 1 == point2[0] and point1[1] + 1 == point2[1]):
                if (self.free([point1[0], point1[1] + 1]) and self.free([point1[0] + 1, point1[1]])):
                    accessible = True
            # diagonal up left
            elif (point1[0] - 1 == point2[0] and point1[1] + 1 == point2[1]):
                if (self.free([point1[0], point1[1] + 1]) and self.free([point1[0] - 1,point1[1]])):
                    accessible = True
            # diagonal down right
            elif (point1[0] + 1 == point2[0] and point1[1] - 1 == point2[1]):
                if (self.free([point1[0], point1[1] - 1]) and self.free([point1[0] + 1,point1[1]])):
                    accessible = True
            # diagonal down left
            elif (point1[0] - 1 == point2[0] and point1[1] - 1 == point2[1]):
                if (self.free([point1[0], point1[1] - self.cell_height]) and self.free([point1[0] - 1,point1[1]])):
                    accessible = True

        return accessible

    def move_to_gridpoint(self, point):
        # snap a certain point to the nearest unoccupied grid point
        # i.e. you go from [m] to a certain index in the grid

        # determine distance of point to all surrounding grid points
        # find the amount of cell widths/heights which fits in the point
        # this is the closest gridpoint
        moved_point = [0, 0]

        # remove offset, i.e. substract the bottom left gridpoint
        moved_point[0] = point[0] - (self.position[0] - 0.5*self.width + 0.5*self.cell_width)
        moved_point[1] = point[1] - (self.position[1] - 0.5*self.height + 0.5*self.cell_height)
        # determine how many times point fits in cell dimensions, this gives the indices in the grid
        moved_point[0] = int(round(float(moved_point[0])/self.cell_width))
        moved_point[1] = int(round(float(moved_point[1])/self.cell_height))

        if moved_point in self.occupied:
            # closest grid point is occupied, check all neighbours of this point
            points_to_check = [[moved_point[0]+1, moved_point[1]],
                               [moved_point[0]-1, moved_point[1]],
                               [moved_point[0], moved_point[1]+1],
                               [moved_point[0], moved_point[1]-1],
                               [moved_point[0]+1, moved_point[1]+1],
                               [moved_point[0]+1, moved_point[1]-1],
                               [moved_point[0]-1, moved_point[1]+1],
                               [moved_point[0]-1, moved_point[1]-1]]
            # remove inaccessible points from points_to_check
            points_to_check = filter(self.in_bounds, points_to_check)
            points_to_check = filter(self.free, points_to_check)
            # select closest point which is not occupied
            if points_to_check is not None:
                # worst case: only a diagonally placed cell is available
                # --> distance to it is sqrt(cell_width**2+cell_height**2)
                # use an upper bound on this to avoid taking sqrt a lot to
                # initialize d_min
                d_min = 2*max(self.cell_height, self.cell_width)
                for p in points_to_check:
                    distance = self.distance_between_cells(p, moved_point)
                    if distance < d_min:
                        d_min = distance
                        moved_point = p

        # convert position of moved_point to indices
        return moved_point

    def distance_between_cells(self, cell1, cell2):
        if cell1 == cell2:
            return 0
        elif cell1[0] == cell2[0]:
            return self.cell_height
        elif cell1[1] == cell2[1]:
            return self.cell_width
        else:
            return np.sqrt(self.cell_width**2 + self.cell_height**2)

    def get_neighbors(self, point):
        # get all the accessible neighbouring cells of a certain point
        x, y = point
        results = [[x+1, y], [x-1, y],
                   [x, y+1],[x, y-1],
                   [x-1, y+1], [x+1, y+1],
                   [x-1, y-1], [x+1, y-1]]
        results = filter(self.in_bounds, results)
        results = filter(self.free, results)
        results = filter(lambda x: self.is_accessible(point, x), results)
        return results

    def get_occupied_cells(self, environment):
        # blank out the grid points which are occupied by a certain obstacle
        occupied_cells = []
        cells = []
        centers_x = np.arange(self.position[0]-self.width*0.5 + 0.5*self.cell_width,
                              self.position[0]+self.width*0.5 + 0.5*self.cell_width,self.cell_width)
        centers_y = np.arange(self.position[1]-self.height*0.5 + 0.5*self.cell_height,
                              self.position[1]+self.height*0.5 + 0.5*self.cell_height, self.cell_height)
        i, j = 0, 0
        for x in centers_x:
            for y in centers_y:
                cells.append({'pos': [x,y], 'index': [i, j]})
                j += 1
            i += 1
            j = 0

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
                        if vertex_x[k] == min(vertex_x):
                            # take into account offset to avoid waypoints which are so close
                            # to obstacles that they are not reachable
                            v_x = vertex_x[k] + pos[0] - self.offset[0]
                        else:
                            # take into account offset to avoid waypoints which are so close
                            # to obstacles that they are not reachable
                            v_x = vertex_x[k] + pos[0] + self.offset[0]
                        if vertex_y[k] == min(vertex_y):
                            v_y = vertex_y[k] + pos[1] - self.offset[1]
                        else:
                            v_y = vertex_y[k] + pos[1] + self.offset[1]
                        vertices.append([v_x, v_y])
                if isinstance(obstacle.shape, Circle):
                    r = obstacle.shape.radius
                    # approximate circle by a square and add these vertices
                    # take into account offset, e.g. to avoid waypoints which
                    # are closer to obstacles than the vehicle can reach
                    vertices = [[pos[0] + r + self.offset[0], pos[1] + r + self.offset[1]],
                                [pos[0] + r + self.offset[0], pos[1] - r - self.offset[1]],
                                [pos[0] - r - self.offset[0], pos[1] + r + self.offset[1]],
                                [pos[0] - r - self.offset[0], pos[1] - r - self.offset[1]]]
                vertices = np.array(vertices)
                vertices = np.round(vertices, 4)  # rounding off vertex positions, for easier comparison below

                occ_cells = []
                for cell in cells:
                    blocked = False  # boolean to indicate if cell is blocked
                    # calculate cell vertices
                    cell_vertices = []
                    cell_vertices.append([cell['pos'][0] - 0.5*self.cell_width, cell['pos'][1] - 0.5*self.cell_height])
                    cell_vertices.append([cell['pos'][0] + 0.5*self.cell_width, cell['pos'][1] - 0.5*self.cell_height])
                    cell_vertices.append([cell['pos'][0] - 0.5*self.cell_width, cell['pos'][1] + 0.5*self.cell_height])
                    cell_vertices.append([cell['pos'][0] + 0.5*self.cell_width, cell['pos'][1] + 0.5*self.cell_height])
                    cell_vertices = np.array(cell_vertices)
                    # cell center is inside the convex hull of the vertices of an obstacle
                    if (min(vertices[:,0]) < cell['pos'][0] < max(vertices[:,0]) and
                        min(vertices[:,1]) < cell['pos'][1] < max(vertices[:,1])):
                        occ_cells.append(cell)
                    else:
                        # check if any of the cell vertices are inside the convex hull of the obstacle vertices
                        for cell_v in cell_vertices:
                            if (min(vertices[:,0]) < cell_v[0] < max(vertices[:,0]) and
                                min(vertices[:,1]) < cell_v[1] < max(vertices[:,1])):
                                    # avoid adding the same cell multiple times
                                    if cell not in occ_cells:
                                        occ_cells.append(cell)
                                    blocked = True  # cell is blocked, go to next cell
                                    break
                        # check if any of the obstacle vertices are inside the convex hull of the cell vertices
                        if not blocked:  # cell was not detected as blocked yet
                            for v in vertices:
                                if (min(cell_vertices[:,0]) < v[0] < max(cell_vertices[:,0]) and
                                    min(cell_vertices[:,1]) < v[1] < max(cell_vertices[:,1])):
                                        # avoid adding the same cell multiple times
                                        if cell not in occ_cells:
                                            occ_cells.append(cell)
                                        break  # one obstacle vertex is inside the cell, go to next cell
                # if cell is found to be occupied, remove it, i.e. don't check again for next obstacle
                for cell in occ_cells:
                    cells.remove(cell)
                # add cells which are occupied by the obstacle to the collection of occupied cells
                occupied_cells.extend(occ_cells)

        # only return the indices of the occupied cells
        occ_cells = []
        for cell in occupied_cells:
            occ_cells.append(cell['index'])
        return occ_cells

    def draw(self):
        # draw the grid
        plt.figure()
        #plot centers
        centers_x = np.arange(self.position[0]-self.width*0.5 + 0.5*self.cell_width,
                              self.position[0]+self.width*0.5 + 0.5*self.cell_width,self.cell_width)
        centers_y = np.arange(self.position[1]-self.height*0.5 + 0.5*self.cell_height,
                              self.position[1]+self.height*0.5 + 0.5*self.cell_height, self.cell_height)
        i, j = 0, 0
        for x in centers_x:
            for y in centers_y:
                if [i,j] not in self.occupied:
                    plt.plot(x,y,'ro')
                j += 1
            i += 1
            j = 0
        #plot grid lines
        x_bottom = self.position[0] - 0.5*self.width
        x_top = self.position[0] + 0.5*self.width
        y_bottom = self.position[1] - 0.5*self.height
        y_top = self.position[1] + 0.5*self.height
        # make int because number of lines can only be an integer
        for k in range(int(self.width/self.cell_width)+1):
            x_point = x_bottom + k*self.cell_width
            plt.plot([x_point,x_point], [y_bottom,y_top], 'r-')
        for k in range(int(self.height/self.cell_height)+1):
            y_point = y_bottom + k*self.cell_height
            plt.plot([x_bottom,x_top], [y_point,y_point], 'r-')
        plt.draw()

class SquareGrid(Grid):
    # special case of a normal Grid, width = height
    def __init__(self, size, position, n_cells, offset=[0.,0.]):
        # make a general grid, with square cell
        Grid.__init__(self, size, size, position, n_cells, offset)