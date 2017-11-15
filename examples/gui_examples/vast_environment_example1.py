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


# This example is very similar to p2p_multiframeproblem.py, and uses the same principles
# The difference is that in the current example, a GUI is used to assist the user in building up
# the desired environment. For an explanation about how to use the GUI, see below.

from omgtools import *

# create vehicle
vehicle = Holonomic(shapes = Circle(radius=0.5), options={'syslimit': 'norm_2'},
                    bounds={'vmax': 1.2, 'vmin':-1.2, 'amax':10, 'amin':-10})
veh_size = vehicle.shapes[0].radius

# create environment with the help of a GUI

#####################################
####### how to use the GUI? #########
#####################################
# create obstacles:
# -) select the shape of the obstacle you want to place (Circle or Rectangle)
# -) set the size of this shape (width, height for Rectangle, radius for Circle)
# -) set the x- and y-velocity if the obstacle is moving
# -) check Bounce if you want the moving obstacle to bounce off other obstacles or off the wall
# -) click at the desired position in the grid
# obstacles are automatically placed such that their center corresponds to the intersection of 4 cells

# select start and goal:
# start = right click somewhere in the grid
# goal = right click again
# if you made a mistake you can again click two times to make a new start and goal

# buttons:
# -) the Remove button removes the obstacle which was placed most recently
# clicking this button multiple times removes more obstacles, in the reverse order
# of their creation (starting with the obstacle which was created most recently)
# -) the Save checkmark lets you save the current environment, in the file environment.pickle
# -) the Load button loads the most recently saved environment
# -) the Quit button closes the GUI and stops the example
# -) the Ready button closes the GUI and continues the example,
# by extracting your created environment from the GUI
# -) the LoadSVG button lets you load in an environment which is defined inside an SVG
# when loading an svg, the GUI asks how many pixels there are in 1m e.g. '10',
# and how many cells you like horizontally and vertically e.g. '20,20'
# the loaded obstacles are all stationary, but after loading an svg, you can manually add extra (moving) obstacles
# for drawing.svg use e.g. 25 pixels/m and 30,30 cells
# for maze_small.svg use e.g. 5 pixels/m and 30,30 cells
# for maze_big.svg use e.g. 2 pixels/m and 100,100 cells (consider raising vehicle velocity limits to speed up)
# -) at the far right and below the Save checkmark there are arrows, to scroll through the grid

# Note: LoadSVG only works for a limited type of svg-files for the moment, consisting of
# line, polyline, rect or circle objects

# Note: to run this example just click Load in the GUI and select vast_environment_example1.pickle

import Tkinter as tk
root = tk.Tk()
# the number of desired cells in horizontal and vertical direction are passed as an option
# together with the width and height of the environment, this determines the cell size
gui = EnvironmentGUI(parent=root, width=16, height=16, position=[0,0], options={'n_cells': [20,20]})
root.mainloop()  # run the GUI
environment = gui.get_environment()  # get the environment from the GUI

# get start and goal from GUI
clicked = gui.get_clicked_positions()
vehicle.set_initial_conditions(clicked[0])
vehicle.set_terminal_conditions(clicked[1])
start, goal = clicked[0], clicked[1]

# make global planner
# the amount of cells are extracted from the GUI and were either passed by the user to the GUI, or kept at default values
globalplanner = AStarPlanner(environment, gui.n_cells, start, goal)

# Note: When 'corridor' is selected as frame_type and your vehicle size is larger than the cell size,
# shifting frames sometimes causes problems
# for frame_type='shift', e.g. frame_size = 9
options = {'freeT': True, 'horizon_time': 10, 'no_term_con_der': False,
           'n_frames': 2, 'frame_type': 'corridor', 'scale_up_fine': True, 'check_moving_obs_ts': 0.1}
# options = {'n_frames': 2, 'frame_type': 'shift', 'frame_size': 5, 'check_moving_obs_ts': 0.1}
schedulerproblem=SchedulerProblem(vehicle, environment, globalplanner, options=options)

# Note: using linear solver ma57 is optional, normally it reduces the solving time
# multiproblem.set_options({'solver_options':
#     {'ipopt': {'ipopt.linear_solver': 'ma57'}}})

simulator = Simulator(schedulerproblem)
schedulerproblem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
# schedulerproblem.save_movie('scene', format='gif', name='example1_minobs', number_of_frames=300, movie_time=30, axis=False)
# schedulerproblem.save_movie('scene', format='tikz', name='example1_minobs', number_of_frames=100, movie_time=10, axis=False)
