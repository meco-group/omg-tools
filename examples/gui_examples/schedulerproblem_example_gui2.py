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


# This code shows an expansion of a normal multiframeproblem, in which a large environment is split over
# several frames. In a multiframeproblem we only consider one room/frame at a time.
# Using a schedulerproblem, you can combine frames, i.e. a trajectory is computed through multiple frames at a
# time. The advantage is that e.g. moving obstacles around the corner can immediately be taken into account.

from omgtools import *

# create vehicle
vehicle = Holonomic(shapes = Circle(radius=0.4), options={'syslimit': 'norm_2'},
                    bounds={'vmax': 1.2, 'vmin':-1.2, 'amax':10, 'amin':-10})

# create environment with the help of a GUI

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

# make problem
options = {}
schedulerproblem = SchedulerProblem(vehicle, environment, globalplanner, options=options,
                                    frame_type='min_nobs', n_frames=2)

# simulate the problem
simulator = Simulator(schedulerproblem)

# define what you want to plot
schedulerproblem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
vehicle.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)'])

# run it!
simulator.run()

schedulerproblem.plot_movie('scene', number_of_frames=100, repeat=False)
schedulerproblem.save_movie('scene', format='gif', name='schedulergif', number_of_frames=100, movie_time=10, axis=False)
# schedulerproblem.save_movie('scene', number_of_frames=50, name='scheduler', axis=False)
