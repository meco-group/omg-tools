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

# With GUI-determined environment
import sys, os
sys.path.insert(0,os.getcwd()+'/..')
from omgtools import *

# create vehicle
vehicle = Holonomic(shapes = Circle(radius=0.1), options={'syslimit': 'norm_2'}, bounds={'vmax': 1, 'vmin':-1, 'amax':10, 'amin':-10})

# create environment
# stationary obstacles via GUI
import Tkinter as tk
root = tk.Tk()
options={'cell_size': 0.25}
gui = EnvironmentGUI(parent=root, width=10, height=10, position=[0,0], options=options)
root.mainloop()
environment = gui.get_environment()

# manually add moving obstacles to environment

# change start and goal
clicked = gui.get_clicked_positions()
vehicle.set_initial_conditions(clicked[0])
vehicle.set_terminal_conditions(clicked[1])
start, goal = clicked[0], clicked[1]

# make global planner
globalplanner = AStarPlanner(environment, options['cell_size'], start, goal)

# make coordinator
options={'freeT': True, 'horizon_time': 10, 'no_term_con_der': False}
multiproblem=MultiFrameProblem(vehicle, environment, globalplanner, options=options, frame_type='min_nobs')
multiproblem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
multiproblem.init()

simulator = Simulator(multiproblem)
multiproblem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
multiproblem.save_movie('scene', format='gif', name='multiproblemgif', number_of_frames=100, movie_time=5, axis=False)
# multiproblem.save_movie('scene', format='tikz', name='multiproblemtikz', number_of_frames=100, movie_time=5, axis=False)