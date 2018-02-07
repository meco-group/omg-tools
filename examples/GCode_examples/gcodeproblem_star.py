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


# This example computes a trajectory that a tool of e.g. a milling machine has to follow
# to machine a part, described by GCode in a .nc file, with a certain tolerance.

from omgtools import *

# make GCode reader and run it to obtain an object-oriented description of the GCode
reader = GCodeReader()
# this opens a file dialog in which you can select your GCode as an .nc-file
# the settings in this file are made specifically for the rsq5.nc file
GCode = reader.run()

n_blocks = 3  # amount of GCode blocks to combine
variable_tolerance = False
split_circle = True
tol = 0.01  # required tolerance of the machined part [mm]
bounds = {'vmin':-16.6, 'vmax':16.6,
          'amin':-20e3, 'amax':20e3,
          'jmin':-1500e3, 'jmax':1500e3}  # [mm]
tool = Tool(tol, bounds=bounds, options={'vel_limit':'axes'})  # tool to follow the GCode
tool.define_knots(knot_intervals=20)
tool.set_initial_conditions(GCode[0].start)  # start position of first GCode block
tool.set_terminal_conditions(GCode[-1].end)  # goal position of last GCode block

# solve with a GCodeSchedulerProblem: this problem will combine n_blocks of GCode
# and compute trajectories for the tool
# each block will be converted to a room, that is put inside the total environment
# there are two room shapes: Rectangle and Ring (circle segment with inner and outer diameter)
# if you want to compute trajectories by using the deployer, put with_deployer=True
schedulerproblem = GCodeSchedulerProblem(tool, GCode, n_segments=n_blocks, split_circle=split_circle,
                                         variable_tolerance=variable_tolerance,)

schedulerproblem.set_options({'solver_options': {'ipopt': {'ipopt.tol': 1e-5,
                                                           'ipopt.linear_solver': 'ma57',
                                                           'ipopt.warm_start_bound_push': 1e-6,
                                                           'ipopt.warm_start_mult_bound_push': 1e-6,
                                                           'ipopt.warm_start_mult_bound_push': 1e-6,
                                                           'ipopt.mu_init': 1e-5,
                                                           # 'ipopt.hessian_approximation': 'limited-memory',
                                                           'ipopt.max_iter': 20000}}})#,
# put problem in deployer: choose this if you just want to obtain the trajectories for the tool
deployer = Deployer(schedulerproblem, sample_time=0.001)

# define what you want to plot
schedulerproblem.plot('scene')
tool.plot('state', knots=True, prediction=True, labels=['x (m)', 'y (m)', 'z (m)'])
tool.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)'])
tool.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'])
tool.plot('ddinput', knots=True, prediction=True, labels=['j_x (m/s^3)', 'j_y (m/s^3)', 'j_z (m/s^3)'])

# run using a receding horizon of one segment
deployer.update_segment()

# plot measured positions on reference workpiece
import numpy as np
pos_data = np.loadtxt(open("realposition_star.csv", "rb"), delimiter=",")

plt.figure(6)
plt.plot(pos_data[:,0], pos_data[:,1], 'g')

import pdb; pdb.set_trace()  # breakpoint c78baa4a //
# plotting and saving afterwards is only available when using the simulator
# schedulerproblem.plot_movie('scene', number_of_frames=100, repeat=False)
# schedulerproblem.save_movie('scene', format='gif', name='gcodegif', number_of_frames=100, movie_time=10, axis=False)
# schedulerproblem.save_movie('scene', number_of_frames=50, name='gcodescheduler', axis=False)