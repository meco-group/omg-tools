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
# the settings inside this example are made specifically for the anchor2D.nc file
GCode = reader.run()

# amount of GCode blocks to combine
n_blocks = 2
# variable_tolerance: allow less freedom in the middle of segments, allow more freedom in the area of connection
# Warning: if you receive a message 'infeasible problem' try increasing the amount of knot_intervals,
# this gives more freedom
# it is also possible that segments with variable tolerance are too short, check with required stop distance of tool
variable_tolerance = False
# required tolerance of the machined part [mm]
tol = 0.05
# reduced tolerance for middle of segments
tol_small = 0.1*tol
# how to split segments if using variable tolerance e.g. 0.1 --> 0.1, 0.8, 0.1 * total length
split_small = 0.1
# minimal length of segment that you want to split
split_length = 6.
# split_circle: split large circle segments in smaller ones to avoid shortcuts in the trajectory
split_circle = True
bounds = {'vmin':-100, 'vmax':100,
          'amin':-500, 'amax':500,
          'jmin':-1500e3, 'jmax':1500e3}  # [mm]
# vel_limit: if the limiting factor is the machining process, put 'machining'
# if the limiting factor is the velocity of the axes themselves, put: 'axes'
tool = Tool(tol, bounds=bounds, options={'vel_limit':'machining','variable_tolerance':variable_tolerance}, tol_small=tol_small)  # tool to follow the GCode
tool.define_knots(knot_intervals=10)
tool.set_initial_conditions(GCode[0].start)  # start position of first GCode block
tool.set_terminal_conditions(GCode[-1].end)  # goal position of last GCode block

# solve with a GCodeSchedulerProblem: this problem will combine n_blocks of GCode
# and compute trajectories for the tool
# each block will be converted to a room, that is put inside the total environment
# there are two room shapes: Rectangle and Ring (circle segment with inner and outer diameter)
# if you want to compute trajectories by using the deployer, put with_deployer=True
schedulerproblem = GCodeSchedulerProblem(tool, GCode, n_segments=n_blocks, split_circle=split_circle,
                                         variable_tolerance=variable_tolerance, split_length=split_length, split_small=split_small)

schedulerproblem.set_options({'solver_options': {'ipopt': {'ipopt.tol': 1e-5,
                                                           'ipopt.linear_solver': 'ma57',
                                                           'ipopt.warm_start_bound_push': 1e-6,
                                                           'ipopt.warm_start_mult_bound_push': 1e-6,
                                                           'ipopt.warm_start_mult_bound_push': 1e-6,
                                                           'ipopt.mu_init': 1e-5,
                                                           # 'ipopt.hessian_approximation': 'limited-memory',
                                                           'ipopt.max_iter': 20000}}})#,

# put problem in deployer, computes the trajectories for the tool
deployer = Deployer(schedulerproblem, sample_time=0.001)

# define what you want to plot
schedulerproblem.plot('scene')
# tool.plot('state', knots=True, prediction=True, labels=['x (m)', 'y (m)', 'z (m)'])
# tool.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)'])
# tool.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'])
# tool.plot('ddinput', knots=True, prediction=True, labels=['j_x (m/s^3)', 'j_y (m/s^3)', 'j_z (m/s^3)'])

# run using a receding horizon of one segment
deployer.update_segment()

# plotting afterwards, and saving is only available when using the simulator
# schedulerproblem.plot_movie('scene', number_of_frames=100, repeat=False)
# schedulerproblem.save_movie('scene', format='gif', name='gcodegif', number_of_frames=100, movie_time=10, axis=False)
# schedulerproblem.save_movie('scene', number_of_frames=50, name='gcodescheduler', axis=False)
plt.show(block=True)
