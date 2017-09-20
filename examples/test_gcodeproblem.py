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

# make GCode reader and run it to obtain an object-oriented description of the GCode
reader = GCodeReader()
# this opens a file dialog in which you can select your GCode as an .nc-file
GCode = reader.run()

n_blocks = 3  # amount of GCode blocks to combine
tol = 5e-1  # required tolerance of the machined part [mm]
bounds = {'vmin':-1e3, 'vmax':1e3,
          'amin':-20e3, 'amax':20e3,
          'jmin':-850e3, 'jmax':850e3}  # [mm]
tool = Tool(tol, bounds = bounds)  # tool to follow the GCode
tool.define_knots(knot_intervals=10)
tool.set_initial_conditions(GCode[0].start)  # start position of first GCode block
tool.set_terminal_conditions(GCode[-1].end)  # goal position of last GCode block

# solve with a GCodeSchedulerProblem: this problem will combine n_blocks of GCode
# and compute trajectories for the tool
# each block will be converted to a room, that is put inside the total environment
# there are two room shapes: Rectangle and Ring (circle segment with inner and outer diameter)
# if you want to compute trajectories by using the deployer, put with_deployer=True
schedulerproblem = GCodeSchedulerProblem(tool, GCode, n_segments=n_blocks, with_deployer=True)
schedulerproblem.set_options({'solver_options': {'ipopt': {'ipopt.tol': 1e-8,
														   'ipopt.linear_solver': 'ma57'}}})#,
                                                           # 'ipopt.hessian_approximation': 'limited-memory'}}})
# put problem in deployer: choose this if you just want to obtain the trajectories for the tool
deployer = Deployer(schedulerproblem, sample_time=0.001)
# put problem in simulator: choose this if you want to simulate step by step, and investigate after each segment
simulator = Simulator(schedulerproblem, sample_time=0.001)

# define what you want to plot
schedulerproblem.plot('scene')
tool.plot('state', knots=True, prediction=True, labels=['x (m)', 'y (m)', 'z (m)'])
tool.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)'])
tool.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'])
tool.plot('ddinput', knots=True, prediction=True, labels=['j_x (m/s^3)', 'j_y (m/s^3)', 'j_z (m/s^3)'])

# run using a receding horizon of one segment
deployer.run_segment()
# simulator.run_segment()

# plotting and saving afterwards is only available when using the simulator
# schedulerproblem.plot_movie('scene', number_of_frames=100, repeat=False)
# schedulerproblem.save_movie('scene', format='gif', name='gcodegif', number_of_frames=100, movie_time=10, axis=False)
# schedulerproblem.save_movie('scene', number_of_frames=50, name='gcodescheduler', axis=False)