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

# make GCode reader and run it to obtain object-oriented description of the GCode
reader = GCodeReader()
GCode = reader.run()

n_blocks = 3  # amount of GCode blocks to combine
tol = 5e-1  # required tolerance of the machined part
bounds = {'vmin':-5, 'vmax':5,
          'amin':-50, 'amax':50,
          'jmin':-500, 'jmax': 500}
tool = Tool(tol, bounds = bounds)
tool.set_initial_conditions(GCode[0].start)  # start position of first GCode block
tool.set_terminal_conditions(GCode[-1].end)  # goal position of last GCode block

# solve with a GCodeSchedulerProblem: this problem will combine n_blocks of GCode and compute trajectories for the tool
# each block will be converted to a room, that is put inside the total environment
# there are two room shapes: rectangle and ring (circle segment with inner and outer diameter)
schedulerproblem = GCodeSchedulerProblem(tool, GCode, n_segments=n_blocks)

# put problem in simulator
simulator = Simulator(schedulerproblem, sample_time=0.001)

# define what you want to plot
schedulerproblem.plot('scene')
tool.plot('state', knots=True, prediction=True, labels=['x (m)', 'y (m)', 'z (m)'])
tool.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)'])
tool.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'])

# simulate using a receding horizon of one segment
simulator.run_segment()

schedulerproblem.plot_movie('scene', number_of_frames=100, repeat=False)
schedulerproblem.save_movie('scene', format='gif', name='gcodegif', number_of_frames=100, movie_time=10, axis=False)
# schedulerproblem.save_movie('scene', number_of_frames=50, name='gcodescheduler', axis=False)