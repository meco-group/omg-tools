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

n_blocks = 3  # amount of GCode blocks to combine
tol = 0.01  # required tolerance of the machined part [mm]
bounds = {'vmin':-16.6, 'vmax':16.6,  # [mm/s]
          'amin':-20e3, 'amax':20e3,  # [mm/s**2]
          'jmin':-1500e3, 'jmax':1500e3}  # [mm/s**3]
# tool = Tool(tol, bounds = bounds)  # tool to follow the GCode
# tool.define_knots(knot_intervals=10)
# tool.set_initial_conditions(GCode[0].start)  # start position of first GCode block
# tool.set_terminal_conditions(GCode[-1].end)  # goal position of last GCode block


# split in parts that don't contain a G01 z-retraction command,
# followed by a block that do contain a retraction command
GCode_blocks = []
blocks = []
for block in GCode:
    if block.type not in ['G00', 'G01'] or (block.Z0 == block.Z1):
        # arc segment, or no movement in z-direction
        blocks.append(block)
    else:
        # this is a z-retract/engage block
        # save the previous blocks
        if blocks:
            GCode_blocks.append(blocks)
        # clear blocks variable
        blocks = []
        # add z-block separately
        GCode_blocks.append([block])
GCode_blocks.append(blocks)
# otherwise doesn't work if there are no Z-trajectories

# loop over all blocks: sequence of Z-engage/retraction blocks and normal XY blocks
for idx, GCode_block in enumerate(GCode_blocks):
    if (len(GCode_block) == 1 and GCode_block[0].Z0 != GCode_block[0].Z1):
        if GCode_block[0].Z0 < GCode_block[0].Z1:
            file = open('z_engage_tol0_01_f100_j1500_reorder_.csv', 'r')
        else:
            file = open('z_retract_tol0_01_f1000_j1500_reorder_.csv', 'r')
        data = file.read().splitlines()
        file.close()
        new_data = []
        for d in data:
            d = d.split(',')
            new_data.append([GCode_block[0].X0, float(d[1]), float(d[2]), GCode_block[0].Y0, float(d[4]), float(d[5]), float(d[6]), float(d[7]), float(d[8])])
        if idx == 0:
            park = [[0, 0, 0, 0, 0, 0, -2, 0, 0]]  # where to park the tool before machining
            new_data = park + new_data  # place parking spot before data
        np.savetxt('trajmat'+str(idx)+'.csv', new_data , delimiter=',')

    else:
        tool = Tool(tol, bounds = bounds)  # tool to follow the GCode
        tool.define_knots(knot_intervals=10)
        tool.set_initial_conditions(GCode_block[0].start)  # start position of first GCode block
        tool.set_terminal_conditions(GCode_block[-1].end)  # goal position of last GCode block

        # solve with a GCodeSchedulerProblem: this problem will combine n_blocks of GCode
        # and compute trajectories for the tool
        # each block will be converted to a room, that is put inside the total environment
        # there are two room shapes: Rectangle and Ring (circle segment with inner and outer diameter)
        # if you want to compute trajectories by using the deployer, put with_deployer=True
        # if the trajectory takes the wrong side of a ring segment with large arc angle,
        # you can split ring segments with an arc_angle >3*pi/4 by putting splitting=True
        if len(GCode_block) < n_blocks:
            schedulerproblem = GCodeSchedulerProblem(tool, GCode_block, n_segments=len(GCode_block), with_deployer=True, split_circle = True, variable_tolerance=False)
        else:
            schedulerproblem = GCodeSchedulerProblem(tool, GCode_block, n_segments=n_blocks, with_deployer=True, split_circle = True, variable_tolerance=False)
        schedulerproblem.set_options({'solver_options': {'ipopt': {'ipopt.tol': 1e-5,
                                                                   'ipopt.linear_solver': 'ma57',
                                                                   'ipopt.warm_start_bound_push': 1e-6,
                                                                   'ipopt.warm_start_mult_bound_push': 1e-6,
                                                                   'ipopt.warm_start_mult_bound_push': 1e-6,
                                                                   'ipopt.mu_init': 1e-5,
                                                                   'ipopt.hessian_approximation': 'limited-memory',
                                                                   'ipopt.max_iter': 20000}}})#,
        # put problem in deployer: choose this if you just want to obtain the trajectories for the tool
        deployer = Deployer(schedulerproblem, sample_time=0.0001)
        # put problem in simulator: choose this if you want to simulate step by step
        # simulator = Simulator(schedulerproblem, sample_time=0.0001)

        # define what you want to plot
        schedulerproblem.plot('scene')
        tool.plot('state', knots=True, prediction=True, labels=['x (m)', 'y (m)', 'z (m)'])
        tool.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)'])
        tool.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'])
        tool.plot('ddinput', knots=True, prediction=True, labels=['j_x (m/s^3)', 'j_y (m/s^3)', 'j_z (m/s^3)'])

        # run using a receding horizon of one segment
        deployer.update_segment()
        # simulator.run_segment()

        # save results in csv file
        if idx == 0:
            first = True
        else:
            first = False
        deployer.save_results(count=idx, first=first)  # sample_time = 0.0001

        # plotting afterwards, and saving is only available when using the simulator
        # schedulerproblem.plot_movie('scene', number_of_frames=100, repeat=False)
        # schedulerproblem.save_movie('scene', format='gif', name='gcodegif', number_of_frames=100, movie_time=10, axis=False)
        # schedulerproblem.save_movie('scene', number_of_frames=50, name='gcodescheduler', axis=False)