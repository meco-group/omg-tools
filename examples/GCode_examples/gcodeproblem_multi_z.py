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
# to machine a part, described by GCode (in a .nc file), with a certain tolerance.
# It can handle GCode files that contain a workpiece, machined in the XY-plane, but also require a
# movement in the z-axis from time to time. E.g. when maching a workpiece that consists of a layer
# in the XY-plane, followed by a downward movement of the z-axis, a new layer in the XY-plane etc.
# Example file: rsq5_multi.nc

# Or a GCode file describing the machining of several circles. After machining each circle,
# the z-axis is retracted, the tool moves to a new position, the z-axis is again moved to the old position,
# and the new circle can be machined. Engaging the material needs to be done with caution, when retracting
# or moving without machining, the velocity limits can be increased. When machining, the process determines
# the maximum velocity.
# Example file: Star_shift_scale.nc

from omgtools import *
import os

# make GCode reader and run it to obtain an object-oriented description of the GCode,
# by opening a file dialog in which you can select your GCode as an .nc-file
reader = GCodeReader()
# this opens a file dialog in which you can select your GCode as an .nc-file
# the settings inside this example are made specifically for the anchor2D.nc file
GCode = reader.run()

n_blocks = 3  # amount of GCode blocks to combine
tol = 0.01  # required tolerance of the machined part [mm]
# normal bounds
bounds = {'vmin':-16.6, 'vmax':16.6,  # [mm/s]
          'amin':-20e3, 'amax':20e3,  # [mm/s**2]
          'jmin':-1500e3, 'jmax':1500e3}  # [mm/s**3]
# bounds when engaging the workpiece
bounds_engage = {'vmin':-1.66, 'vmax':1.66,  # [mm/s]
          'amin':-20e3, 'amax':20e3,  # [mm/s**2]
          'jmin':-1500e3, 'jmax':1500e3}  # [mm/s**3]
# bounds when not machining
bounds_free = {'vmin':-150, 'vmax':150,  # [mm/s]
          'amin':-20e3, 'amax':20e3,  # [mm/s**2]
          'jmin':-1500e3, 'jmax':1500e3}  # [mm/s**3]
# is the tool inside the material or not?
tool_free = False


# loop over all blocks: sequence of z-engage/retraction blocks and normal xy-plane blocks
for idx, GCode_block in enumerate(GCode_blocks):
    if (len(GCode_block) == 1 and GCode_block[0].Z0 != GCode_block[0].Z1):
        if GCode_block[0].Z0 < GCode_block[0].Z1:
            # solve optimization problem to engage
            tool = Tool(tol, bounds=bounds_engage)
            # at end of movement, tool is not free
            tool_free = False
        else:
            # solve optimization problem to retract
            tool = Tool(tol, bounds=bounds_free)
            # at end of movement, tool is free
            tool_free = True

        # switch x and z position
        x0 = GCode_block[0].X0
        x1 = GCode_block[0].X1
        z0 = GCode_block[0].Z0
        z1 = GCode_block[0].Z1
        GCode_block[0].X0 = z0
        GCode_block[0].X1 = z1
        GCode_block[0].Z0 = x0
        GCode_block[0].Z1 = x1

        # assign new start and end
        GCode_block[0].start = [GCode_block[0].X0, GCode_block[0].Y0, GCode_block[0].Z0]
        GCode_block[0].end = [GCode_block[0].X1, GCode_block[0].Y1, GCode_block[0].Z1]

        tool.define_knots(knot_intervals=10)
        tool.set_initial_conditions(GCode_block[0].start)  # start position of first GCode block
        tool.set_terminal_conditions(GCode_block[0].end)  # goal position of last GCode block

        schedulerproblem = GCodeSchedulerProblem(tool, GCode_block, n_segments=1)
        schedulerproblem.set_options({'solver_options': {'ipopt': {'ipopt.tol': 1e-5,
                                                               'ipopt.linear_solver': 'ma57',
                                                               'ipopt.warm_start_bound_push': 1e-6,
                                                               'ipopt.warm_start_mult_bound_push': 1e-6,
                                                               'ipopt.warm_start_mult_bound_push': 1e-6,
                                                               'ipopt.mu_init': 1e-5,
                                                               'ipopt.hessian_approximation': 'limited-memory',
                                                               'ipopt.max_iter': 20000}}})#,
                                                                   'ipopt.max_iter': 20000}}})#,
        # put problem in deployer: choose this if you just want to obtain the trajectories for the tool
        deployer = Deployer(schedulerproblem, sample_time=0.0001)

        # run using a receding horizon of one segment
        deployer.update_segment()

        # switch x and z back with reorder parameter
        deployer.save_results(count=idx)
    else:
        if tool_free:
            # tool is not in material, so axis limits are selected
            tool = Tool(tol, bounds = bounds_free)
        else:
            # tool is in material, so process limits velocity
            tool = Tool(tol, bounds = bounds)

        tool.define_knots(knot_intervals=10)
        tool.set_initial_conditions(GCode_block[0].start)  # start position of first GCode block
        tool.set_terminal_conditions(GCode_block[-1].end)  # goal position of last GCode block

        if len(GCode_block) < n_blocks:
            schedulerproblem = GCodeSchedulerProblem(tool, GCode_block, n_segments=len(GCode_block), split_circle = True)
        else:
            schedulerproblem = GCodeSchedulerProblem(tool, GCode_block, n_segments=n_blocks, split_circle = True)
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

        # define what you want to plot
        schedulerproblem.plot('scene')
        tool.plot('state', knots=True, prediction=True, labels=['x (m)', 'y (m)', 'z (m)'])
        tool.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)'])
        tool.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'])
        tool.plot('ddinput', knots=True, prediction=True, labels=['j_x (m/s^3)', 'j_y (m/s^3)', 'j_z (m/s^3)'])

        # run using a receding horizon of one segment
        deployer.update_segment()

        # save results in csv file
        deployer.save_results(count=idx)  # sample_time=0.0001