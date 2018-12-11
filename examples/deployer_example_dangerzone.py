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
import time
import matplotlib.pyplot as plt
import numpy as np

# this file demonstrates how to use the deployer in a motion planning application with a schedulerproblem

# create vehicle
vehicle = Holonomic(shapes = Circle(0.4), bounds={'vxmin': -1, 'vymin': -1, 'vxmax': 1, 'vymax': 1, 'vmax':1,
                            'axmin': -1, 'aymin': -1, 'axmax': 1, 'aymax': 1}, options={'syslimit':'norm_inf', 'velocity_weight': 100.})
veh_size = vehicle.shapes[0].radius

start = [-9,-9]
goal = [9,9]
vehicle.set_initial_conditions(start)  # dummy: required for problem.init()
vehicle.set_terminal_conditions(goal)  # dummy: required for problem.init()

# create environment
environment = Environment(room={'shape': Square(20), 'position': [0,0], 'draw':True})

rect = Rectangle(width=4, height=4)
environment.add_obstacle(Obstacle({'position': [-6,-6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [-6,-0]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [-6,6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [0,-6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [0,0]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [0,6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [6,-6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [6,0]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [6,6]}, shape=rect))

# add one dangerzone to the environment
dangerzone = DangerZone({'position': [-3., -3]}, shape=Rectangle(width=3, height=4),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.5},
                           simulation={'trajectories': {}})
environment.add_danger_zone(dangerzone)

# each crossroad is a dangerzone, decide at runtime which one is considered by the current vehicle
dangerzones = [DangerZone({'position': [-3., -3]}, shape=Rectangle(width=3, height=4),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.5},
                           simulation={'trajectories': {}}),
               DangerZone({'position': [3., -3]}, shape=Rectangle(width=3, height=4),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.5},
                           simulation={'trajectories': {}}),
               DangerZone({'position': [3., 3]}, shape=Rectangle(width=3, height=4),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.5},
                           simulation={'trajectories': {}})]


# make global planner
# [25,25] = number of cells in vertical and horizonal direction
# select 50,50 or 20,20 cells
globalplanner = AStarPlanner(environment, [50,50], start, goal, options={'veh_size': veh_size})

# make problem
# 'n_frames': number of frames to combine when searching for a trajectory
# 'check_moving_obs_ts': check in steps of ts seconds if a moving obstacle is inside the frame
# 'frame_type': 'corridor': creates corridors
  # 'scale_up_fine': tries to scale up the frame in small steps, leading to the largest possible corridor
  # 'l_shape': cuts off corridors, to obtain L-shapes, and minimize the influence of moving obstacles
# 'frame_type': 'shift': creates frames of fixed size, around the vehicle
  # 'frame_size': size of the shifted frame
options = {'frame_type': 'corridor', 'scale_up_fine': True, 'n_frames': 1, 'l_shape':False}
schedulerproblem = SchedulerProblem(vehicle, environment, globalplanner, options=options)
schedulerproblem.set_options({'solver_options': {'ipopt': {
                                                       # 'ipopt.linear_solver': 'ma57',
                                                       # 'ipopt.hessian_approximation': 'limited-memory'
                                                       }}})

# create deployer
update_time = 0.1
sample_time = 0.01
deployer = Deployer(schedulerproblem, sample_time, update_time)

current_time = 0
current_state = start
state_traj = np.c_[current_state]
input_traj = np.c_[[0.0, 0.0]]

n_samp = int(np.round(update_time/sample_time, 6))

target_reached = False
deployer.reset() # let's start from new initial guess
while not target_reached:
    # 'measure' current state (here ideal trajectory following is simulated)
    if state_traj.shape[1] > 1:
        current_state = state_traj[:, -n_samp-1]
    else:
        current_state = state_traj[:, 0]
    # check which danger zone is most closeby and update position accordingly
    min_distance = np.inf
    for zone in dangerzones:
        dist = euclidean_distance_between_points(current_state, zone.signals['position'][:,-1])
        if dist < min_distance:
            min_distance = dist
            closest_dangerzone = zone
    # update danger zone position to the one of the closest
    environment.danger_zones[0].signals['position'] = closest_dangerzone.signals['position']
    # update motion planning
    trajectories = deployer.update(current_time, current_state)
    # store state & input trajectories -> simulation of ideal trajectory following
    state_traj = np.c_[state_traj, trajectories['state'][:, 1:n_samp+1]]
    input_traj = np.c_[input_traj, trajectories['input'][:, 1:n_samp+1]]
    # check target
    if (np.linalg.norm(goal-state_traj[:, -1]) < 1e-2 and np.linalg.norm(input_traj[:, -1]) < 1e-1):
        target_reached = True
    # update time
    current_time += update_time

# plot results
n_t = state_traj.shape[1]
time = np.linspace(0., n_t*sample_time, n_t)

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(time, state_traj[0, :])
plt.subplot(2, 1, 2)
plt.plot(time, state_traj[1, :])

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(time, input_traj[0, :])
plt.subplot(2, 1, 2)
plt.plot(time, input_traj[1, :])

plt.figure(1)
plt.plot(state_traj[0, :], state_traj[1, :], 'g')
import pdb; pdb.set_trace()  # breakpoint 66ea645c //
