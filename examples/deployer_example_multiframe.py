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

# this file demonstrates how to use the deployer in a motion planning application

# create vehicle
vehicle = Holonomic()
vehicle.set_options({'safety_distance': 0.1})

start = [0., 0.]
goal = [6., 3.5]

vehicle.set_initial_conditions(start) # dummy: required for problem.init()
vehicle.set_terminal_conditions(goal) # dummy: required for problem.init()
veh_size = vehicle.shapes[0].radius

# create static environment
environment = Environment(room={'shape': Rectangle(width=7., height=4.5), 'position': [3., 1.75]})
rectangle = Rectangle(width=1., height=1.)

environment.add_obstacle(Obstacle({'position': [1., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [5., 1.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1., 2.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [3., 2.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [5., 2.5]}, shape=rectangle))

# make global planner to gerenate waypoints through static environment
globalplanner = AStarPlanner(environment, [25, 25], start, goal, options={'veh_size': veh_size})
waypoints = globalplanner.get_path()

# make schedular problem to generate trajectories localy through n frames
options = {'freeT': True, 'frame_type': 'corridor', 'scale_up_fine': True, 'n_frames': 1}
schedulerproblem = SchedulerProblem(vehicle, environment, None, options=options)

schedulerproblem.set_global_path(waypoints)

# dynamic obstacles
obstacle = Obstacle({'position': [2., 1.]}, shape=Circle(0.1))
schedulerproblem.set_moving_obstacles([obstacle])

# create deployer
update_time = 0.1
sample_time = 0.01
deployer = Deployer(schedulerproblem, sample_time, update_time)

# simulate behavior using deployer
current_time = 0
current_state = start
state_traj = np.c_[current_state]
input_traj = np.c_[[0.0, 0.0]]

n_samp = int(np.round(update_time/sample_time, 6))

vehicle.set_terminal_conditions(goal)
target_reached = False
# obstacle can dynamically be updated
vehicle.set_initial_conditions(current_state) # for init guess
deployer.reset() # let's start from new initial guess
while not target_reached:
    # 'measure' current state (here ideal trajectory following is simulated)
    if state_traj.shape[1] > 1:
        current_state = state_traj[:, -n_samp-1]
    else:
        current_state = state_traj[:, 0]
    # update motion planning
    trajectories = deployer.update(current_time, current_state)
    # store state & input trajectories -> simulation of ideal trajectory following
    state_traj = np.c_[state_traj, trajectories['state'][:, 1:n_samp+1]]
    input_traj = np.c_[input_traj, trajectories['input'][:, 1:n_samp+1]]
    # check target
    if (np.linalg.norm(goal-state_traj[:, -1]) < 1e-2 and np.linalg.norm(input_traj[:, -1]) < 1e-2):
        target_reached = True
    if (schedulerproblem.iteration > 300):
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

plt.figure()
plt.plot(state_traj[0, :], state_traj[1, :])
