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
vehicle.set_options({'ideal_prediction': False})
vehicle.set_initial_conditions([0., 0.]) # dummy: required for problem.init()
vehicle.set_terminal_conditions([0., 0.]) # dummy: required for problem.init()

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

obstacle = Obstacle({'position': [0., 0.]}, shape=Circle(0.2))
environment.add_obstacle(obstacle)

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()

# create deployer
update_time = 0.1
sample_time = 0.01
deployer = Deployer(problem, sample_time, update_time)

# simulation of a motion planning application: go through 3 via points, while
# an obstacle is changing position
via_points = [[2., -1.5], [2., 2.], [-1.5, 2.]]
obstacle_positions = [[0., 0.], [1.5, 0.], [1., 2.]]

current_time = 0.
state_traj = np.c_[[-1.5, -1.5]]
input_traj = np.c_[[0.0, 0.0]]

n_samp = int(np.round(update_time/sample_time, 6))

for via_point, obstacle_pos in zip(via_points, obstacle_positions):
    vehicle.set_terminal_conditions(via_point)
    target_reached = False
    obstacle.set_state({'position': obstacle_pos})
    vehicle.set_initial_conditions(via_point) # for init guess
    deployer.reset()
    while not target_reached:
        t0 = time.time()
        if (t0-current_time-update_time) >= 0.:
            current_time = t0
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
            if (np.linalg.norm(via_point-state_traj[:, -1]) < 1e-2 and np.linalg.norm(input_traj[:, -1]) < 1e-2):
                target_reached = True

# plot results
n_t = state_traj.shape[1]
time = np.linspace(0., n_t*sample_time+1, n_t)

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
