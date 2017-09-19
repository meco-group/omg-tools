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

import numpy as np
from matplotlib import pyplot as plt


class Deployer:

    def __init__(self, problem, sample_time=0.01, update_time=0.1):
        self.set_problem(problem)
        self.update_time = update_time
        self.sample_time = sample_time
        self.current_time = 0.
        self.iteration0 = True

    def set_problem(self, problem):
        self.problem = problem

    def reset(self):
        self.iteration0 = True
        self.problem.reinitialize()

    def update(self, current_time, states=None, inputs=None, dinputs=None, update_time=None, enforce_states=False, enforce_inputs=False):
        current_time = float(current_time)
        if not update_time:
            update_time = self.update_time
        ### Adapted
        if hasattr(self.problem.vehicles[0], 'signals'):
            if round(update_time - float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time),4) >= self.sample_time:
                update_time = float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time)
        # is there enough time left to update with the normal update time?
        # if not, lower update time
        elif hasattr(self.problem.vehicles[0], 'trajectories'):
            if round(update_time - float(self.problem.vehicles[0].trajectories['time'][:, -1] - self.current_time),4) >= self.sample_time:
                update_time = float(self.problem.vehicles[0].trajectories['time'][:, -1] - self.current_time)
        if (self.iteration0):
            self.iteration0 = False
            self.problem.initialize(current_time)
            delay = 0
        else:
            delay = int((current_time - self.current_time - update_time)/self.sample_time)

        # if the update time + delay is more than the time which is left, leave out the delay
        if hasattr(self.problem.vehicles[0], 'trajectories'):
            if (delay + int(np.round(update_time/self.sample_time, 6))) > int(np.round(float(self.problem.vehicles[0].trajectories['time'][:, -1] - self.current_time)/self.sample_time,6)):
                delay = 0

        self.problem.predict(current_time, update_time, self.sample_time, states, inputs, dinputs, delay, enforce_states, enforce_inputs)
        self.problem.solve(current_time, update_time)
        self.problem.store(current_time, update_time, self.sample_time)
        self.current_time = current_time
        # return trajectories
        trajectories = {}
        if len(self.problem.vehicles) == 1:
            return self.problem.vehicles[0].trajectories
        else:
            for vehicle in self.problem.vehicles:
                trajectories[str(vehicle)] = vehicle.trajectories
        return trajectories

    def run_segment(self):
        self.reset()

        # create figures
        plt.figure(2)  # state
        plt.figure(3)  # input
        plt.figure(4)  # dinput
        plt.figure(5)  # ddinput
        plt.figure(6)  # trajectory

        # initialize trajectories
        state_traj = np.c_[self.problem.curr_state]
        input_traj = np.c_[[0.,0.,0.]]
        dinput_traj = np.c_[[0.,0.,0.]]
        ddinput_traj = np.c_[[0.,0.,0.]]

        current_time = 0.
        target_reached = False

        while not target_reached:
            if not hasattr(self.problem.vehicles[0], 'result_splines'):
                # problem not yet solved before
                update_time = 0.
                states = self.problem.curr_state
                inputs = None
                dinputs = None
                enforce_inputs = False  # since there are no inputs yet
                current_time = 0.
                trajectories = self.update(current_time, states=states, inputs=inputs, dinputs=dinputs, update_time=update_time, enforce_states=True, enforce_inputs=enforce_inputs)
            else:
                update_time = 0.  # not used, and if used it would be wrong
                current_time += self.problem.motion_times[0]
                enforce_inputs = True
                trajectories = self.update(current_time, states=states, inputs=inputs, dinputs=dinputs, update_time=update_time, enforce_states=True, enforce_inputs=enforce_inputs)
                self.current_time = current_time + self.problem.motion_times[0]

            # computed motion time for first segment
            update_time = self.problem.motion_times[0]
            # required amount of samples to take from trajectories to reach connection point between segments
            n_samp = int(np.round(update_time/self.sample_time, 6))

            pos_splines = self.problem.vehicles[0].result_splines
            input_splines = [s.derivative(1) for s in pos_splines]
            dinput_splines = [s.derivative(2) for s in pos_splines]
            ddinput_splines = [s.derivative(3) for s in pos_splines]
            states = np.hstack([s(update_time) for s in pos_splines])  # state at end of first segment
            inputs = np.hstack([s(update_time) for s in input_splines])  # input at end of first segment
            dinputs = np.hstack([s(update_time) for s in dinput_splines])
            ddinputs = np.hstack([s(update_time) for s in ddinput_splines])

            # save and plot results
            if trajectories is not None:
                # state trajectory, append current state because this is not reached at a multiple of sample_time
                state_traj = np.c_[state_traj, trajectories['state'][:, 1:n_samp+1], states]
                # input trajectory, append current input
                input_traj = np.c_[input_traj, trajectories['input'][:, 1:n_samp+1], inputs]
                dinput_traj = np.c_[dinput_traj, trajectories['dinput'][:, 1:n_samp+1], dinputs]
                ddinput_traj = np.c_[ddinput_traj, trajectories['ddinput'][:, 1:n_samp+1], ddinputs]
                n_t = state_traj.shape[1]  # amount of points in trajectory
                time = np.linspace(0, current_time+update_time, n_t)  # make time vector

                plt.figure(2)
                plt.subplot(2, 1, 1)
                plt.cla()
                plt.plot(time, state_traj[0, :])
                plt.subplot(2, 1, 2)
                plt.cla()
                plt.plot(time, state_traj[1, :])
                plt.pause(0.1)

                plt.figure(3)
                plt.subplot(2, 1, 1)
                plt.cla()
                plt.plot(time, input_traj[0, :])
                plt.subplot(2, 1, 2)
                plt.cla()
                plt.plot(time, input_traj[1, :])
                plt.pause(0.1)

                plt.figure(4)
                plt.subplot(2, 1, 1)
                plt.cla()
                plt.plot(time, dinput_traj[0, :])
                plt.subplot(2, 1, 2)
                plt.cla()
                plt.plot(time, dinput_traj[1, :])
                plt.pause(0.1)

                plt.figure(5)
                plt.subplot(2, 1, 1)
                plt.cla()
                plt.plot(time, ddinput_traj[0, :])
                plt.subplot(2, 1, 2)
                plt.cla()
                plt.plot(time, ddinput_traj[1, :])
                plt.pause(0.1)

                plt.figure(6)
                plt.cla()
                # plot trajectory
                plt.plot(state_traj[0, :], state_traj[1, :])
                # plot environment
                for room in self.problem.environment.rooms:
                    points = room['shape'].draw(room['pose'][:3])[0][0]
                    # add first point again to close shape
                    points = np.c_[points, [points[0,0], points[1,0]]]
                    plt.plot(points[0,:], points[1,:], color='red', linestyle = '--', linewidth= 1.2)
                plt.pause(0.1)

                # check if target is reached
                if (np.linalg.norm(self.problem.goal_state-state_traj[:, -1]) < 1e-2 and np.linalg.norm(input_traj[:, -1]) < 1e-2):
                    target_reached = True

        # target reached, print final information
        self.problem.final()