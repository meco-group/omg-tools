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

    def update_segment(self):
        self.reset()

        # boolean to select if we want to go to next segment
        # e.g. is True if current solution was infeasible
        self.problem.no_update = False

        # create figures
        plt.figure(2)  # state
        plt.figure(3)  # input
        plt.figure(4)  # dinput
        plt.figure(5)  # ddinput
        plt.figure(6)  # scene

        # initialize trajectories
        self.state_traj = np.c_[self.problem.curr_state]
        self.input_traj = np.c_[[0.,0.,0.]]
        self.dinput_traj = np.c_[[0.,0.,0.]]
        self.ddinput_traj = np.c_[[0.,0.,0.]]

        current_time = 0.
        target_reached = False

        while not target_reached:
            if not hasattr(self.problem.vehicles[0], 'result_splines'):
                # problem not yet solved before
                update_time = 0.
                current_time = 0.
                states = self.problem.curr_state
                inputs = None
                dinputs = None
                ddinputs = None
                enforce_inputs = False  # since there are no inputs yet
                trajectories = self.update(current_time, states=states, inputs=inputs, dinputs=dinputs,
                                           update_time=update_time, enforce_states=True, enforce_inputs=enforce_inputs)
            else:
                # problem was solved before
                update_time = 0.  # this value is passed on to update, but is not used
                current_time += self.problem.motion_times[0]  # step in time
                enforce_inputs = True

                # set new values at end values of previous iteration
                states = np.array(self.states_end)
                inputs = np.array(self.inputs_end)
                dinputs = np.array(self.dinputs_end)
                ddinputs = np.array(self.ddinputs_end)

                trajectories = self.update(current_time, states=states, inputs=inputs, dinputs=dinputs,
                                           update_time=update_time, enforce_states=True, enforce_inputs=enforce_inputs)
                self.current_time = current_time + self.problem.motion_times[0]

            # computed motion time for first segment
            update_time = self.problem.motion_times[0]
            # required amount of samples to take from trajectories to reach connection point between segments
            n_samp = int(np.round(update_time/self.sample_time, 6))

            # save and plot results
            if trajectories is not None:

                # update values
                # use original splines here, not the ones from concat_splines, since they are less accurate
                pos_splines = self.problem.vehicles[0].result_spline_segments[0]
                input_splines = [s.derivative(1) for s in pos_splines]
                dinput_splines = [s.derivative(2) for s in pos_splines]
                ddinput_splines = [s.derivative(3) for s in pos_splines]
                # compute values at end of first segment = start of the next iteration
                self.states_end = np.hstack([s(1.) for s in pos_splines])
                self.inputs_end = np.hstack([s(1.) for s in input_splines])*1./self.problem.motion_times[0]
                self.dinputs_end = np.hstack([s(1.) for s in dinput_splines])*1./self.problem.motion_times[0]**2
                self.ddinputs_end = np.hstack([s(1.) for s in ddinput_splines])*1./self.problem.motion_times[0]**3

                # save old values
                # these are the ones that end at the starting state of current iteration
                state_traj_old = np.array(self.state_traj)
                input_traj_old = np.array(self.input_traj)
                dinput_traj_old = np.array(self.dinput_traj)
                ddinput_traj_old = np.array(self.ddinput_traj)

                # state trajectory, append current state because this is not necessarily
                # reached at a multiple of sample_time
                self.state_traj = np.c_[self.state_traj, trajectories['state'][:, 1:n_samp+1], self.states_end]
                # input trajectory, append current input
                self.input_traj = np.c_[self.input_traj, trajectories['input'][:, 1:n_samp+1], self.inputs_end]
                self.dinput_traj = np.c_[self.dinput_traj, trajectories['dinput'][:, 1:n_samp+1], self.dinputs_end]
                self.ddinput_traj = np.c_[self.ddinput_traj, trajectories['ddinput'][:, 1:n_samp+1], self.ddinputs_end]

                # plot trajectories of state, input,...
                self.plot_results(current_time, update_time)

                # check if problem was solved successfully
                self.check_results(states, inputs, dinputs, ddinputs, current_time,
                                   state_traj_old, input_traj_old, dinput_traj_old, ddinput_traj_old)

                # check if target is reached
                if (np.linalg.norm(self.problem.goal_state-self.state_traj[:, -1]) < 1e-2 and np.linalg.norm(self.input_traj[:, -1]) < 1e-2):
                    target_reached = True

        # target reached, print final information
        self.problem.final()

    def check_results(self, states, inputs, dinputs, ddinputs, current_time, state_traj_old, input_traj_old, dinput_traj_old, ddinput_traj_old):
        # Try to improve solution for last segment:
        # If the latest segment was not what you liked, there are two options:
        # 1) let the user decide about each segment if it is good or not
        # user_input = ''
        # while (not user_input in ['yes', 'no']):
        #     user_input = raw_input("Are you happy with the latest computed segment (yes/no): ")
        # if user_input == 'no':
        # 2) automatically re-solve a slightly adapted version of the problem when no optimal solution was found
        if self.problem.no_update and self.problem.local_problem.problem.stats()['return_status'] == 'Solve_Succeeded':
            self.problem.no_update = False
        elif not self.problem.local_problem.problem.stats()['return_status'] == 'Solve_Succeeded':
            # problem was not solved well, don't go to next segment
            import pdb; pdb.set_trace()  # breakpoint f9e5f88a //
            self.problem.no_update = True
            # reset saved trajectories
            # i.e. trajectories starting at the starting point of trajectory that was not successfully computed
            self.state_traj = np.array(state_traj_old)
            self.input_traj = np.array(input_traj_old)
            self.dinput_traj = np.array(dinput_traj_old)
            self.ddinput_traj = np.array(ddinput_traj_old)

            # reset states and inputs
            self.states_end = states  # + np.random.rand(3,)*1e-5  # perturb initial state randomly

            # compute perturbed input, that lies on the line between the last two inputs of the input traj
            inputs = [0, 0, 0]  # initialize
            # draw line between last and second last input to compute the y-coordinate of
            # the slightly changed initial point, that is right outside the connection of these two points
            x1, y1, z1 = self.input_traj[:,-2]  # second last point
            x2, y2, z2 = self.input_traj[:,-1]  # last point
            inputs[0] = x2+(x2-x1)*0.01  # perturb
            if (abs(y2 - y1) > 1e-3):  # line is not vertical
                a = (y2-y1)/(x2-x1)  # slope
                b = -x1*a+y1  # offset
                inputs[1] = a*inputs[0] + b
            else:
                inputs[1] = y1
            self.input_traj[:,-1] = inputs  # replace the old 'last point'

            # inputs = inputs_old + np.random.rand(3,)*1e-4  # perturb initial input randomly
            self.dinputs_end = dinputs
            self.ddinputs_end = ddinputs

            # reset time to previous value
            if self.current_time != 0.:
                self.current_time -= self.problem.motion_times[0]
            if current_time != 0.:
                current_time -= self.problem.motion_times[0]

            self.cnt += 1
            # if it takes too many iterations, stop
            if self.cnt > 20:
                raise RuntimeError('Couldn\'t find a feasible trajectory for this segment, stopping calculations')
                return

            # remove the motion time that was computed for the segment that was not accepted
            self.problem.motion_time_log.pop()
        else:  # user was happy or optimal solution found, just continue
            self.cnt = 0

    def plot_results(self, current_time, update_time):
        n_t = self.state_traj.shape[1]  # amount of points in trajectory
        time = np.linspace(0, current_time+update_time, n_t)  # make time vector

        plt.figure(2)
        plt.subplot(3, 1, 1)
        plt.cla()
        plt.plot(time, self.state_traj[0, :])
        plt.ylabel('x[mm]')
        plt.subplot(3, 1, 2)
        plt.cla()
        plt.plot(time, self.state_traj[1, :])
        plt.ylabel('y[mm]')
        plt.subplot(3, 1, 3)
        plt.cla()
        plt.plot(time, self.state_traj[2, :])
        plt.ylabel('z[mm]')
        plt.pause(0.1)

        plt.figure(3)
        plt.subplot(3, 1, 1)
        plt.cla()
        plt.plot(time, self.input_traj[0, :])
        plt.ylabel('vx[mm/s]')
        plt.subplot(3, 1, 2)
        plt.cla()
        plt.plot(time, self.input_traj[1, :])
        plt.ylabel('vy[mm/s]')
        plt.subplot(3, 1, 3)
        plt.cla()
        plt.plot(time, self.input_traj[2, :])
        plt.ylabel('vz[mm/s]')
        plt.pause(0.1)

        plt.figure(4)
        plt.cla()
        # plot total velocity
        plt.plot(time, np.sqrt(self.input_traj[0, :]**2+self.input_traj[1, :]**2))
        plt.ylabel('v[mm/s]')
        plt.pause(0.1)

        plt.figure(5)
        plt.subplot(3, 1, 1)
        plt.cla()
        plt.plot(time, self.dinput_traj[0, :])
        plt.ylabel('ax[mm/s^2]')
        plt.subplot(3, 1, 2)
        plt.cla()
        plt.plot(time, self.dinput_traj[1, :])
        plt.ylabel('ay[mm/s^2]')
        plt.subplot(3, 1, 3)
        plt.cla()
        plt.plot(time, self.dinput_traj[2, :])
        plt.ylabel('az[mm/s^2]')
        plt.pause(0.1)

        plt.figure(6)
        plt.subplot(3, 1, 1)
        plt.cla()
        plt.plot(time, self.ddinput_traj[0, :])
        plt.ylabel('jx[mm/s^3]')
        plt.subplot(3, 1, 2)
        plt.cla()
        plt.plot(time, self.ddinput_traj[1, :])
        plt.ylabel('jy[mm/s^3]')
        plt.subplot(3, 1, 3)
        plt.cla()
        plt.plot(time, self.ddinput_traj[2, :])
        plt.ylabel('jz[mm/s^3]')
        plt.pause(0.1)

        plt.figure(7)
        plt.cla()
        # plot trajectory
        plt.plot(self.state_traj[0, :], self.state_traj[1, :])
        plt.xlabel('x[mm]')
        plt.ylabel('y[mm]')
        # plot future trajectory
        eval = np.linspace(0,1,100)
        future_splines = self.problem.vehicles[0].result_spline_segments[1:]
        plt.figure(7)
        for spline in future_splines:
            plt.plot(spline[0](eval),spline[1](eval),color='gray')
        # plot environment
        for room in self.problem.environment.room:
            points = room['shape'].draw(room['pose'][:2]+[0])[0][0]  # no extra rotation to plot
            # add first point again to close shape
            points = np.c_[points, [points[0,0], points[1,0]]]
            plt.plot(points[0,:], points[1,:], color='red', linestyle = '--', linewidth= 1.2)
            # plot GCode center points
            # plt.plot(room['start'][0], room['start'][1], 'gx')
            # plt.plot(room['end'][0], room['end'][1], 'gx')
        plt.pause(0.1)

    def save_results(self, count=0, reorder=False):
        # write results to file
        if not reorder:
            data = np.c_[self.state_traj[0,:], self.input_traj[0,:], self.dinput_traj[0,:],
                         self.state_traj[1,:], self.input_traj[1,:], self.dinput_traj[1,:],
                         self.state_traj[2,:], self.input_traj[2,:], self.dinput_traj[2,:]]  # pos, vel, acc in xyz
        else:
            # reorder the data, x and z are switched
            data = np.c_[self.state_traj[2,:], self.input_traj[2,:], self.dinput_traj[2,:],
                         self.state_traj[1,:], self.input_traj[1,:], self.dinput_traj[1,:],
                         self.state_traj[0,:], self.input_traj[0,:], self.dinput_traj[0,:]]  # pos, vel, acc in xyz
        np.savetxt('trajectories_'+str(count)+'.csv', data , delimiter=',')