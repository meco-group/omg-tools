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
import pickle

from ..basics.spline import BSpline


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
        self.problem.no_update = False

        # create variables to save all data
        self.motion_times = []
        self.result_coeffs = []

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
                ddinputs = None
                enforce_inputs = False  # since there are no inputs yet
                current_time = 0.
                trajectories = self.update(current_time, states=states, inputs=inputs, dinputs=dinputs, update_time=update_time, enforce_states=True, enforce_inputs=enforce_inputs)
            else:
                update_time = 0.  # this input to the function is not used, and if used it would be wrong
                current_time += self.problem.motion_times[0]
                enforce_inputs = True

                # set new values at end values of previous iteration
                states = np.array(states_end)
                inputs = np.array(inputs_end)
                dinputs = np.array(dinputs_end)
                ddinputs = np.array(ddinputs_end)

                trajectories = self.update(current_time, states=states, inputs=inputs, dinputs=dinputs, update_time=update_time, enforce_states=True, enforce_inputs=enforce_inputs)
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
                states_end = np.hstack([s(1.) for s in pos_splines])
                inputs_end = np.hstack([s(1.) for s in input_splines])*1./self.problem.motion_times[0]
                dinputs_end = np.hstack([s(1.) for s in dinput_splines])*1./self.problem.motion_times[0]**2
                ddinputs_end = np.hstack([s(1.) for s in ddinput_splines])*1./self.problem.motion_times[0]**3

                # save old values
                # these are the ones that end at the starting state of current iteration
                state_traj_old = np.array(state_traj)
                input_traj_old = np.array(input_traj[:])
                dinput_traj_old = np.array(dinput_traj[:])
                ddinput_traj_old = np.array(ddinput_traj[:])

                # state trajectory, append current state because this is not necessarily reached at a multiple of sample_time
                state_traj = np.c_[state_traj, trajectories['state'][:, 1:n_samp+1], states_end]
                # input trajectory, append current input
                input_traj = np.c_[input_traj, trajectories['input'][:, 1:n_samp+1], inputs_end]
                dinput_traj = np.c_[dinput_traj, trajectories['dinput'][:, 1:n_samp+1], dinputs_end]
                ddinput_traj = np.c_[ddinput_traj, trajectories['ddinput'][:, 1:n_samp+1], ddinputs_end]
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
                # plot future trajectory
                eval = np.linspace(0,1,100)
                future_splines = self.problem.vehicles[0].result_spline_segments[1:]
                for spline in future_splines:
                    plt.figure(6)
                    plt.plot(spline[0](eval),spline[1](eval),color='gray')
                # plot environment
                for room in self.problem.environment.room:
                    points = room['shape'].draw(room['pose'][:3])[0][0]
                    # add first point again to close shape
                    points = np.c_[points, [points[0,0], points[1,0]]]
                    plt.plot(points[0,:], points[1,:], color='red', linestyle = '--', linewidth= 1.2)
                plt.figure(7)
                plt.cla()
                plt.plot(time, np.sqrt(input_traj[0, :]**2+input_traj[1, :]**2))
                plt.pause(0.1)

                # save results in big list
                self.result_coeffs.append([s.coeffs for s in self.problem.vehicles[0].result_spline_segments[0]])
                self.motion_times.append(self.problem.motion_times[0])

                # Neglect bad segment:
                # if the latest computation step gave worse results than before, don't use it
                # reset time to the lowest calculated time for this segment
                # reset coefficients of spline to those of the corresponding segment
                # add this trajectory to states, inputs,...
                # motion_time_log = np.array([np.zeros(1,self.problem.cnt), np.zeros(self.problem.n_segments)])
                # motion_time_log[1,:] = self.problem.motion_times
                # what about the initial guess? make new one, or re-use the one of the 'bad' segment?


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
                    self.problem.no_update = True
                    # reset saved trajectories
                    # i.e. trajectories starting at the starting point of trajectory that was not successfully computed
                    state_traj = np.array(state_traj_old)
                    input_traj = np.array(input_traj_old)
                    dinput_traj = np.array(dinput_traj_old)
                    ddinput_traj = np.array(ddinput_traj_old)

                    # reset states and inputs
                    states_end = states  # + np.random.rand(3,)*1e-5  # perturb initial state randomly

                    # compute perturbed input, that lies on the line between the last two inputs of the input traj
                    inputs = [0, 0, 0]  # initialize
                    # draw line between last and second last input to compute the y-coordinate of
                    # the slightly changed initial point, that is right outside the connection of these two points
                    x1, y1, z1 = input_traj[:,-2]  # second last point
                    x2, y2, z2 = input_traj[:,-1]  # last point
                    inputs[0] = x2+(x2-x1)*0.01  # perturb
                    if (abs(y2 - y1) > 1e-3):  # line is not vertical
                        a = (y2-y1)/(x2-x1)  # slope
                        b = -x1*a+y1  # offset
                        inputs[1] = a*inputs[0] + b
                    else:
                        inputs[1] = y1
                    input_traj[:,-1] = inputs  # replace the old 'last point'

                    # inputs = inputs_old + np.random.rand(3,)*1e-4  # perturb initial input randomly
                    dinputs_end = dinputs
                    ddinputs_end = ddinputs

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
                    # remove data that was saved for segment that was not accepted
                    self.result_coeffs.pop()
                    self.motion_times.pop()
                else:  # user was happy or optimal solution found, just continue
                    self.cnt = 0

                # check if target is reached
                if (np.linalg.norm(self.problem.segments[-1]['end']-state_traj[:, -1]) < 1e-2 and np.linalg.norm(input_traj[:, -1]) < 1e-2):
                    target_reached = True

        # target reached, print final information
        self.problem.final()

        # save results:
        ## 1) save state_traj
        # self.state_traj = state_traj
        # self.input_traj = input_traj
        # self.dinput_traj = dinput_traj
        ## 2) rebuild
        self.save_splines()

    def save_splines(self):
        # save coefficients of splines, such that you can rebuild them later
        file = open('result_coeffs.pickle','wb')
        pickle.dump(self.result_coeffs,file, protocol=pickle.HIGHEST_PROTOCOL)
        file.close()

        file = open('result_times.pickle','wb')
        pickle.dump(self.motion_times,file, protocol=pickle.HIGHEST_PROTOCOL)
        file.close()

    def generate_setpoints(self, sample_time=None, type='pos'):

        if sample_time is None:
            sample_time = self.sample_time

        basis = self.problem.vehicles[0].basis

        # read in coeffs, make splines of it and evaluate them to obtain the setpoints with the required sampling frequency

        # we open the file for reading
        file = open('result_coeffs.pickle','r')
        # load the object from the file into var b
        result_coeffs = pickle.load(file)
        file.close()
        file = open('result_times.pickle', 'r')
        motion_times = pickle.load(file)

        pos_splines = np.array([[],[]])
        vel_splines = np.array([[],[]])
        acc_splines = np.array([[],[]])

        for idx, coeffs in enumerate(result_coeffs):
            # don't include last point, since this would give a double evaluation: end of spline1 = begin of spline2
            n_samp = int(np.round(motion_times[idx]/self.sample_time, 6))
            eval = np.linspace(0,1-sample_time,n_samp)

            spline1 = BSpline(basis, coeffs[0])
            dspline1 = spline1.derivative(1)*(1./motion_times[idx])
            ddspline1 = spline1.derivative(2)*(1./motion_times[idx]**2)

            spline2 = BSpline(basis, coeffs[1])
            dspline2 = spline2.derivative(1)*(1./motion_times[idx])
            ddspline2 = spline2.derivative(2)*(1./motion_times[idx]**2)

            pos_splines = np.c_[pos_splines, np.array([spline1, spline2])]
            vel_splines = np.c_[vel_splines, np.array([dspline1, dspline2])]
            acc_splines = np.c_[acc_splines, np.array([ddspline1, ddspline2])]

        if type == 'pos':
            pos_x = [p(eval) for p in pos_splines[0,:]]
            pos_y = [p(eval) for p in pos_splines[1,:]]
            return np.r_[pos_x, pos_y]
        elif type == 'vel':
            vel_x = [p(eval) for p in vel_splines[0,:]]
            vel_y = [p(eval) for p in vel_splines[:,1]]
            return np.r_[vel_x, vel_y]
        elif type == 'acc':
            acc_x = [p(eval) for p in acc_splines[0,:]]
            acc_y = [p(eval) for p in acc_splines[:,1]]
            return np.r_[acc_x, acc_y]
        else:
            raise RuntimeError('Invalid type selected: ', type)