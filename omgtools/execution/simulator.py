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
from deployer import Deployer
from plotlayer import PlotLayer


class Simulator:

    def __init__(self, problem, sample_time=0.01, update_time=0.1):
        self.deployer = Deployer(problem, sample_time, update_time)
        self.update_time = update_time
        self.sample_time = sample_time
        self.problem = problem
        PlotLayer.simulator = self
        self.reset_timing()

    def set_problem(self, problem):
        self.deployer.set_problem(problem)
        self.problem = problem

    def run(self):
        self.deployer.reset()
        stop = False
        while not stop:
            stop = self.update()
            ### adapted ###
            if (stop or self.update_time - float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time)) > self.sample_time:
                update_time = float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time)
                self.update_timing(update_time-self.sample_time) #correcting for first time
            else:
                self.update_timing()

        self.problem.final()
        # return trajectories and signals
        trajectories, signals = {}, {}
        if len(self.problem.vehicles) == 1:
            return self.problem.vehicles[0].traj_storage, self.problem.vehicles[0].signals
        else:
            for vehicle in self.problem.vehicles:
                trajectories[str(vehicle)] = vehicle.traj_storage
                signals[str(vehicle)] = vehicle.signals
        return trajectories, signals

    def step(self, update_time=0.1):
        stop = self.update()
        if stop:
            update_time = float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time)
            self.update_timing(update_time)
            self.problem.final()
        else:
            self.update_timing(update_time)
        # return trajectories and signals
        trajectories, signals, curr_state = {}, {}, {}
        # determine remaining motion time
        # depends on problem type, for fixedT motion_time is always = horizon_time
        # local import required to avoid circular dependency
        from ..problems.point2point import FixedTPoint2point, FreeTPoint2point
        if isinstance(self.problem, FixedTPoint2point):
            motion_time = self.problem.options['horizon_time']
        elif isinstance(self.problem, FreeTPoint2point):
            motion_time = self.problem.father.get_variables(self.problem, 'T')[0][0]

        if len(self.problem.vehicles) == 1:
            return self.problem.vehicles[0].signals['state'][:,-1], self.current_time, motion_time, stop, self.problem.vehicles[0].traj_storage, self.problem.vehicles[0].signals
        else:
            for vehicle in self.problem.vehicles:
                trajectories[str(vehicle)] = vehicle.traj_storage
                signals[str(vehicle)] = vehicle.signals
                curr_state[str(vehicle)] = vehicle.signals['state'][:,-1]
        return curr_state, self.current_time, motion_time, stop, trajectories, signals

    def update(self):
        # update deployer
        self.deployer.update(self.current_time)
        # simulate problem
        self.problem.simulate(self.current_time, self.update_time, self.sample_time)
        # check stop condition
        stop = self.problem.stop_criterium(self.current_time, self.update_time)
        return stop

    def reset_timing(self):
        self.current_time = 0.
        self.time = np.r_[0.]

    def update_timing(self, update_time=None):
        update_time = self.update_time if not update_time else update_time
        self.current_time += update_time
        n_samp = int(np.round(update_time/self.sample_time, 6))
        self.time = np.r_[self.time, np.linspace(
            self.time[-1]+self.sample_time, self.time[-1]+n_samp*self.sample_time, n_samp)]

    def run_once(self, simulate=True, **kwargs):
        if 'hard_stop' in kwargs:
            hard_stop = kwargs['hard_stop']
        else:
            hard_stop = None
        self.deployer.reset()
        self.deployer.update(self.current_time, None, update_time=np.inf)
        if not simulate:
            return None
        if hard_stop:
            self.hard_stop(self.current_time, hard_stop['time'], hard_stop['perturbation'])
        else:
            self.problem.simulate(self.current_time, np.inf, self.sample_time)
        self.problem.final()
        # determine timing
        update_time = self.problem.vehicles[0].signals['time'][:, -1] - self.current_time
        self.update_timing(update_time)
        # return trajectories
        trajectories = {}
        if len(self.problem.vehicles) == 1:
            return self.problem.vehicles[0].trajectories
        else:
            for vehicle in self.problem.vehicles:
                trajectories[str(vehicle)] = vehicle.trajectories
        return trajectories

    def run_segment(self, simulate=True, **kwargs):
        self.deployer.reset()
        stop = False
        while not stop:
            if isinstance(self.problem.motion_times[0], (int,float)):
                self.update_time = self.problem.motion_times[0]

                # Todo: use self.problem.motion_times[1] here? then in deployer.update you can predict with the right time
                # in first iteration the used update time is just a guess
                # in second iteration, self.problem.motion_times has the two calculated values, but the first one is already used,
                # so actually you want to use the second one
                # in the end this update time is only used for state prediction, so not really used in our case, since we know
                # the state at end of segment
                # downside: now in the first iteration you will use the guess for the second segment...
            else:
                self.update_time = 0.1  # in first iteration make small step
            # update deployer
            self.deployer.update(self.current_time, None, self.update_time)
            self.update_time = self.problem.motion_times[0]
            # simulate problem
            self.problem.simulate(self.current_time, self.update_time, self.sample_time)

            # adapt state and input to the values at the beginning of the next trajectory, this is necessary because
            # the simulation can only reach points in time that are multiples of the sample time, such that a gap mostly
            # between the end position of the simulation and the point in which segments are connected, leading to problems
            # when constraints are active at the connection point between segments
            pos_splines = self.problem.vehicles[0].result_splines
            input_splines = [s.derivative(1) for s in pos_splines]
            dinput_splines = [s.derivative(2) for s in pos_splines]
            self.problem.vehicles[0].overrule_state(np.hstack([s(self.problem.motion_times[0]) for s in pos_splines]))
            self.problem.vehicles[0].overrule_input(np.hstack([s(self.problem.motion_times[0]) for s in input_splines]),
                                             dinput=np.hstack([s(self.problem.motion_times[0]) for s in dinput_splines]))

            # check stop condition
            stop = self.problem.stop_criterium(self.current_time, self.update_time)
            ### adapted ###
            if (stop or self.update_time - float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time)) > self.sample_time:
                update_time = float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time)
                self.update_timing(update_time-self.sample_time) #correcting for first time
            else:
                self.update_timing()
        self.problem.final()
        # return trajectories and signals
        trajectories, signals = {}, {}
        if len(self.problem.vehicles) == 1:
            return self.problem.vehicles[0].traj_storage, self.problem.vehicles[0].signals
        else:
            for vehicle in self.problem.vehicles:
                trajectories[str(vehicle)] = vehicle.traj_storage
                signals[str(vehicle)] = vehicle.signals
        return trajectories, signals

    def hard_stop(self, current_time, stop_time, perturbation):
        self.problem.simulate(current_time, stop_time, self.sample_time)
        for k, vehicle in enumerate(self.problem.vehicles):
            vehicle.overrule_state(vehicle.signals['state'][:, -1] + np.array(perturbation[k]))
            vehicle.overrule_input(np.zeros(len(vehicle.prediction['input'])))

    def sleep(self, sleep_time):
        self.problem.sleep(self.current_time, sleep_time, self.sample_time)
        self.update_timing(sleep_time)

    def time2index(self, time):
        Ts = self.sample_time
        for k, t in enumerate(self.time):
            t = np.round(t, 6)
            if (t <= time) and (time < (t+Ts)) and ((time-t) <= (t+Ts-time)):
                return k
