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
from ..basics import GraphicalDebugger


class Simulator:

    def __init__(self, problem, sample_time=0.01, update_time=0.1, options=None):
        options = options or {}
        self.set_default_options()
        self.set_options(options)
        self.problem = problem
        PlotLayer.simulator = self
        
        if self.options['debugging']:
            # pass on debugging option to problem
            self.problem.options['debugging'] = True

            # gather number of variables, constraints, parameters
            # Todo: gather automatically
            nx = 35 #problem.problem.size_in('x0')[0]
            ng = 514 #problem.problem.size_in('ubg')[0]
            np = 19 #problem.problem.size_in('p')[0]

            # make graphical debugger and pass to problem
            graph_debugger = GraphicalDebugger('debugger', self, nx, ng, np)
            self.problem.graphical_debugger = graph_debugger
 
        self.deployer = Deployer(self.problem, sample_time, update_time)
        self.update_time = update_time
        self.sample_time = sample_time

        self.reset_timing()

    def set_default_options(self):
        self.options = {'debugging': False}

    def set_options(self, options):
        if 'debugging' in options:
            self.options['debugging'] = options['debugging']

    def set_problem(self, problem):
        self.deployer.set_problem(problem)
        self.problem = problem

    def run(self, init_reset=True):
        if init_reset:
            self.deployer.reset()
        stop = False
        while not stop:            
            stop = self.update()
            if stop:
                update_time = float(self.problem.vehicles[0].signals['time'][:, -1] - self.current_time)
                self.update_timing(update_time)
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

    def run_once(self, simulate=True, init_reset=True, **kwargs):
        if 'hard_stop' in kwargs:
            hard_stop = kwargs['hard_stop']
        else:
            hard_stop = None
        if init_reset:
            self.deployer.reset()
        self.deployer.update(self.current_time, None, np.inf)
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
