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

from plots import Plots


class Simulator:

    def __init__(self, problem):
        self.problem = problem
        self.plot = Plots(problem)

    def run(self):
        current_time = 0.
        self.problem.initialize()
        stop = False
        while not stop:
            stop, current_time = self.update(current_time)
        self.problem.final()

    def update(self, current_time):
        # solve problem
        self.problem.solve(current_time)
        # update everything
        current_time = self.problem.update(current_time)
        self.plot.update()
        # check termination criteria
        stop = self.problem.stop_criterium()
        return stop, current_time

    def run_once(self):
        # solve problem
        self.problem.solve(0.)
        # update everything
        self.problem.update(0., full_update=True)
        self.plot.update()
        self.problem.final()
        # return trajectories
        trajectories = {}
        for vehicle in self.problem.vehicles:
            trajectories[str(vehicle)] = vehicle.trajectories
        return trajectories
