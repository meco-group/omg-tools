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

import threading
import numpy as np
import omgtools as omg


class MotionPlanningThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.mp_trigger = threading.Event()
        self.ready = threading.Event()
        self.proceed = threading.Event()
        self.mp_trigger.clear()
        self.ready.clear()
        self.proceed.set()

    def set_controller(self, controller):
        self.controller = controller

    def configure(self, sample_time, update_time):
        # create vehicle
        self.quad = omg.Quadrotor3D(0.5, bounds={'u1max': 15., 'u2min': -0.5, 'u2max': 0.5, 'u3min': -0.5, 'u3max': 0.5})
        self.quad.set_options({'safety_distance': 0.1, 'safety_weight': 10, 'room_constraints': False, 'substitution': False})
        self.quad.set_initial_conditions([0, 0, 0, 0, 0, 0, 0, 0])
        self.quad.set_terminal_conditions([0, 0, 0])
        # create environment
        environment = omg.Environment(room={'shape': omg.Cuboid(8, 6, 8)})
        # create a point-to-point problem
        problem = omg.Point2point(self.quad, environment, freeT=False)
        problem.set_options({'horizon_time': 15, 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
        problem.init()
        self.quad.problem = problem
        # create deployer
        self.deployer = omg.Deployer(problem, sample_time, update_time)
        # default goal
        self._goal = np.r_[0, 0, 0]

    def run(self):
        while(self.proceed.is_set()):
            self.mp_trigger.wait()
            self.mp_trigger.clear()
            self.update()
            self.controller.store_trajectories(self.input, self.state)
            self.ready.set()
        print 'stopped motion planner'

    def trigger(self, time, state0):
        self.ready.clear()
        self.mp_trigger.set()
        self.time = time
        self.state0 = state0

    def set_goal(self, goal):
        self.goal = goal
        self.new_goal = True

    def update(self):
        if self.new_goal:
            self.new_goal = False
            self.quad.set_initial_conditions(self.state0)
            self.quad.set_terminal_conditions(self.goal)
            self.deployer.reset()
            print 'resetted deployer!'

        trajectories = self.deployer.update(self.time, self.state0)
        self.input = trajectories['input']
        self.state = trajectories['state']

    def get_trajectories(self):
        return self.input, self.state
