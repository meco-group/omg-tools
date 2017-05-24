#!/usr/bin/env python

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


import roslib; roslib.load_manifest('p3dx_motionplanner')
import omgtools as omg
import rospy
import numpy as np
from std_msgs.msg import Bool
from p3dx_motionplanner.msg import Trigger, FleetTrajectories, P3DXTrajectory, Settings


class MotionPlanner(object):
    _result = FleetTrajectories

    def __init__(self):
        self._mp_result_topic = rospy.Publisher('mp_result', FleetTrajectories, queue_size=1)
        self._mp_feedback_topic = rospy.Publisher('mp_feedback', Bool, queue_size=1)
        rospy.Subscriber('mp_trigger', Trigger, self.update)
        rospy.Subscriber('mp_configure', Settings, self.configure)

    def configure(self, st):
        print 'configure motionplanner'
        # timing
        self._sample_time = st.sample_time
        self._update_time = st.update_time
        # robots
        self._n_robots = len(st.fleet_config)
        self._vehicles = [omg.Dubins(shapes=omg.Circle(0.35), options={'degree': 2}, bounds={'vmax': 0.5, 'wmax': np.pi/3., 'wmin': -np.pi/3.}) for k in range(self._n_robots)]
        for k in range(self._n_robots):
            self._vehicles[k].define_knots(knot_intervals=10)
        self._fleet = omg.Fleet(self._vehicles)
        if self._n_robots == 1:
            self._fleet.set_initial_conditions([[0., 0., 0.]])
            self._fleet.set_terminal_conditions([[0., 0., 0.]])
        else:
            init_pose = [[0., 0., 0.] for k in range(self._n_robots)]
            terminal_pose = [[0., 0., 0.] for k in range(self._n_robots)]
            self._fleet.set_configuration([c.pose for c in st.fleet_config], orientation=st.init_pose[0].pose[2])
            self._fleet.set_initial_conditions(init_pose)
            self._fleet.set_terminal_conditions(terminal_pose)
        # environment
        room = {'shape': omg.Rectangle(st.room.shape[0], st.room.shape[1]), 'position': [st.room.position[0], st.room.position[1]]}
        self._obstacles = []
        for k, obst in enumerate(st.obstacles):
            if len(obst.shape) == 1:
                shape = omg.Circle(obst.shape[0])
            elif len(obst.shape) == 2:
                shape = omg.Beam(width=obst.shape[0], height=obst.shape[1], orientation=obst.pose[2])
            self._obstacles.append(omg.Obstacle({'position': [obst.pose[0], obst.pose[1]]}, shape=shape))

        environment = omg.Environment(room=room)
        environment.add_obstacle(self._obstacles)
        self._robobst = st.robobst
        # create problem
        print 'creating problem'
        if self._n_robots == 1:
            problem = omg.Point2point(self._fleet, environment, freeT=False)
            problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.hessian_approximation': 'limited-memory'}}})
            problem.set_options({'hard_term_con': False, 'horizon_time': 10.})
        else:
            options = {'rho': 5., 'horizon_time': 35., 'hard_term_con': True, 'init_iter':5}
            problem = omg.FormationPoint2point(self._fleet, environment, options=options)
            problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57', 'ipopt.max_iter': 500}}})
        problem.init()
        self._deployer = omg.Deployer(problem, self._sample_time, self._update_time)
        self._deployer.reset()
        self._mp_feedback_topic.publish(True)

    def start(self):
        rospy.init_node('p3dx_motionplanner')
        self._goal = [[np.inf, np.inf, np.inf] for k in range(3)]
        print 'listening'
        rospy.spin()

    def update(self, cmd):
        print 'started motion planning update!'
        reset = False
        for k, vehicle in enumerate(self._fleet.vehicles):
            if cmd.goal[k].pose != self._goal[k]:
                self._goal[k] = cmd.goal[k].pose[:]
                self._vehicles[k].set_initial_conditions(cmd.state[k].pose[:])
                self._vehicles[k].set_terminal_conditions(cmd.goal[k].pose[:])
                reset = True
        if reset:
            self._deployer.reset()
            print 'resetted deployer!'
        state0 = [cmd.state[k].pose[:] for k in range(self._n_robots)]
        for l, k in enumerate(self._robobst):
            pos = cmd.obstacles[l].pose[:2]
            vel = cmd.obstacles[l].velocity[:2]
            pos = np.round(pos, 1)
            vel = np.round(vel, 1)
            for n in range(self._n_robots):
                self._deployer.problem.problems[n].environment.obstacles[k].set_state({'position': pos, 'velocity': vel})
        trajectories = self._deployer.update(cmd.current_time, state0)
        if self._n_robots == 1:
            self._result = [P3DXTrajectory(v_traj=trajectories['input'][0, :], w_traj=trajectories['input'][1, :])]
        else:
            self._result = [P3DXTrajectory(v_traj=trajectories['vehicle'+str(k)]['input'][0, :], w_traj=trajectories['vehicle'+str(k)]['input'][1, :]) for k in range(self._n_robots)]
        self._mp_result_topic.publish(self._result)

if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
