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
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
import numpy as np
import tf
from std_msgs.msg import Bool
from p3dx_motionplanner.msg import Trigger, FleetTrajectories, P3DXPose, Obstacle, Room, Settings


class Controller(object):
    _cmd_twist = Twist()
    _trigger = Trigger()

    def __init__(self, sample_time, update_time, n_robots, obst_traj=[]):
        rospy.init_node('p3dx_controller')
        self._sample_time = sample_time
        self._update_time = update_time
        self._mp_status = False
        self._n_robots = n_robots
        self._obst_traj = obst_traj
        self._robobst = list(obst_traj.keys())
        self._robot_est_pose = [[0., 0., 0.] for k in range(n_robots)]
        self._robot_real_pose = [[0., 0., 0.] for k in range(n_robots)]
        self._robobst_est_pose = [[0., 0.] for k in range(len(self._robobst))]
        self._robobst_est_velocity = [[0., 0.] for k in range(len(self._robobst))]
        self._vel_traj = [{'v': [], 'w': []} for k in range(n_robots)]
        self._vel_traj_applied = [{'v': [], 'w': []} for k in range(n_robots)]
        self._cmd_vel_topic = [rospy.Publisher('robot'+str(k)+'/p3dx/cmd_vel', Twist, queue_size=1) for k in range(n_robots)]
        self._cmd_vel_robobst_topic = [rospy.Publisher('robobst'+str(l)+'/p3dx/cmd_vel', Twist, queue_size=1) for l in range(len(self._robobst))]

        self._mp_trigger_topic = rospy.Publisher('mp_trigger', Trigger, queue_size=1)
        self._mp_configure_topic = rospy.Publisher('mp_configure', Settings, queue_size=1)
        rospy.Subscriber('mp_result', FleetTrajectories, self.get_mp_result)
        rospy.Subscriber('mp_feedback', Bool, self.get_mp_feedback)
        # for k in range(n_robots):
        #     rospy.Subscriber('robot'+str(k)+'/p3dx/base_pose_ground_truth', Odometry, callback=self.get_est_pose, callback_args=k)
        rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.get_model_states)
        # for l in range(len(self._robobst)):
        #     rospy.Subscriber('robobst'+str(l)+'/p3dx/base_pose_ground_truth', Odometry, callback=self.get_est_pose_robobst, callback_args=l)

    def get_model_states(self, data):
        for k in range(self._n_robots):
            index = data.name.index('p3dx'+str(k))
            self._robot_est_pose[k][0] = data.pose[index].position.x
            self._robot_est_pose[k][1] = data.pose[index].position.y
            qt = data.pose[index].orientation
            r, p, y = tf.transformations.euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
            self._robot_est_pose[k][2] = y
        for k in range(len(self._robobst)):
            index = data.name.index('p3dx_obs'+str(k))
            self._robobst_est_pose[k] = [data.pose[index].position.x, data.pose[index].position.y]
            self._robobst_est_velocity[k] = [data.twist[index].linear.x, data.twist[index].linear.y]

    # def get_est_pose(self, data, k):
    #     self._robot_est_pose[k][0] = data.pose.pose.position.x
    #     self._robot_est_pose[k][1] = data.pose.pose.position.y
    #     qt = data.pose.pose.orientation
    #     r, p, y = tf.transformations.euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
    #     self._robot_est_pose[k][2] = y

    # def get_est_pose_robobst(self, data, k):
    #     self._robobst_est_pose[k][0] = data.pose.pose.position.x
    #     self._robobst_est_pose[k][1] = data.pose.pose.position.y
    #     self._robobst_est_velocity[k][0] = data.twist.twist.linear.x
    #     self._robobst_est_velocity[k][1] = data.twist.twist.linear.y

    # def get_real_pose(self, data, k):
    #     self._robot_real_pose[k][0] = data.pose[1].position.x
    #     self._robot_real_pose[k][1] = data.pose[1].position.y
    #     qt = data.pose[1].orientation
    #     r, p, y = tf.transformations.euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
    #     self._robot_real_pose[k][2] = y

    def get_mp_feedback(self, data):
        self._mp_status = data

    def get_mp_result(self, data):
        print('got result!')
        v_traj = [data.trajectories[k].v_traj for k in range(self._n_robots)]
        w_traj = [data.trajectories[k].w_traj for k in range(self._n_robots)]
        self.store_trajectories(v_traj, w_traj)

    def update(self):
        pose0 = [self._robot_est_pose[k][:] for k in range(self._n_robots)]
        if self._init:
            if not self._new_trajectories:
                return
            self._index = int(self._update_time/self._sample_time)
            self._init = False
        if self._index >= int(self._update_time/self._sample_time):
            if self._new_trajectories:
                # load fresh trajectories
                self.load_trajectories()
                self._new_trajectories = False
                self._time += self._index*self._sample_time
                self._index = 0
                # trigger motion planner
                self.fire_motionplanner(self._time, pose0)
            else:
                print('overtime!')
        # send velocity sample
        for k in range(self._n_robots):
            self._cmd_twist.linear.x = self._vel_traj[k]['v'][self._index]
            self._cmd_twist.angular.z = self._vel_traj[k]['w'][self._index]
            self._cmd_vel_topic[k].publish(self._cmd_twist)
            self._vel_traj_applied[k]['v'].append(self._cmd_twist.linear.x)
            self._vel_traj_applied[k]['w'].append(self._cmd_twist.angular.z)
        for l, k in enumerate(self._robobst):
            if (self._time) >= self._obst_traj[k]['t']:
                cmd_twist = Twist()
                cmd_twist.linear.x = self._obst_traj[k]['v']
                cmd_twist.angular.z = self._obst_traj[k]['w']
                self._cmd_vel_robobst_topic[l].publish(cmd_twist)
        self._index += 1

    def load_trajectories(self):
        for k in range(self._n_robots):
            self._vel_traj[k]['v'] = self._vel_traj_strg[k]['v'][:]
            self._vel_traj[k]['w'] = self._vel_traj_strg[k]['w'][:]

    def store_trajectories(self, v_traj, w_traj):
        self._vel_traj_strg = [{} for _ in range(self._n_robots)]
        for k in range(self._n_robots):
            self._vel_traj_strg[k] = {'v': v_traj[k], 'w': w_traj[k]}
        self._new_trajectories = True

    def proceed(self):
        if len(self._vel_traj_applied[0]['v']) == 0:
            return True
        stop = True
        for k in range(self._n_robots):
            pos_nrm = np.linalg.norm(np.array(self._robot_est_pose[k]) - np.array(self._goal[k].pose))
            vel_nrm = np.linalg.norm([self._vel_traj_applied[k]['v'][-1], self._vel_traj_applied[k]['w'][-1]])
            stop *= (pos_nrm < 0.1 and vel_nrm < 0.1)
        return not stop

    def set_goal(self, goal):
        self._goal = goal
        self._time = 0.
        pose0 = self._robot_est_pose[:]
        self._new_trajectories = False
        self.fire_motionplanner(self._time, pose0)
        self._init = True

    def fire_motionplanner(self, time, pose0):
        print('firing!')
        self._trigger.goal = self._goal
        self._trigger.state = [P3DXPose(pose0[k][:]) for k in range(self._n_robots)]
        self._trigger.obstacles = [Obstacle(pose=self._robobst_est_pose[k], velocity=self._robobst_est_velocity[k]) for k in range(len(self._robobst))]
        self._trigger.current_time = time
        self._mp_trigger_topic.publish(self._trigger)

    def start(self):
        rate = rospy.Rate(1./self._sample_time)
        proceed = True
        while (not self._mp_status):
            rate.sleep()
        print('controller started!')
        self.set_goal(self._settings.terminal_pose)
        k = 0
        while (proceed):
            k += 1
            controller.update()
            proceed = controller.proceed()
            rate.sleep()
        for k in range(self._n_robots):
            self._cmd_twist.linear.x = 0.
            self._cmd_twist.angular.z = 0.
            self._cmd_vel_topic[k].publish(self._cmd_twist)
        print('target reached!')

    def init_gazebo(self, st):
        rospy.set_param('gazebo/use_sim_time', True)
        try:
            ssm = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException as e:
            print('Service call failed: %s' % (e))
        for k in range(self._n_robots):
            pose0, twist0 = Pose(), Twist()
            pose0.position.x = st.init_pose[k].pose[0]
            pose0.position.y = st.init_pose[k].pose[1]
            x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, st.init_pose[k].pose[2])
            pose0.orientation.x = x
            pose0.orientation.y = y
            pose0.orientation.z = z
            pose0.orientation.w = w
            twist0.linear.x = 0.
            twist0.angular.z = 0.
            mod0 = ModelState('p3dx'+str(k), pose0, twist0, 'world')
            ssm(mod0)
        for l, k in enumerate(st.robobst):
            pose0, twist0 = Pose(), Twist()
            pose0.position.x = st.obstacles[k].pose[0]
            pose0.position.y = st.obstacles[k].pose[1]
            x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, st.obstacles[k].pose[2])
            pose0.orientation.x = x
            pose0.orientation.y = y
            pose0.orientation.z = z
            pose0.orientation.w = w
            twist0.linear.x = 0.
            twist0.angular.z = 0.
            mod0 = ModelState('p3dx_obs'+str(l), pose0, twist0, 'world')
            ssm(mod0)

    def configure(self):
        print('configure controller')
        st = Settings()
        # timing
        st.sample_time = self._sample_time
        st.update_time = self._update_time
        # robots
        configuration = omg.RegularPolyhedron(0.5, self._n_robots, np.pi).vertices.T
        configurationT = omg.RegularPolyhedron(0.5, self._n_robots, np.pi/2.).vertices.T
        init_ctr = [-3.5, -1.]
        terminal_ctr = [3.5, 1.]
        st.fleet_config = [P3DXPose(pose=[c[0], c[1], 0.]) for c in configuration]
        st.init_pose = [P3DXPose(pose=[init_ctr[0]+c[0], init_ctr[1]+c[1], np.pi/2.]) for c in configuration]
        st.terminal_pose = [P3DXPose(pose=[terminal_ctr[0]+c[0], terminal_ctr[1]+c[1], 0.]) for c in configurationT]
        # environment
        st.room = Room(position=[0., 0.], shape=[10., 5.])
        obstacles = []
        obstacles.append(Obstacle(pose=[-2., -2.3, np.pi/2.], shape=[4., 0.1]))
        obstacles.append(Obstacle(pose=[0.5, -1.5, np.pi/2.], shape=[0.35]))
        st.obstacles = obstacles
        st.robobst = self._robobst
        # set motionplanner
        self._mp_configure_topic.publish(st)
        # init gazebo
        self.init_gazebo(st)
        self._settings = st

obst_traj = {1: {'t': 10., 'v': 0.3, 'w': 0.}}

if __name__ == '__main__':
    sample_time = 0.01
    update_time = 0.5
    controller = Controller(sample_time, update_time, n_robots=3, obst_traj=obst_traj)
    rospy.sleep(0.5)
    controller.configure()
    controller.start()
