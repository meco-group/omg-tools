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

import time
import motionplanner
import numpy as np
import matplotlib.pyplot as plt
from dronekit import *
from pymavlink import *

def mix_with_white(color, perc_white=80.):
    r, g, b = color[0], color[1], color[2]
    r_m = ((100. - perc_white)*r + perc_white)/100.
    g_m = ((100. - perc_white)*g + perc_white)/100.
    b_m = ((100. - perc_white)*b + perc_white)/100.
    return [r_m, g_m, b_m]


class Controller(object):

    def __init__(self, sample_time, update_time):
        self.sample_time = sample_time
        self.update_time = update_time
        self.mp = motionplanner.MotionPlanningThread()
        self.mp.set_controller(self)
        self.mp.start()
        self.sim_vehicle = connect('127.0.0.1:14550', wait_ready = True, rate=20)
        self.sim_vehicle.add_attribute_listener('location.local_frame', self.store_position)
        self.sim_vehicle.add_attribute_listener('attitude', self.store_orientation)
        self.sim_vehicle.add_attribute_listener('velocity', self.store_velocity)
        self.simulation = {'time': np.zeros((1, 0)),
                           'velocity_ref': np.zeros((3, 0)), 'velocity': np.zeros((3, 0)),
                           'position_ref': np.zeros((3, 0)), 'position': np.zeros((3, 0)),
                           'orientation_ref': np.zeros((3, 0)), 'orientation': np.zeros((3, 0))}
        self.pose = {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0}
        self.velocity = {'x': 0, 'y': 0, 'z': 0}

    def takeoff(self, altitude):
        while not self.sim_vehicle.is_armable:
            time.sleep(0.1)
        print 'arming motors!'
        self.sim_vehicle.mode = VehicleMode('GUIDED')
        self.sim_vehicle.armed = True
        while not self.sim_vehicle.armed:
            time.sleep(0.1)
        print 'taking off!'
        self.sim_vehicle.simple_takeoff(altitude)
        while np.linalg.norm(self.sim_vehicle.location.global_relative_frame.alt - altitude) > 5e-1:
            time.sleep(0.1)
        print 'reached target altitude'

    def set_yaw(self, yaw):
        print 'setting yaw...'
        msg = self.sim_vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            yaw,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to self.sim_vehicle
        self.sim_vehicle.send_mavlink(msg)
        while np.linalg.norm(self.sim_vehicle.attitude.yaw - yaw) > 1e-1:
            self.set_velocity(0, 0, 0)
            time.sleep(0.1)
        print 'yaw set'

    def set_velocity(self, vx, vy, vz):
        # print '[vx, vy] = [%f, %f]' % (vx, vy)
        msg = self.sim_vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            # mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # local frame
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.sim_vehicle.send_mavlink(msg)

    def set_target(self, x, y, z, vx, vy, vz):
        # print '[vx, vy] = [%f, %f]' % (vx, vy)
        msg = self.sim_vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            # mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # local frame
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000000, # type_mask (position and speeds enabled)
            x, y, z, # x, y, z positions (not used)
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.sim_vehicle.send_mavlink(msg)

    def store_position(self, self2, name, position):
        self.pose['x'] = position.north
        self.pose['y'] = position.east
        self.pose['z'] = position.down

    def store_orientation(self, self2, name, orientation):
        self.pose['roll'] = orientation.roll
        self.pose['pitch'] = orientation.pitch
        self.pose['yaw'] = orientation.yaw

    def store_velocity(self, self2, name, velocity):
        self.velocity['x'] = velocity[0]
        self.velocity['y'] = velocity[1]
        self.velocity['z'] = -velocity[2]

    def update(self):
        # get current state
        current_position = np.r_[self.pose['x'], self.pose['y'], self.pose['z']]
        current_orientation = np.r_[self.pose['roll'], self.pose['pitch'], self.pose['yaw']]
        current_velocity = np.r_[self.velocity['x'], self.velocity['y'], self.velocity['z']]

        current_state = np.r_[self.pose['x'], -self.pose['y'], -self.pose['z'],
                              self.velocity['x'], -self.velocity['y'], -self.velocity['z'],
                              self.pose['roll'], -self.pose['pitch']]


        # update motion planner when it's time
        if self.init:
            if not self.new_trajectories:
                return
            self.index = int(self.update_time/self.sample_time)
            self.time += self.update_time
            self.init = False
        if self.index >= int(self.update_time/self.sample_time):
            if self.new_trajectories:
                # load fresh trajectories
                self.load_trajectories()
                self.new_trajectories = False
                self.index = 0
                # trigger motion planner
                self.mp.trigger(self.time, current_state)
            else:
                print 'overtime!'
        # send sample to low-level quadrotor controller
        self.set_target(self.state_traj[0, self.index], self.state_traj[1, self.index], self.state_traj[2, self.index],
                        self.state_traj[3, self.index], self.state_traj[4, self.index], self.state_traj[5, self.index])
        # save
        self.simulation['time'] = np.c_[self.simulation['time'], self.time]
        self.simulation['position_ref'] = np.c_[self.simulation['position_ref'], self.state_traj[:3, self.index]]
        self.simulation['velocity_ref'] = np.c_[self.simulation['velocity_ref'], self.state_traj[3:6, self.index]]
        self.simulation['orientation_ref'] = np.c_[self.simulation['orientation_ref'], np.r_[self.state_traj[6:, self.index], 0.]]
        self.simulation['position'] = np.c_[self.simulation['position'], np.c_[current_position]]
        self.simulation['velocity'] = np.c_[self.simulation['velocity'], np.c_[current_velocity]]
        self.simulation['orientation'] = np.c_[self.simulation['orientation'], np.c_[current_orientation]]
        # plot
        if self.index == 0:
            self.update_plots()
        self.index += 1
        self.time = np.round(self.time + self.sample_time, 2)

    def load_trajectories(self):
        self.input_traj = self.input_traj_strg[:]
        self.state_traj = self.state_traj_strg[:]

    def store_trajectories(self, input_traj, state_traj):
        self.input_traj_strg = input_traj
        self.state_traj_strg = np.c_[state_traj[0, :], -state_traj[1, :], -state_traj[2, :],
                                     state_traj[3, :], -state_traj[4, :], -state_traj[5, :],
                                     state_traj[6, :], -state_traj[7, :]].T
        self.new_trajectories = True

    def proceed(self):
        if self.simulation['position'].shape[1] == 0:
            return True
        return not (np.linalg.norm(self.simulation['position'][:, -1] - np.r_[self.goal[0], self.goal[1], -self.goal[2]]) < 5e-1
            and np.linalg.norm(self.simulation['orientation'][:, -1]) < 5e-1
            and np.linalg.norm(self.simulation['velocity_ref'][:, -1]) < 1e-2)

    def set_goal(self, goal):
        self.goal = goal
        self.time = 0.
        # get current state
        current_state = np.r_[self.pose['x'], -self.pose['y'], -self.pose['z'],
                              self.velocity['x'], -self.velocity['y'], -self.velocity['z'],
                              self.pose['roll'], -self.pose['pitch']]
        self.new_trajectories = False
        self.mp.set_goal(goal)
        self.mp.trigger(self.time, current_state)
        self.init = True

    def start(self, goal):
        proceed = True
        self.init_plots()
        self.takeoff(10)
        self.set_yaw(0)
        print 'controller started!'
        self.set_goal(goal)
        # self.index = 0
        while (proceed):
            t0 = time.time()
            self.update()
            proceed = self.proceed()
            t1 = time.time()
            dt = self.sample_time - (t1 - t0)
            if dt < 0:
                print 'control loop time exceeds sample time: %f ' % (t1-t0)
            else:
                time.sleep(dt)

        # send zeros?
        self.set_velocity(0, 0, 0)
        print 'target reached!'
        self.mp.proceed.clear()
        self.mp.mp_trigger.set()
        self.update_plots()

    def configure(self):
        print 'configure controller'
        self.mp.configure(self.sample_time, self.update_time)

    def init_plots(self):
        self.figs = []
        self.axes = {}
        self.figs.append(plt.figure())
        self.axes['pose'] = []
        for k in range(6):
            self.axes['pose'].append(self.figs[-1].add_subplot(6, 1, k+1))
            self.axes['pose'][k].plot([], [], color=mix_with_white([17./255., 110./255., 138./255.]))
            self.axes['pose'][k].plot([], [], color=[17./255., 110./255., 138./255.])
        self.figs.append(plt.figure())
        self.axes['velocity'] = []
        for k in range(3):
            self.axes['velocity'].append(self.figs[-1].add_subplot(3, 1, k+1))
            self.axes['velocity'][k].plot([], [], color=mix_with_white([17./255., 110./255., 138./255.]))
            self.axes['velocity'][k].plot([], [], color=[17./255., 110./255., 138./255.])

    def update_plots(self):
        for k in range(3):
            self.axes['pose'][k].lines[0].set_data(self.simulation['time'], self.simulation['position_ref'][k, :])
            self.axes['pose'][k].lines[1].set_data(self.simulation['time'], self.simulation['position'][k, :])
            self.axes['pose'][k].relim()
            self.axes['pose'][k].autoscale_view(True, True, True)
        for k in range(3):
            self.axes['pose'][3+k].lines[0].set_data(self.simulation['time'], self.simulation['orientation_ref'][k, :])
            self.axes['pose'][3+k].lines[1].set_data(self.simulation['time'], self.simulation['orientation'][k, :])
            self.axes['pose'][3+k].relim()
            self.axes['pose'][3+k].autoscale_view(True, True, True)
        for k in range(3):
            self.axes['velocity'][k].lines[0].set_data(self.simulation['time'], self.simulation['velocity_ref'][k, :])
            self.axes['velocity'][k].lines[1].set_data(self.simulation['time'], self.simulation['velocity'][k, :])
            self.axes['velocity'][k].relim()
            self.axes['velocity'][k].autoscale_view(True, True, True)
        for fig in self.figs:
            fig.canvas.draw()

if __name__ == '__main__':
    sample_time = 0.1
    update_time = 1.
    controller = Controller(sample_time, update_time)
    time.sleep(0.1)
    controller.configure()
    controller.start(np.r_[0., 0., 20.])
    plt.show(block=True)
