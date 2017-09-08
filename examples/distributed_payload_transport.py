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

from omgtools import *
import os
import csv

#Load Vehicle Parameters
m0 = 2.0
m1 = 2.0
m2 = 2.0
m3 = 2.0
K1 = 60.0
K2 = 60.0
K3 = 60.0
C1 = 20.
C2 = 20.
C3 = 20.
l_free = 0.42

#Initial Positions
offset = [0.,0.]
#offset = [0.0,0.0]
d0_i = np.add([1.0,1.0],offset)
d1_i = np.add([0.8,0.8],offset)
d2_i = np.add([0.8,1.2],offset)
d3_i = np.add([1.4,1.0],offset)

#Target position of payload
target = [4.0,2.5]

#Terminal Positions
d0_f = target
d1_f = np.subtract(np.add(target,d1_i),d0_i)
d2_f = np.subtract(np.add(target,d2_i),d0_i)
d3_f = np.subtract(np.add(target,d3_i),d0_i)

#Tuning params for weights
dynamics_approx = 1.0
terminal_condn = 1.
init_velocity = 1.
rho = 0.5
accn_penalty = 0.
angle_constraints = 1
tuning_weights = [dynamics_approx, terminal_condn, init_velocity,angle_constraints,accn_penalty]

# create fleet
N = 3
vehicles = []
vehicles.append(HolonomicF(m1,m2,m3,K1,K2,K3,C1,C2,C3,m0,l_free,tuning_weights))
vehicles.append(HolonomicF(m2,m3,m1,K2,K3,K1,C2,C3,C1,m0,l_free,tuning_weights))
vehicles.append(HolonomicF(m3,m1,m2,K3,K1,K2,C3,C1,C2,m0,l_free,tuning_weights))

vehicles[0].options['stop_tol'] = 0.001
vehicles[1].options['stop_tol'] = 0.001
vehicles[2].options['stop_tol'] = 0.001

fleet = Fleet(vehicles)

init_states     = [[d0_i[0], 0., d1_i[0], 0., d2_i[0], 0., d3_i[0], 0., d0_i[1], 0., d1_i[1], 0., d2_i[1], 0., d3_i[1], 0.],
                        [d0_i[0], 0., d2_i[0], 0., d3_i[0], 0., d1_i[0], 0., d0_i[1], 0., d2_i[1], 0., d3_i[1], 0., d1_i[1], 0.],
                        [d0_i[0], 0., d3_i[0], 0., d1_i[0], 0., d2_i[0], 0., d0_i[1], 0., d3_i[1], 0., d1_i[1], 0., d2_i[1], 0.]]


#Final Position Order -[x11,y11],[x10,y10]
terminal_positions = [[d1_f[0],d1_f[1],d0_f[0],d0_f[1]],
                               [d2_f[0],d2_f[1],d0_f[0],d0_f[1]],
                               [d3_f[0],d3_f[1],d0_f[0],d0_f[1]]]

fleet.set_initial_conditions(init_states)
fleet.set_terminal_conditions(terminal_positions)

# create environment
environment = Environment(room={'shape': Rectangle(5.,3.),'position':[2.5,1.5]})
environment.add_obstacle(Obstacle({'position': [3.0, 2.05]}, shape=Square(0.2)))
#environment.add_obstacle(Obstacle({'position': [4.0, 2.0]}, shape=Square(0.2)))
#environment.add_obstacle(Obstacle({'position': [5.0, 4.0]}, shape=Square(0.8)))

# create a formation point-to-point problem
options = {'rho': rho, 'horizon_time': 10.}
casadi_path = os.path.join(os.getenv('HOME'), 'casadi-py27-np1.9.1-v3.0.0')
options['directory'] = os.path.join(os.getcwd(), 'export/')
# path to object files of your exported optimization problem
options['casadiobj'] = os.path.join(options['directory'], 'bin/')
# your casadi include path
options['casadiinc'] = os.path.join(casadi_path, 'include/')
# your casadi library path
options['casadilib'] = os.path.join(casadi_path, 'casadi/')
problem = PayloadTransport(fleet, environment, options=options)
problem.options['separate_build']= True
#problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem.init()
#problem.export(options)

# create simulator
simulator = Simulator(problem, sample_time=0.01, update_time = 0.5)
#vehicles[0].plot('vehicle_j_trajs', knots=True, labels=['x_12','y_12'])
#vehicles[1].plot('local_positions', knots=True, labels=['x_2','y_2'])
fleet.plot('payload_pos', knots=True, labels=['x_0','y_0'])
fleet.plot('velocities', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])
fleet.plot('accelerations', knots=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)'])
fleet.plot('acceleration_residue', knots=True, labels=['acceleration_x_residue (m/s^2)','acceleration_y_residue (m/s^2)'])
problem.plot('scene')
problem.plot('residuals')
#fleet.plot('neighbor_splines_x', knots=True, labels=['x_12', 'x_13'])
#fleet.plot('neighbor_splines_y', knots=True, labels=['y_12', 'y_13'])

# run it!
trajectories, signals = simulator.run()


problem.plot_movie('scene', repeat=True, number_of_frames=150)
#problem.save_movie('scene',format = 'gif', number_of_frames=50)

#fleet.save_movie('velocities',format = 'gif',number_of_frames=150, knots=True, prediction=True, axis=True, labels=['v_x (m/s)', 'v_y (m/s)'])


if 1==0:
    import matplotlib.pyplot as plt
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(np.transpose(signals['vehicle0']['time'][0]),signals['vehicle0']['velocities'][0])
    plt.plot(np.transpose(signals['vehicle1']['time'][0]),signals['vehicle1']['velocities'][0])
    plt.plot(np.transpose(signals['vehicle2']['time'][0]),signals['vehicle2']['velocities'][0])
    plt.xlabel(r'time [s]')
    plt.ylabel(r'v_x [m/s]')
    plt.ylim(0,0.5)
    plt.subplot(2,1,2)
    plt.plot(np.transpose(signals['vehicle0']['time'][0]),signals['vehicle0']['velocities'][1])
    plt.plot(np.transpose(signals['vehicle1']['time'][0]),signals['vehicle1']['velocities'][1])
    plt.plot(np.transpose(signals['vehicle2']['time'][0]),signals['vehicle2']['velocities'][1])
    plt.xlabel(r'time [s]')
    plt.ylabel(r'v_y [m/s]')
    plt.ylim(0,0.5)
    plt.savefig('1vsothers.svg')
    plt.close()
