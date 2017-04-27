import sys, os


sys.path.insert(0, os.getcwd()+"/..")
from omgtools import *
import matplotlib.pyplot as plt
import csv
import numpy as np

vehicle = Holonomic(shapes=Square(0.4))
number_knot_intervals = 9.
vehicle.define_knots(knot_intervals=number_knot_intervals)  # adapt amount of knot intervals
vehicle.set_initial_conditions([0.5, 0.5])
vehicle.set_terminal_conditions([4.5, 4.5])

# create trailer
trailer = TrailerHolonomic(lead_veh=vehicle,  shapes=Square(0.3), l_hitch = 0.6,
                  bounds={'tmax': 45., 'tmin': -45.})  # ldie limiet boeit niet, wordt niet in rekening gebracht.
# Note: the knot intervals of lead_veh and trailer should be the same
trailer.define_knots(knot_intervals=number_knot_intervals)  # adapt amount of knot intervals
trailer.set_initial_conditions([0.])  # input orientation in deg
trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking
# set up environment
environment = Environment(room={'shape': Rectangle(7., 7.), 'position': [2.5, 2.5]})

# set up problem
problem = Point2point(trailer, environment, freeT=True)
problem.father.add(vehicle)
problem.set_options({'horizon_time': 6})
problem.init()

# simulate
# vehicle.plot('input')
simulator = Simulator(problem)
problem.plot('scene')
trailer.plot('input', knots=True, labels=['v_x (m/s)','v_y (m/s)'])
trailer.plot('state', knots=True, labels=['x_tr (m)', 'y_tr (m)', 'theta_tr (rad)', 'x_veh (m)', 'y_veh (m)', 'theta_veh (rad)'])
trajectories1 = simulator.run_once()


# pos_traj1 = trajectories1['vehicle0']['state'][:, :]
# vel_traj1 = trajectories1['vehicle0']['input'][:, :]
# #theta_traj1 = np.linspace(0, np.pi, pos_traj1.shape[1])


vehicle.set_terminal_conditions([0.5, 0.5])
trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking

problem.reinitialize()

trajectories2 = simulator.run_once()
problem.save_movie('scene', format='gif', name='lead_off_rechtdoor5_ma57', number_of_frames=100, movie_time=5, axis=False)

# pos_traj2 = trajectories2['vehicle0']['state'][:, :]
# vel_traj2 = trajectories2['vehicle0']['input'][:, :]
# #theta_traj2 = np.linspace(np.pi, 0, pos_traj2.shape[1])
#
#
# pos_traj = np.c_[pos_traj1, pos_traj2]
# vel_traj = np.c_[vel_traj1, vel_traj2]
# #theta_traj = np.r_[theta_traj1, theta_traj2]
#
#
# pos_traj = np.c_[np.c_[pos_traj[:, 0]]*np.ones(500), pos_traj]
# vel_traj = np.c_[np.c_[vel_traj[:, 0]]*np.ones(500), vel_traj]
# #theta_traj = np.r_[theta_traj[0]*np.ones(500), theta_traj]
#
# n = pos_traj.shape[1]
#
# # save trajectory in csv-format
# # data = np.c_[pos_traj.T, np.zeros(n), vel_traj.T, np.zeros(n)]
# #data = np.c_[pos_traj.T, theta_traj.T, vel_traj.T, np.zeros(n)]
# data = np.c_[pos_traj.T, vel_traj.T, np.zeros(n)]
#
# # data = np.c_[pos_traj, trajectories1['vehicle0']['state'][0, 1]*np.ones(n), np.zeros(n), vel_traj, np.zeros(n), np.zeros(n)]
# # data = np.c_[np.zeros(n), pos_traj, np.zeros(n), np.zeros(n), vel_traj, np.zeros(n)]
# # data = np.c_[pos_traj, pos_traj, np.zeros(n), vel_traj, vel_traj, np.zeros(n)]
# # data = np.c_[np.zeros(n), np.zeros(n), pos_traj, np.zeros(n), np.zeros(n), vel_traj]
# # np.savetxt('trajectories_xy.csv', data, delimiter=',')
# np.savetxt('trajectories_xyt.csv', data, delimiter=',')
#
# plt.figure()
# plt.hold(True)
# #plt.plot(data[:, 0])
# #plt.plot(data[:, 1])
# plt.plot(data[:, 2])
#
# plt.figure()
# plt.hold(True)
# #plt.plot(data[:, 3])
# #plt.plot(data[:, 4])
# plt.plot(data[:, 5])
#
# plt.show(block=True)
