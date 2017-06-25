import sys, os


sys.path.insert(0, os.getcwd()+"/..")
from omgtools import *
import matplotlib.pyplot as plt
import csv
import numpy as np

options = {}
options['safety_distance'] = 0.;
vehicle = HolonomicOrient(shapes=Rectangle(0.4, 0.32), options=options, bounds={'vmax': 0.1,'vmin':-0.1, 'wmax': 60., 'wmin': -60.})  # in deg
number_knot_intervals = 9.
vehicle.define_knots(knot_intervals=number_knot_intervals)  # adapt amount of knot intervals
vehicle.set_initial_conditions([0.5, .5, 0.])
vehicle.set_terminal_conditions([4., .5, 0.])

# create trailer
trailer = TrailerJolandHolonomic(lead_veh=vehicle,  shapes=Rectangle(0.3,0.2), l_hitch = 0.215, l_hitch1 = 0.2075,
                  bounds={'tmax': 40., 'tmin': -40.}, options=options)
# Note: the knot intervals of lead_veh and trailer should be the same
trailer.define_knots(knot_intervals=number_knot_intervals)  # adapt amount of knot intervals
trailer.set_initial_conditions([0.])  # input orientation in deg
trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking
# set up environment
environment = Environment(room={'shape': Rectangle(5., 3.), 'position': [2.5, 1.5]})
rectangle = Rectangle(width=.2, height=3.)

environment.add_obstacle(Obstacle({'position': [2., 0.]}, shape=rectangle))

# set up problem
problem = Point2point(trailer, environment, freeT=True)
problem.father.add(vehicle)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57','ipopt.hessian_approximation': 'limited-memory'}}})
problem.init()

# simulate
# vehicle.plot('input')
simulator = Simulator(problem)
problem.plot('scene')
trailer.plot('input', knots=True, labels=['v_x (m/s)','v_y (m/s)', 'theta_veh (rad)'])
trailer.plot('state', knots=True, labels=['x_tr (m)', 'y_tr (m)', 'theta_tr (rad)', 'x_veh (m)', 'y_veh (m)', 'theta_veh (rad)'])
simulator.run_once()


# pos_veh = trajectories1['vehicle1']['state'][3:, :]
# vel_veh = trajectories1['vehicle1']['input'][:, :]
# theta_tr = trajectories1['vehicle1']['state'][2, :]
# # #theta_traj1 = np.linspace(0, np.pi, pos_traj1.shape[1])
#
#
# # vehicle.set_terminal_conditions([0.5, 0.5, 0.])
# # trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking
#
# # problem.reinitialize()
#
# # trajectories2 = simulator.run_once()
problem.save_movie('scene', format='gif', name='smooth_trajectory', number_of_frames=100, movie_time=5, axis=False)
#
# # pos_traj2 = trajectories2['vehicle0']['state'][:, :]
# # vel_traj2 = trajectories2['vehicle0']['input'][:, :]
# # #theta_traj2 = np.linspace(np.pi, 0, pos_traj2.shape[1])
# #
# #
# # pos_traj = np.c_[pos_traj1, pos_traj2]
# # vel_traj = np.c_[vel_traj1, vel_traj2]
# # #theta_traj = np.r_[theta_traj1, theta_traj2]
# #
# #
#
# pos_veh = np.c_[np.c_[pos_veh[:, 0]]*np.ones(500), pos_veh]
# vel_veh = np.c_[np.c_[vel_veh[:, 0]]*np.ones(500), vel_veh]
# theta_tr=np.r_[theta_tr[0]*np.ones(500), theta_tr]
#
# # #theta_traj = np.r_[theta_traj[0]*np.ones(500), theta_traj]
# #
# n = pos_veh.shape[1]
# #
# # # save trajectory in csv-format
# # # data = np.c_[pos_traj.T, np.zeros(n), vel_traj.T, np.zeros(n)]
# data = np.c_[pos_veh.T, theta_tr.T, vel_veh.T, np.zeros(n)]
# #data = np.c_[pos_veh.T, vel_veh.T, np.zeros(n).T]
# #
# # # data = np.c_[pos_traj, trajectories1['vehicle0']['state'][0, 1]*np.ones(n), np.zeros(n), vel_traj, np.zeros(n), np.zeros(n)]
# # # data = np.c_[np.zeros(n), pos_traj, np.zeros(n), np.zeros(n), vel_traj, np.zeros(n)]
# # # data = np.c_[pos_traj, pos_traj, np.zeros(n), vel_traj, vel_traj, np.zeros(n)]
# # # data = np.c_[np.zeros(n), np.zeros(n), pos_traj, np.zeros(n), np.zeros(n), vel_traj]
# # # np.savetxt('trajectories_xy.csv', data, delimiter=',')
# np.savetxt('trajectories_trailer_obstakel.csv', data, delimiter=',')
# #
# # plt.figure()
# # plt.hold(True)
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
