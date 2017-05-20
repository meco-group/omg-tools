obstacles = range(1,10)
import sys, os
import numpy as np

sys.path.insert(0, os.getcwd() + "/..")
from omgtools import *
# av_update_times = []
# max_update_times = []
# buildtimes = []
# median_update_times = []
# objectives = []
#
# for obs in obstacles:
#
#     knot = 9.
#     # create vehicle
#     vehicle = Holonomic(shapes=Circle(0.2))
#     vehicle.define_knots(knot_intervals=knot)  # adapt amount of knot intervals
#     vehicle.set_initial_conditions([2., 5.])  # input orientation in deg
#     #vehicle.set_terminal_conditions([3., 3., 90.])
#     vehicle.set_terminal_conditions([8., 5.]) #eerste waarde naar rechts en tweede omhoog
#     # create trailer
#     trailer = TrailerHolonomic(lead_veh=vehicle,  shapes=Rectangle(0.3,0.2), l_hitch = 0.6)  # ldie limiet boeit niet, wordt niet in rekening gebracht.
#     # Note: the knot intervals of lead_veh and trailer should be the same
#     trailer.define_knots(knot_intervals=knot)  # adapt amount of knot intervals
#     trailer.set_initial_conditions([0.])  # input orientation in deg
#     #trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking
#
#     # create environment
#     environment = Environment(room={'shape': Square(10.), 'position': [5.,5.]})
#     rectangle = Rectangle(width=.2, height=4.)
#     for i in range(obs):
#         environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=rectangle))
#
#     # trajectory = {'position': {'time': [3.],
#     #                            'values': [[-0.0 , -1.5]]}}
#     # # Here we defined the time-axis and the corresponding values for velocity.
#     # # Note that these values should be interpreted relatively: eg. at time 3, we
#     # # _add_ an extra velocity of [-0.15, 0.0].
#     # # You could also change position and acceleration in a similar way.
#
#     # # trajectories are put in a simulation dictionary
#     # simulation = {'trajectories': trajectory}
#     # # here we can give a different simulation model, by providing an A and B matrix
#     # # simulation['model'] = {'A': A, 'B': B}
#     # # The behaviour of an obstacle is expressed by x_dot = A*x + B*u
#     # # The state x is composed of [position, velocity, acceleration]
#     # # The input can be provided via a trajectory (is by default 0)
#     # # simulation['trajectories']['input'] = {'time': time, 'values': values}
#     # # simulation['trajectories']['input'] = {'time': time, 'values': values}
#     # # Here the values should be interpreted as absolute.
#
#     # environment.add_obstacle(Obstacle({'position': [6., 8.]}, shape=rectangle,
#     #                                   simulation=simulation))
#
#
#     # environment.add_obstacle(Obstacle({'position': [2., 1.5]}, shape=rectangle))
#     # create a point-to-point problem
#     problem = Point2point(trailer, environment, freeT=True)  # pass trailer to problem
#     # todo: isn't there are a cleaner way?
#     problem.father.add(vehicle)  # add vehicle to optifather, such that it knows the trailer variables
#     # extra solver settings which may improve performance https://www.coin-or.org/Ipopt/documentation/node53.html#SECTION0001113010000000000000
#     problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
#     #problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma27'}}})
#     #problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
#     buildtime =problem.init()
#     problem.plot('scene')
#
#
#
#     # create simulator
#     simulator = Simulator(problem, update_time = 0.5)
#
#     # run it!
#     simulator.run()
#     max_update_time = max(problem.update_times) * 1000.
#     av_update_time = (sum(problem.update_times) * 1000. /len(problem.update_times))
#     median_update_time = np.median(problem.update_times) * 1000.
#     obj = problem.compute_objective()
#
#     max_update_times.append(max_update_time)
#     av_update_times.append(av_update_time)
#     median_update_times.append(median_update_time)
#     buildtimes.append(buildtime)
#     objectives.append(obj)
#
#     #sys.modules[__name__].__dict__.clear()
#
# data = np.c_[obstacles,buildtimes,  max_update_times, av_update_times, median_update_times, objectives]
# np.savetxt('obstacles_ma57.csv', data, delimiter=',')
#

av_update_times = []
max_update_times = []
buildtimes = []
median_update_times = []
objectives = []

for obs in obstacles:

    knot = 9.
    # create vehicle
    vehicle = Holonomic(shapes=Circle(0.2))
    vehicle.define_knots(knot_intervals=knot)  # adapt amount of knot intervals
    vehicle.set_initial_conditions([2., 5.])  # input orientation in deg
    #vehicle.set_terminal_conditions([3., 3., 90.])
    vehicle.set_terminal_conditions([8., 5.]) #eerste waarde naar rechts en tweede omhoog
    # create trailer
    trailer = TrailerHolonomic(lead_veh=vehicle,  shapes=Rectangle(0.3,0.2), l_hitch = 0.6)  # ldie limiet boeit niet, wordt niet in rekening gebracht.
    # Note: the knot intervals of lead_veh and trailer should be the same
    trailer.define_knots(knot_intervals=knot)  # adapt amount of knot intervals
    trailer.set_initial_conditions([0.])  # input orientation in deg
    #trailer.set_terminal_conditions([0.])  # this depends on the application e.g. driving vs parking

    # create environment
    environment = Environment(room={'shape': Square(10.), 'position': [5.,5.]})
    rectangle = Rectangle(width=.2, height=4.)
    for i in range(obs):
        environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=rectangle))

    # trajectory = {'position': {'time': [3.],
    #                            'values': [[-0.0 , -1.5]]}}
    # # Here we defined the time-axis and the corresponding values for velocity.
    # # Note that these values should be interpreted relatively: eg. at time 3, we
    # # _add_ an extra velocity of [-0.15, 0.0].
    # # You could also change position and acceleration in a similar way.

    # # trajectories are put in a simulation dictionary
    # simulation = {'trajectories': trajectory}
    # # here we can give a different simulation model, by providing an A and B matrix
    # # simulation['model'] = {'A': A, 'B': B}
    # # The behaviour of an obstacle is expressed by x_dot = A*x + B*u
    # # The state x is composed of [position, velocity, acceleration]
    # # The input can be provided via a trajectory (is by default 0)
    # # simulation['trajectories']['input'] = {'time': time, 'values': values}
    # # simulation['trajectories']['input'] = {'time': time, 'values': values}
    # # Here the values should be interpreted as absolute.

    # environment.add_obstacle(Obstacle({'position': [6., 8.]}, shape=rectangle,
    #                                   simulation=simulation))


    # environment.add_obstacle(Obstacle({'position': [2., 1.5]}, shape=rectangle))
    # create a point-to-point problem
    problem = Point2point(trailer, environment, freeT=True)  # pass trailer to problem
    # todo: isn't there are a cleaner way?
    problem.father.add(vehicle)  # add vehicle to optifather, such that it knows the trailer variables
    # extra solver settings which may improve performance https://www.coin-or.org/Ipopt/documentation/node53.html#SECTION0001113010000000000000
    problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57','ipopt.hessian_approximation': 'limited-memory'}}})
    #problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma27'}}})
    #problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
    buildtime =problem.init()
    problem.plot('scene')



    # create simulator
    simulator = Simulator(problem, update_time = 0.5)

    # run it!
    simulator.run()
    max_update_time = max(problem.update_times) * 1000.
    av_update_time = (sum(problem.update_times) * 1000. /len(problem.update_times))
    median_update_time = np.median(problem.update_times) * 1000.
    obj = problem.compute_objective()

    max_update_times.append(max_update_time)
    av_update_times.append(av_update_time)
    median_update_times.append(median_update_time)
    buildtimes.append(buildtime)
    objectives.append(obj)

    #sys.modules[__name__].__dict__.clear()

data = np.c_[obstacles,buildtimes,  max_update_times, av_update_times, median_update_times, objectives]
np.savetxt('obstacles_ma57_approx.csv', data, delimiter=',')
