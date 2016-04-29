# Implementation of pick and place case of LVD
# questions: ruben.vanparys@kuleuven.be

import sys
import os
sys.path.insert(0, os.getcwd()+'/../')
import matplotlib
from omgtools import *
from omgtools.vehicles.lvd_machine import LVD

save = False

# create plate
# limits on position mid-point plate
# bounds = {'smin': [None, -0.95, 0.405], 'smax': [None, -0.55, 1.5]}
bounds = {'smin': [1.4, -0.85, 0.405], 'smax': [8.6, -0.1, 1.155]}
# shutdown room constraints (we use constraints on position)
options = {'room_constraints': False}
# create plate object
shape = Plate(Rectangle(3., 1.5), 0.01)
plate = LVD(shape, options=options, bounds=bounds)
plate.define_knots(knot_intervals=11)

plate.set_initial_conditions([8.5, -0.75, 0.405])
plate.set_terminal_conditions([1.5, -0.55, 1.015])

# create environment
environment = Environment(
    room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})

# obstacles for collision avoidance
cuboids = False  # False -> a bit faster
if cuboids:
    # table
    shape1 = Cuboid(3.350, 1.7, 1.0)
    shape1.radius = 0.005  # for safety
    position1 = [1.675, -0.65, 0.5]
    # cover + frame
    shape2 = Cuboid(0.58, 3.0, 1.0)
    position2 = [0.29 + 0.1, 0.0, 1.6] # +0.1 for safety
    # measure
    shape3 = Cuboid(0.15, 0.15, 1.15)
    position3 = [6.875, -1.325, 0.575]
else:
    # table
    vertices1 = np.c_[[1.675, -0.85, -0.5], [1.675, 0.85, -0.5],
                      [1.675, -0.85,  0.5], [1.675, 0.85,  0.5],
                      [-1.675, -0.85,  0.5], [-1.675, 0.85,  0.5]]
    shape1 = Polyhedron3D(vertices1, 0.005)
    position1 = [1.675, -0.65, 0.5]
    # cover + frame
    vertices2 = np.c_[[0.29, 1.5, -0.5], [0.29, 1.5, 0.5],
                      [0.29, -1.5, -0.5], [0.29, -1.5, 0.5]]
    shape2 = Polyhedron3D(vertices2, 0.001)
    position2 = [0.29 + 0.1, 0.0, 1.6] # +0.1 for safety
    # measure
    vertices3 = np.c_[[0.075, -0.075, -0.575], [0.075, 0.075, -0.575],
                      [0.075, -0.075, 0.575], [0.075, 0.075, 0.575],
                      [-0.075, -0.075, 0.575], [-0.075, 0.075, 0.575]]
    shape3 = Polyhedron3D(vertices3, 0.001)
    position3 = [6.875, -1.325, 0.575]

obstacle1 = Obstacle({'position': position1}, shape1, {}, {'draw': False})
obstacle2 = Obstacle({'position': position2}, shape2, {}, {'draw': False})
obstacle3 = Obstacle({'position': position3}, shape3, {}, {'draw': False})

environment.add_obstacle([obstacle1, obstacle2, obstacle3])

# obstacles just for drawing
frame = Obstacle(
    {'position': [0.165, 0.0, 1.85]}, Cuboid(0.33, 3.0, 0.9), {}, {'avoid': False})
table = Obstacle(
    {'position': [1.5, 0.0, 0.5]}, Cuboid(3., 3., 1.), {}, {'avoid': False})
cover = Obstacle(
    {'position': [0.455, 0.7, 1.475]}, Cuboid(0.25, 1., 0.75), {}, {'avoid': False})
beam = Obstacle(
    {'position': [1.35, -1.95, 1.2]}, Cuboid(2.7, 0.5, 0.4), {}, {'avoid': False})
leg = Obstacle(
    {'position': [3.2, -0.9, 0.35]}, Cuboid(0.3, 1.6, 0.7), {}, {'avoid': False})
pilar = Obstacle(
    {'position': [3.2, -1.925, 0.75]}, Cuboid(0.3, 0.45, 1.5), {}, {'avoid': False})
measure = Obstacle(
    {'position': [6.875, -1.325, 0.575]}, Cuboid(0.15, 0.15, 1.15), {}, {'avoid': False})
environment.add_obstacle([frame, table, cover, beam, leg, pilar, measure])

# create a point-to-point problem
problem = Point2point(plate, environment, freeT=True)
problem.set_options(
    {'solver': {'ipopt.linear_solver': 'ma57'}, 'horizon_time': 10.})
problem.init()

# create simulator
simulator = Simulator(problem)

# run it!
trajectories = simulator.run_once()

# show results
# plate.plot('state', labels=['x (m)', 'y (m)', 'z (m)'])
# plate.plot('velocity', labels=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'])
# plate.plot(
#     'acceleration', labels=['ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'])
# plate.plot(
#     'jerk', labels=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'])
# problem.plot_movie(
#     'scene', number_of_frames=40, repeat=False, view=[60, 45])
# problem.plot_movie(
#     'scene', number_of_frames=40, repeat=False, view=[0, 90])
# problem.plot_movie(
#     'scene', number_of_frames=40, repeat=True, view=[90, 0])

# save results
if save:
    plate.save_plot('state', 'position', labels=[
        'x (m)', 'y (m)', 'z (m)'], figurewidth='15cm', figureheight='4cm')
    plate.save_plot('velocity', 'velocity', labels=[
        'dx (m/s)', 'dy (m/s)', 'dz (m/s)'], figurewidth='15cm', figureheight='4cm')
    plate.save_plot('acceleration', 'acceleration', labels=[
        'ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'], figurewidth='15cm', figureheight='4cm')
    plate.save_plot('jerk', 'jerk', labels=[
        'dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'], figurewidth='15cm', figureheight='4cm')

    problem.save_movie('scene', 'scene1', number_of_frames=40, view=[30, 60], axis=False)
    problem.save_movie('scene', 'scene2', number_of_frames=40, view=[0, 90], axis=False)
    problem.save_movie('scene', 'scene3', number_of_frames=40, view=[90, 0], axis=False)
matplotlib.pyplot.show(block=True)
