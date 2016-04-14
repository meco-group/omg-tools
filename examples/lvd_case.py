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
bounds = {'smin': [None, -0.95, 0.405], 'smax': [None, -0.55, 1.5]}
# bounds = {'smin': [1.4, -0.85, 0.405], 'smax': [8.6, -0.55, 1.155]}
# shutdown room constraints (we use constraints on position)
options = {'room_constraints': False}
# shape of plate
shape = Plate(Rectangle(3., 1.5), 0.01)
plate = LVD(shape, options=options, bounds=bounds)
# plate.define_knots(knot_intervals=11)

knot_intervals = 10
s = np.linspace(0., 0.5*np.pi, knot_intervals+1)
# s = np.linspace(0., 1., knot_intervals+1)
knots = np.r_[np.zeros(plate.degree), 1.-np.cos(s), np.ones(plate.degree)]
knots = np.r_[np.zeros(plate.degree), s, np.ones(plate.degree)]
# knots = np.insert(knots, plate.degree+2, 0.5*(knots[plate.degree+1]+knots[plate.degree+2]))
for k in range(plate.degree+2, plate.degree+4):
    knots = np.insert(knots, k, 0.5*(knots[k-1]+knots[k]))

# strange behaviour???
plate.define_knots(knots = knots)


plate.set_initial_conditions([8.5, -0.75, 0.405])
plate.set_terminal_conditions([1.5, -0.55, 1.015])

# create environment
environment = Environment(
    room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})

# obstacles for collision avoidance
cuboids = False  # False -> a bit faster
if cuboids:
    shape1 = Cuboid(6.950, 1.7, 1.0)
    shape1.radius = 0.005  # for safety
    shape2 = Cuboid(0.85, 3.0, 1.0)
else:
    vertices1 = np.c_[[3.475, -0.85, -0.5], [3.475, 0.85, -0.5],
                      [3.475, -0.85,  0.5], [3.475, 0.85,  0.5],
                      [-3.475, -0.85,  0.5], [-3.475, 0.85,  0.5]]
    shape1 = Polyhedron3D(vertices1, 0.005)
    vertices2 = np.c_[[0.425, 1.5, -0.5], [0.425, 1.5, 0.5],
                      [0.425, -1.5, -0.5], [0.425, -1.5, 0.5]]
    shape2 = Polyhedron3D(vertices2, 0.001)

obstacle1 = Obstacle({'position': [3.475, -0.65, 0.5]}, shape1, {}, {'draw': False})
obstacle2 = Obstacle({'position': [0.425, 0.0, 1.6]}, shape2, {}, {'draw': False})
environment.add_obstacle([obstacle1, obstacle2])

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
    {'position': [6.875, -1.325, 0.4]}, Cuboid(0.15, 0.15, 0.8), {}, {'avoid': False})
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
simulator.plot.set_options({'knots': True})
simulator.plot.show('state', label=['x (m)', 'y (m)', 'z (m)'])
# simulator.plot.show('velocity', label=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'])
# simulator.plot.show(
#     'acceleration', label=['ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'])
# simulator.plot.show(
#     'jerk', label=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'])
simulator.plot.show_movie(
    'scene', number_of_frames=40, repeat=False, view=[60, 45])
simulator.plot.show_movie(
    'scene', number_of_frames=40, repeat=True, view=[0, 90])

# save results
if save:
    simulator.plot.save('state', 'position', label=[
                        'x (m)', 'y (m)', 'z (m)'], figurewidth='15cm', figureheight='4cm')
    simulator.plot.save('velocity', 'velocity', label=[
                        'dx (m/s)', 'dy (m/s)', 'dz (m/s)'], figurewidth='15cm', figureheight='4cm')
    simulator.plot.save('acceleration', 'acceleration', label=[
                        'ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'], figurewidth='15cm', figureheight='4cm')
    simulator.plot.save('jerk', 'jerk', label=[
                        'dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'], figurewidth='15cm', figureheight='4cm')

    simulator.plot.save_movie('scene', 'scene1', number_of_frames=40, view=[30, 60], limits=[
        [0., 10.], [-4.65, 5.35], [-4.85, 6.15]])
    simulator.plot.save_movie('scene', 'scene2', number_of_frames=40, view=[0, 90], limits=[
        [0., 10.], [-4.65, 5.35], [-4.85, 6.15]])
matplotlib.pyplot.show(block=True)
