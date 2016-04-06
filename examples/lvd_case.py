# Implementation of pick and place case of LVD
# questions: ruben.vanparys@kuleuven.be
import sys, os
sys.path.insert(0, os.getcwd()+'/../')
import matplotlib
from omgtools import *
from omgtools.vehicles.lvd_machine import LVD

# create plate
# limits on position mid-point plate
bounds = {'smin': [None, -0.95, 0.405], 'smax': [None, -0.55, None]}
# shutdown room constraints (we use constraints on position)
options = {'room_constraint': None}
# shape of plate
shape = Plate(Rectangle(3., 1.5), 0.01)
plate = LVD(shape, options=options, bounds=bounds)

plate.set_initial_conditions([8.5, -0.75, 0.405])
plate.set_terminal_conditions([1.5, -0.55, 1.015])

# create environment
environment = Environment(
    room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})

# obstacles for collision avoidance
cuboid1 = Cuboid(6.950, 1.7, 1.0)
cuboid1.radius = 0.005  # for safety
cuboid = Obstacle({'position': [3.475, -0.65, 0.5]}, cuboid1, draw=False)
frame = Obstacle(
    {'position': [0.165, 0.0, 1.85]}, Cuboid(0.33, 3.0, 0.9), avoid=True)
environment.add_obstacle([cuboid, frame])

# obstacles just for drawing
table = Obstacle(
    {'position': [1.5, 0.0, 0.5]}, Cuboid(3., 3., 1.), avoid=False)
cover = Obstacle(
    {'position': [0.455, 0.7, 1.475]}, Cuboid(0.25, 1., 0.75), avoid=False)
beam = Obstacle(
    {'position': [1.35, -1.95, 1.2]}, Cuboid(2.7, 0.5, 0.4), avoid=False)
leg = Obstacle(
    {'position': [3.2, -0.9, 0.35]}, Cuboid(0.3, 1.6, 0.7), avoid=False)
pilar = Obstacle(
    {'position': [3.2, -1.925, 0.75]}, Cuboid(0.3, 0.45, 1.5), avoid=False)
measure = Obstacle(
    {'position': [6.875, -1.325, 0.4]}, Cuboid(0.15, 0.15, 0.8), avoid=False)
environment.add_obstacle([table, cover, beam, leg, pilar, measure])

# create a point-to-point problem
problem = Point2point(plate, environment, freeT=True)
problem.set_options({'solver': {'ipopt.linear_solver': 'ma57'}, 'horizon_time': 12.})
problem.init()

# create simulator
simulator = Simulator(problem)

# run it!
trajectories = simulator.run_once()

# show results
simulator.plot.show('state', label=['x (m)', 'y (m)', 'z (m)'])
simulator.plot.show('velocity', label=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'])
simulator.plot.show(
    'acceleration', label=['ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'])
simulator.plot.show(
    'jerk', label=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'])
simulator.plot.show_movie(
    'scene', number_of_frames=40, repeat=True, view=[60, 45])

# save results
# simulator.plot.save('state', 'position', label=['x (m)', 'y (m)', 'z (m)'], figurewidth='15cm', figureheight='4cm')
# simulator.plot.save('velocity', 'velocity', label=['dx (m/s)', 'dy (m/s)' ,'dz (m/s)'], figurewidth='15cm', figureheight='4cm')
# simulator.plot.save('acceleration', 'acceleration', label=['ddx (m/s^2)', 'ddy (m/s^2)' ,'ddz (m/s^2)'], figurewidth='15cm', figureheight='4cm')
# simulator.plot.save('jerk', 'jerk', label=['dddx (m/s^3)', 'dddy (m/s^3)' ,'dddz (m/s^3)'], figurewidth='15cm', figureheight='4cm')
# simulator.plot.save('scene')

# simulator.plot.show('scene', time=0, view=[30, 60], limits=[
#                     [0., 10.], [-4.65, 5.35], [-4.85, 6.15]])
matplotlib.pyplot.show(block=True)
