import sys
sys.path.insert(0, "/home/ruben/Documents/Work/Programs/motionplanningtoolbox/")
from omgtools import *


# create vehicle
vehicle = Holonomic()

# at start only constraints up to 1st derivative
# vehicle.set_options({'boundary_smoothness': {'initial': 1}})
vehicle.set_options({'safety_distance': 0.1})
# vehicle.set_options({'1storder_delay': True, 'time_constant': 0.1})
# vehicle.set_input_disturbance(fc = 0.01, stdev = 0.05*np.ones(2))

vehicle.set_initial_pose([-1.5, -1.5])
vehicle.set_terminal_pose([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

# environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
# environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
# trajectory = {'velocity': [[3., -0.15, 0.0], [4., 0., 0.15]]}
# environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
#                                   trajectory=trajectory))

# create a point-to-point problem
problem = Point2point(vehicle, environment)
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

options = {}
# options['directory'] = '/home/ruben/ourbot/orocos/ourbot/MotionPlanning/src/Toolbox/'
# options['casadiobj'] = '/home/odroid/orocos/MotionPlanning/src/Toolbox/bin/'
options['casadiinc'] = '/home/ruben/Documents/Work/Repositories/casadi_binary/include/'
options['casadilib'] = '/home/ruben/Documents/Work/Repositories/casadi_binary/casadi/'
options['sourcefiles'] = 'example.cpp'
problem.export('c++', options)

# create simulator
# simulator = Simulator(problem)
# simulator.plot.set_options({'knots': True})
# simulator.plot.create('2d')
# simulator.plot.create('input')

# run it!
# simulator.run()

# show/save some results
# simulator.plot.show_movie('2d', repeat=True)

# matplotlib.pyplot.show(block=True)
