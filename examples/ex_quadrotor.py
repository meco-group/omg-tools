import sys
sys.path.append("/home/ruben/Documents/Work/Programs/motionplanningtoolbox/")
from motionplanning import *

# create vehicle
vehicle = Quadrotor()
vehicle.set_initial_pose([-4., -4.])
vehicle.set_terminal_pose([4., 4.])
vehicle.set_options({'safety_distance': 0.1})
# vehicle.set_options({'1storder_delay': True, 'time_constant': 0.1})
# vehicle.set_input_disturbance(0.01, 0.1*np.ones(2))

# create environment
environment = Environment(room={'shape': Square(5.)})
environment.add_obstacle(Obstacle({'position': [-0.6, -5.4]},
                                  shape=Rectangle(width=0.2, height=12.)))

# create a point-to-point problem
problem = Point2point(vehicle, environment)
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})
simulator.plot.create('2d')
simulator.plot.create('input', label=['Thrust force (N/kg)',
                                      'Pitch rate (rad/s)'])

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('2d', repeat=False)
# simulator.plot.show('2d', time=2.)
# simulator.plot.show('input')
# simulator.plot.save_movie('input', number_of_frames=4)
# simulator.plot.save_movie('2d', number_of_frames=4, name='quadrotor_2d')

matplotlib.pyplot.show(block=True)
