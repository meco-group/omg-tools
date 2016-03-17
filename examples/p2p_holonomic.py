from omgtools import *

# create vehicle
vehicle = Holonomic()
vehicle.set_options({'safety_distance': 0.1})
# vehicle.set_options({'1storder_delay': True, 'time_constant': 0.1})
# vehicle.set_options({'input_disturbance': {'fc':0.01, 'stdev':0.05*np.ones(2)}})

vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
trajectories = {'velocity': {3: [-0.15, 0.0], 4: [0., 0.15]}}
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  trajectories=trajectories))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True, 'prediction': False})
simulator.plot.show('scene')
simulator.plot.show('input')

# run it!
simulator.run()
# simulator.plot.show('a', time=4)

# # show/save some results
simulator.plot.show_movie('scene', repeat=True)
# simulator.plot.save_movie('input', number_of_frames=4)
# simulator.plot.save('a', time=3)
