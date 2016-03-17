from omgtools import *

# create vehicle
vehicle = Holonomic3D(Cuboid(width=0.5, height=0.2, depth=1.))

vehicle.set_initial_conditions([-2., -2., -2])
vehicle.set_terminal_conditions([2., 2., -2])

# create environment
environment = Environment(room={'shape': Cube(5.)})
environment.add_obstacle(Obstacle(
    {'position': [0., 0., -1.5]}, shape=Cuboid(width=0.5, depth=4., height=2.)))
trajectories = {'velocity': {4: [0.0, 0.0, 1.]}}
environment.add_obstacle(Obstacle(
    {'position': [1., 1., -2.25]}, shape=Cube(0.25), trajectories=trajectories))

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=False)
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True, 'prediction': False})
simulator.plot.show('scene')
simulator.plot.show('state')
simulator.plot.show('input')

# run it!
simulator.run()

# # show/save some results
simulator.plot.show_movie('scene', repeat=True)
