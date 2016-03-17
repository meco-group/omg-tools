from omgtools import *

# create vehicle
vehicle = Quadrotor()
vehicle.set_options({'safety_distance': 0.1})
vehicle.set_initial_conditions([-4., -4.])
vehicle.set_terminal_conditions([4., 4.])

# create environment
environment = Environment(room={'shape': Square(10.)})
environment.add_obstacle(Obstacle({'position': [-0.6, -5.4]},
                                  shape=Rectangle(width=0.2, height=12.)))

# create a point-to-point problem
problem = Point2point(vehicle, environment, {'horizon_time': 5})
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True, 'prediction': False})
simulator.plot.show('scene')
simulator.plot.show('input', label=['Thrust force (N/kg)',
                                    'Pitch rate (rad/s)'])

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('scene', repeat=True)
