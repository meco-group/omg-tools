import sys
sys.path.append("/home/ruben/Documents/Work/Programs/motionplanningtoolbox/")
from motionplanning import *
import numpy as np

# create fleet
N = 4

vehicles = [Holonomic() for l in range(N)]
for veh in vehicles:
    veh.set_options({'boundary_smoothness': {'initial': 1}})
    # veh.set_options({'safety_distance': 0.1})
    # veh.set_options({'1storder_delay': True, 'time_constant': 0.1})
    # veh.set_input_disturbance(fc=0.01, stdev=0.05*np.ones(2))

fleet = Fleet(vehicles)
fleet.set_configuration(polyhedron=SymmetricPolyhedron(0.2, N, np.pi/4.))

fleet.set_initial_pose([-1.5, -1.5])
fleet.set_terminal_pose([2., 2.])

# create environment
environment = Environment(room={'shape': Square(2.5)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
trajectory = {'velocity': np.vstack([[3., -0.15, 0.0], [4., 0., 0.15]])}
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  trajectory=trajectory))

# create a formation point-to-point problem
options = {'codegen': {'jit': False}, 'admm': {'rho': 2.}}
problem = FormationPoint2point(fleet, environment, options=options)
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})
simulator.plot.create('2d')
simulator.plot.create('input')

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('2d', repeat=False)
# problem.plot.save_movie('input', 5, 'holonomic')
# problem.plot.show('input')

matplotlib.pyplot.show(block=True)
