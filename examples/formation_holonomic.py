from omgtools import *

# create fleet
N = 4
vehicles = [Holonomic() for l in range(N)]

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
trajectories = {'velocity': {3: [-0.15, 0.0], 4: [0., 0.15]}}
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  trajectories=trajectories))

# create a formation point-to-point problem
options = {'codegen': {'jit': False}, 'admm': {'rho': 2.}, 'horizon_time': 10}
problem = FormationPoint2point(fleet, environment, options=options)
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})
simulator.plot.show('scene')
simulator.plot.show('input')

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('scene', repeat=False)
