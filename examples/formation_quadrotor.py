from omgtools import *

# create fleet
N = 3
vehicles = [Quadrotor(0.2) for l in range(N)]

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.4, N).vertices.T
init_positions = [-4., -4.] + configuration
terminal_positions = [4., 4.] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(10.)})
environment.add_obstacle(Obstacle({'position': [-0.6, 3.7]},
                                  shape=Rectangle(width=0.2, height=3.)))
environment.add_obstacle(Obstacle({'position': [-0.6, -5.4]},
                                  shape=Rectangle(width=0.2, height=10.)))

# create a formation point-to-point problem
options = {'horizon_time': 5., 'codegen': {'jit': False}, 'admm': {'rho': 0.1}}
problem = FormationPoint2point(fleet, environment, options=options)
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})
simulator.plot.show('scene')
simulator.plot.show('input', label=['Thrust force (N/kg)',
                                    'Pitch rate (rad/s)'])

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('scene', repeat=False)
