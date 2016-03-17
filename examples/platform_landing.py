from omgtools import *

# create fleet
N_quad = 2
quadrotors = [Quadrotor(0.2) for l in range(N_quad)]
fleet = Fleet(quadrotors + [Platform()])

configuration = [[0.25], [-0.25], [0.0]]
init_positions = [[1.5, 3.], [-2., 2.], [1.]]
terminal_positions = [[0., 0.1], [0., 0.1], [0.]]

fleet.set_configuration(configuration)
fleet.set_initial_conditions(init_positions)
fleet.set_terminal_conditions(terminal_positions)

# create environment
environment = Environment(room={'shape': Square(5.), 'position': [0., 2.]})
environment.add_obstacle(Obstacle({'position': [1., 1.5]},
                                  shape=Rectangle(width=1, height=0.2)))

# create a formation point-to-point problem
options = {'horizon_time': 5., 'codegen': {'jit': False}, 'admm': {'rho': 3.}}
problem = RendezVous(fleet, environment, options=options)
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
simulator.plot.show_movie('scene', repeat=True)
