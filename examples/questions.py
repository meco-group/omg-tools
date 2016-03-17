from omgtools import *

# create fleet
N = 8
vehicles = [Quadrotor(0.2) for l in range(N)]
fleet = Fleet(vehicles)

configuration = [[-1.5, 0.], [-0.75, 1.29], [0.75, 1.29], [1.5, 0.],
                 [0.75, -1.29], [0., -4.2], [0., -3.3], [0., -2.4]]
init_positions = RegularPolyhedron(4., N, np.pi/4).vertices.T.tolist()
terminal_positions = np.zeros((N, 2)).tolist()

fleet.set_configuration(configuration)
fleet.set_initial_conditions(init_positions)
fleet.set_terminal_conditions(terminal_positions)

# create environment
environment = Environment(room={'shape': Square(10.)})

# create a formation point-to-point problem
options = {'horizon_time': 5, 'codegen': {'jit': False}, 'admm': {'rho': 3.}}
problem = RendezVous(fleet, environment, options=options)
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
simulator.plot.show_movie('scene', repeat=True)
