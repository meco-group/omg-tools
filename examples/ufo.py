from omgtools import *

# create fleet
N = 3
vehicles = [Quadrotor(0.2) for l in range(N)]

fleet = Fleet(vehicles)
configuration = RegularPolyhedron(0.6, N, -np.pi/2).vertices.T
init_positions = [-1., -1.] + configuration
terminal_positions = [11., 11.] + configuration

fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(14.), 'position': [5., 5.]})
trajectory = {'velocity': {1.: [-9, 0.]}}
environment.add_obstacle(Obstacle({'position': [13, 4.]}, UFO(1.5, 0.6), trajectory))

# create a formation point-to-point problem
options = {'horizon_time': 5., 'codegen': {'jit': False}, 'admm': {'rho': 0.07}}
problem = FormationPoint2point(fleet, environment, options=options)
problem.set_options({'solver': {'linear_solver': 'ma57'}})
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})
# simulator.plot.show('scene')

# run it!
simulator.run()

# show/save some results
# simulator.plot.show_movie('scene', repeat=True)
simulator.plot.save_movie('scene', name='ufo', number_of_frames=300)
