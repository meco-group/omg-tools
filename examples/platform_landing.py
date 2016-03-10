from omgtools import *

# create fleet
configuration = [0.25, -0.25, 0.]
N_quad = len(configuration)-1

quadrotors = [Quadrotor(0.2) for l in range(N_quad)]
vehicles = quadrotors + [Platform()]

fleet = Fleet(vehicles)
fleet.set_configuration(points=configuration)

fleet.set_initial_pose(
    [np.array([1.5, 3.]), np.array([-2., 2.]), np.array([1.])])
fleet.set_terminal_pose(
    [np.array([0., 0.1]), np.array([0., 0.1]), np.array([0.])])

# create environment
environment = Environment(room={'shape': Square(5.), 'position': [0., 2.]})
environment.add_obstacle(Obstacle({'position': [1., 1.5]},
                                  shape=Rectangle(width=1, height=0.2)))

# create a formation point-to-point problem
options = {'horizon_time': 3., 'codegen': {'jit': False}, 'admm': {'rho': 3.}}
options['fixed_yT'] = [[1], [1], []]
problem = RendezVous(fleet, environment, options=options)
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})
simulator.plot.create('2d')
simulator.plot.create('input')

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('2d', repeat=True)

matplotlib.pyplot.show(block=True)
