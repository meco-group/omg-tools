from omgtools import *

# create fleet
N = 3

veh_opt = {'horizon_time': 5.}
vehicles = [Quadrotor(0.2, veh_opt) for l in range(N)]
# for veh in vehicles:
#     veh.set_options({'safety_distance': 0.1})
#     veh.set_options({'1storder_delay': True, 'time_constant': 0.1})
#     veh.set_input_disturbance(3., 0.5*np.ones(2))

fleet = Fleet(vehicles)
fleet.set_configuration(polyhedron=SymmetricPolyhedron(0.4, N))

fleet.set_initial_pose([-4., -4.])
fleet.set_terminal_pose([4., 4.])

# create environment
environment = Environment(room={'shape': Square(10.)})
environment.add_obstacle(Obstacle({'position': [-0.6, 3.7]},
                                  shape=Rectangle(width=0.2, height=3.)))
environment.add_obstacle(Obstacle({'position': [-0.6, -5.4]},
                                  shape=Rectangle(width=0.2, height=10.)))

# create a formation point-to-point problem
options = {'codegen': {'jit': False}, 'admm': {'rho': 0.1}}
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

matplotlib.pyplot.show(block=True)
