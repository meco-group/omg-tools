import sys
sys.path.append("/home/ruben/Documents/Work/Programs/motionplanningtoolbox/")
from motionplanning import *

# create fleet
configuration = [np.array([-5., 0.]), np.array([-2.5, 4.3]),
                 np.array([2.5, 4.3]), np.array([5., 0.]),
                 np.array([2.5, -4.3]), np.array([0., -14.]),
                 np.array([0., -11.]), np.array([0., -8.])]
configuration = [0.3*conf for conf in configuration]
N = len(configuration)

veh_opt = {'horizon_time': 5.}
vehicles = [Quadrotor(0.2, veh_opt) for l in range(N)]
# for veh in vehicles:
#     veh.set_options({'safety_distance': 0.1})
#     veh.set_options({'1storder_delay': True, 'time_constant': 0.1})
#     veh.set_input_disturbance(3., 0.5*np.ones(2))

fleet = Fleet(vehicles)
fleet.set_configuration(points=configuration)

init_pos = []
Dth = 2*np.pi/N
Th0 = 6*np.pi/N
R = 4.
for l in range(N):
    init_pos.append([R*np.cos(Th0-l*Dth), R*np.sin(Th0-l*Dth)])
fleet.set_initial_pose(init_pos)

# create environment
environment = Environment(room={'shape': Square(10.)})

# create a formation point-to-point problem
options = {'codegen': {'jit': False}, 'admm': {'rho': 3.}}
problem = RendezVous(fleet, environment, options=options)
problem.init()

# create simulator
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})
simulator.plot.create('2d')
simulator.plot.create('input', label=['Thrust force (N/kg)',
                                      'Pitch rate (rad/s)'])

# run it!
simulator.run()

# show/save some results
simulator.plot.show_movie('2d', repeat=True)
# problem.plot.save_movie('input', 5, 'holonomic')
# problem.plot.show('input')

matplotlib.pyplot.show(block=True)
