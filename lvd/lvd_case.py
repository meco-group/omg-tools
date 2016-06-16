# Implementation of pick and place case of LVD
# questions: ruben.vanparys@kuleuven.be

import sys
import os
sys.path.insert(0, os.getcwd()+'/../')
import matplotlib
import numpy as np
from omgtools import *
from lvd_fap import FAP

# simulation
go_forward = True
hard_stop = True
go_back = True

# results
save_figures = False
plot_figures = True
plot_movies = True


def reinit_after_hard_stop(problem, stop_time=None):
    fap = problem.vehicles[0]
    current_position = fap.signals['state'][:, -1]
    if not stop_time:
        # search stop_time
        stop_time = 0.
        dist = np.inf
        for k in range(fap.trajectories['splines'].shape[1]):
            value = fap.trajectories['splines'][:, k]
            _dist = np.linalg.norm(value - current_position)
            if _dist < dist:
                dist = _dist
                stop_time = fap.trajectories['time'][:, k]
    problem.set_init_time(stop_time)


def save_trajectories(trajectories, name):
    time = np.copy(trajectories['vehicle0']['time'])
    state = np.copy(trajectories['vehicle0']['state'])
    for k in range(3):
        # transform to machine frame
        state[k, :] = state[k, :] - 0.5*plate['dim'][k]
    data = np.r_[time, state].T
    np.savetxt(name+'.csv', data, delimiter=',')

"""modeling parameters"""

# obstacles (defined wrt machine frame)
table = {'pos': [0., -1.5, 0.], 'dim': [3., 3., 1.]}
frame = {'pos': [0., -1.5, 1.4], 'dim': [0.33, 3., 0.9]}
cover = {'pos': [0.33, 0.2, 1.1], 'dim': [0.25, 1., 0.75]}
beam = {'pos': [0., -2.2, 1.], 'dim': [2.7, 0.5, 0.4]}
leg = {'pos': [3.05, -1.7, 0.], 'dim': [0.3, 1.6, 0.7]}
pilar = {'pos': [3.05, -2.15, 0.], 'dim': [0.3, 0.45, 1.5]}
measure = {'pos': [6.8, -1.4, 0.], 'dim': [0.15, 0.15, 1.15]}

# plate (defined wrt machine frame)
plate = {'start': [7., -1.5, 0.4], 'end': [0., -1.3, 1.01], 'end2': [3.5, -1.4, 0.4],
         'dim': [3., 1.5, 0.01]}

# room limits (should prevent collisions with beam & pillar)
room = {'min': [-0.1, -1.6, plate['start'][2]], 'max': [7.1, 0.4, 1.15]}

# some case specific settings
table_tolerance = 0.002
plate_deflection = 0.003 # (plate_deflection + table_tolerance < diff table and end pos plate)
cover_tolerance = 0.01
cover_front_tolerance = 0.1
measure_tolerance = 0.01

# table + leg
obstacle1 = {'label': 'tableleg', 'pos': table['pos'][:], 'dim': table['dim'][:]}
obstacle1['dim'][0] = leg['pos'][0]-table['pos'][0]+leg['dim'][0]
obstacle1['radius'] = table_tolerance + plate_deflection
# cover + frame
obstacle2 = {'label': 'coverframe', 'pos': frame['pos'][:], 'dim': frame['dim'][:]}
obstacle2['dim'][0] = cover_front_tolerance+cover['pos'][0]-frame['pos'][0]+cover['dim'][0]
obstacle2['dim'][2] = frame['pos'][2]+frame['dim'][2]-cover['pos'][2]
obstacle2['pos'][2] = cover['pos'][2]
obstacle2['radius'] = cover_tolerance
# measure
obstacle3 = {'label': 'measure', 'pos': measure['pos'][:], 'dim': measure['dim'][:]}
obstacle3['radius'] = measure_tolerance

obstacles_draw = [table, frame, cover, beam, leg, pilar, measure]
obstacles_avoid = [obstacle1, obstacle2, obstacle3]

"""modeling with omg-tools"""

# 1. create FAP machine object
# create plate shape
shape_plate = Plate(Rectangle(plate['dim'][0], plate['dim'][1]), plate['dim'][2])
# limits on position of plate center
bounds = {'smin': [room['min'][k]+0.5*plate['dim'][k] for k in range(3)],
          'smax': [room['max'][k]+0.5*plate['dim'][k] for k in range(3)]}
bounds['smax'][1] = -0.1
# shutdown room constraints (we use constraints on position)
options = {'room_constraints': False}
fap = FAP(shape_plate, options=options, bounds=bounds)
fap.define_knots(knot_intervals=10)
fap.set_initial_conditions([plate['start'][k]+0.5*plate['dim'][k] for k in range(3)])
fap.set_terminal_conditions([plate['end'][k]+0.5*plate['dim'][k] for k in range(3)])

# 2. create obstacles
obstacles = []
# create obstacles for drawing
for obst in obstacles_draw:
    pos_obst = [obst['pos'][k] + 0.5*obst['dim'][k] for k in range(3)]
    shape_obst = Cuboid(obst['dim'][0], obst['dim'][1], obst['dim'][2])
    obstacles.append(Obstacle({'position': pos_obst}, shape_obst, {}, {'avoid': False}))
# create obstacles for collision avoidance
for obst in obstacles_avoid:
    pos_obst = [obst['pos'][k] + 0.5*obst['dim'][k] for k in range(3)]
    if False:
        shape_obst = Cuboid(obst['dim'][0], obst['dim'][1], obst['dim'][2])
        shape_obst.radius = obst['radius']
    else:
        if obst['label'] == 'tableleg':
            vert_ind = [[1, -1, -1], [1, 1, -1], [1, -1, 1], [1, 1, 1], [-1, -1, 1], [-1, 1, 1]]
        if obst['label'] == 'coverframe':
            vert_ind = [[1, 1, -1], [1, 1, 1], [1, -1, -1], [1, -1, 1]]
        if obst['label'] == 'measure':
            vert_ind = [[1, -1, -1], [1, 1, -1], [1, -1, 1], [1, 1, 1], [-1, -1, 1], [-1, 1, 1]]
        vert_obst = np.zeros((3, len(vert_ind)))
        for k, ind in enumerate(vert_ind):
            vert_obst[:, k] = 0.5*np.array(obst['dim'])*ind
        shape_obst = Polyhedron3D(vert_obst, obst['radius'])
    obstacles.append(Obstacle({'position': pos_obst}, shape_obst, {}, {'draw': False}))

# 3. create environment
environment = Environment(room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})
environment.add_obstacle(obstacles)

# 4. create problem
problem = Point2point(fap, environment, freeT=True)
problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
problem.init()


"""simulation with omg-tools"""

simulator = Simulator(problem, sample_time=0.001)
if plot_figures:
    fap.plot('state', labels=['x (m)', 'y (m)', 'z (m)'])
    fap.plot('velocity', labels=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'])
    fap.plot('acceleration', labels=['ddx (m/s)', 'ddy (m/s)', 'ddz (m/s)'])
    fap.plot('jerk', labels=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'])

# from initial stack to machine
if go_forward:
    if hard_stop:
        trajectories = simulator.run_once(hard_stop={'time': 4., 'perturbation': [0.05, 0.05, 0.05]})
        simulator.sleep(3.)
        # recover from hard stop
        reinit_after_hard_stop(problem, 4.)
        trajectories = simulator.run_once()
    else:
        trajectories = simulator.run_once()
        # save_trajectories(trajectories, 'go_forward')

# from machine to final stack
if go_back:
    if not go_forward:
        fap.set_initial_conditions([plate['end'][k]+0.5*plate['dim'][k] for k in range(3)])
    fap.set_terminal_conditions([plate['end2'][k]+0.5*plate['dim'][k] for k in range(3)])
    # re-initialize
    fap.reinit_splines(problem)
    trajectories = simulator.run_once()
    # save_trajectories(trajectories, 'go_back')

if plot_movies:
    problem.plot_movie('scene', number_of_frames=100, repeat=False, view=[30, 60])
    problem.plot_movie('scene', number_of_frames=100, repeat=False, view=[0, 90])
    problem.plot_movie('scene', number_of_frames=100, repeat=False, view=[90, 0])

if save_figures:
    fap.save_plot('state', 'position', labels=['x (m)', 'y (m)', 'z (m)'],
        figurewidth='15cm', figureheight='4cm')
    fap.save_plot('velocity', 'velocity', labels=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'],
        figurewidth='15cm', figureheight='4cm')
    fap.save_plot('acceleration', 'acceleration', labels=['ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'],
        figurewidth='15cm', figureheight='4cm')
    fap.save_plot('jerk', 'jerk', labels=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'],
        figurewidth='15cm', figureheight='4cm')

    problem.save_movie('scene', 'scene1', number_of_frames=40, view=[30, 60], axis=False)
    problem.save_movie('scene', 'scene2', number_of_frames=40, view=[0, 90], axis=False)
    problem.save_movie('scene', 'scene3', number_of_frames=40, view=[90, 0], axis=False)

matplotlib.pyplot.show(block=True)
