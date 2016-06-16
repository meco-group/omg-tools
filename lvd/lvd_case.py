# Implementation of pick and place case of LVD
# questions: ruben.vanparys@kuleuven.be

import sys
import os
sys.path.insert(0, os.getcwd()+'/../')
import matplotlib
from omgtools import *
from lvd_fap import FAP

save = False

"""quick settings"""

# obstacles (defined wrt machine frame)
table = {'pos': [0., -1.5, 0.], 'dim': [3., 3., 1.]}
frame = {'pos': [0., -1.5, 1.4], 'dim': [0.33, 3., 0.9]}
cover = {'pos': [0.33, 0.2, 1.1], 'dim': [0.25, 1., 0.75]}
beam = {'pos': [0., -2.2, 1.], 'dim': [2.7, 0.5, 0.4]}
leg = {'pos': [3.05, -1.7, 0.], 'dim': [0.3, 1.6, 0.7]}
pilar = {'pos': [3.05, -2.15, 0.], 'dim': [0.3, 0.45, 1.5]}
measure = {'pos': [6.8, -1.4, 0.], 'dim': [0.15, 0.15, 1.15]}

# plate (defined wrt machine frame)
plate = {'start': [7., -1.5, 0.4], 'end': [0., -1.3, 1.01], 'end2': [3.575, -1.5, 0.4],
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

"""implementation with omg-tools"""

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
problem1 = Point2point(fap, environment, freeT=True)
problem1.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
problem1.init()
problem2 = Point2point(fap, environment, freeT=True)
problem2.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
obstacles[-1].set_options({'avoid': False})
problem2.init()


# 5. simulate
simulator = Simulator(problem1, sample_time=0.001)
fap.plot('state', labels=['x (m)', 'y (m)', 'z (m)'])

# from initial stack to machine
trajectories = simulator.run_once()

# from machine to final stack
fap.set_terminal_conditions([plate['end2'][k]+0.5*plate['dim'][k] for k in range(3)])
# re-initialize
fap.reinit_splines(problem2)
simulator.set_problem(problem2)
trajectories = simulator.run_once()

problem2.plot_movie('scene', number_of_frames=40, repeat=False, view=[30, 60])

# fap.set_initial_conditions([plate['end'][k]+0.5*plate['dim'][k] for k in range(3)])
# problem2 = Point2point(fap, environment2, freeT=True)
# problem2.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
# problem2.init()

# simulator.problem = problem2

# trajectories = simulator.run_once()


# environment2 = Environment(room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})
# environment2.add_obstacle(obstacles[:-1]) # no measure
# problem2 = Point2point(fap, environment2, freeT=True)
# problem2.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
# problem2.init()

# fap.set_initial_conditions([plate['end'][k]+0.5*plate['dim'][k] for k in range(3)])
# fap.set_terminal_conditions([plate['end2'][k]+0.5*plate['dim'][k] for k in range(3)])
# simulator.set_problem(problem2)
# simulator.run_once()
# problem2.plot_movie('scene', number_of_frames=40, repeat=False, view=[30, 60])


# fap = create_FAP_machine(plate, room)
# obstacles = create_FAP_obstacles(obstacles_draw, obstacles_avoid)



# simulator = Simulator(problem1, sample_time=0.001)

# trajectories = simulator.run_once(update=True)

# # fap.plot('state', labels=['x (m)', 'y (m)', 'z (m)'], knots=True)


# # trajectories = simulator.run_once(update=True)

# fap.set_initial_conditions([plate['end'][k]+0.5*plate['dim'][k] for k in range(3)])
# fap.set_terminal_conditions([plate['end2'][k]+0.5*plate['dim'][k] for k in range(3)])

# problem2 = create_FAP_problem(fap, obstacles_draw, [obstacle1, obstacle2])

# simulator.set_problem(problem2)
# simulator.run_once()

# problem2.plot_movie('scene', number_of_frames=40, repeat=False, view=[30, 60])
# # 1. create FAP machine object

# # create plate shape
# shape_plate = Plate(Rectangle(plate['dim'][0], plate['dim'][1]), plate['dim'][2])
# # limits on position of plate center
# bounds = {'smin': [room['min'][k]+0.5*plate['dim'][k] for k in range(3)],
#           'smax': [room['max'][k]+0.5*plate['dim'][k] for k in range(3)]}
# bounds['smax'][1] = -0.1

# # shutdown room constraints (we use constraints on position)
# options = {'room_constraints': False}
# fap = FAP(shape_plate, options=options, bounds=bounds)
# fap.define_knots(knot_intervals=10)
# # set initial and terminal position
# fap.set_initial_conditions([plate['start'][k]+0.5*plate['dim'][k] for k in range(3)])
# fap.set_terminal_conditions([plate['end'][k]+0.5*plate['dim'][k] for k in range(3)])

# # 2. create environment

# environment = Environment(room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})

# # create obstacles for collision avoidance
# for obst in obstacles_avoid:
#     pos_obst = [obst['pos'][k] + 0.5*obst['dim'][k] for k in range(3)]
#     if False:
#         shape_obst = Cuboid(obst['dim'][0], obst['dim'][1], obst['dim'][2])
#         shape_obst.radius = obst['radius']
#     else:
#         if obst['label'] == 'tableleg':
#             vert_ind = [[1, -1, -1], [1, 1, -1], [1, -1, 1], [1, 1, 1], [-1, -1, 1], [-1, 1, 1]]
#         if obst['label'] == 'coverframe':
#             vert_ind = [[1, 1, -1], [1, 1, 1], [1, -1, -1], [1, -1, 1]]
#         if obst['label'] == 'measure':
#             vert_ind = [[1, -1, -1], [1, 1, -1], [1, -1, 1], [1, 1, 1], [-1, -1, 1], [-1, 1, 1]]
#         vert_obst = np.zeros((3, len(vert_ind)))
#         for k, ind in enumerate(vert_ind):
#             vert_obst[:, k] = 0.5*np.array(obst['dim'])*ind
#         shape_obst = Polyhedron3D(vert_obst, obst['radius'])
#     # environment.add_obstacle(Obstacle({'position': pos_obst}, shape_obst, {}, {'draw': False}))

# # create obstacles for drawing
# for obst in obstacles_draw:
#     pos_obst = [obst['pos'][k] + 0.5*obst['dim'][k] for k in range(3)]
#     shape_obst = Cuboid(obst['dim'][0], obst['dim'][1], obst['dim'][2])
#     environment.add_obstacle(Obstacle({'position': pos_obst}, shape_obst, {}, {'avoid': False}))

# # 3. create a point-to-point problem

# problem = Point2point(fap, environment, freeT=True)
# problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
# problem.init()

# 4. create simulator, run simulation and show/save results

# simulator = Simulator(problem1, sample_time=0.001)
# trajectories = simulator.run_once(update=True)

# fap.plot('state', labels=['x (m)', 'y (m)', 'z (m)'], knots=True)
# fap.plot('velocity', labels=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'])
# fap.plot('acceleration', labels=['ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'])
# fap.plot('jerk', labels=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'])
# problem.plot_movie('scene', number_of_frames=40, repeat=False, view=[30, 60])
# problem.plot_movie('scene', number_of_frames=40, repeat=False, view=[0, 90])
# problem1.plot_movie('scene', number_of_frames=40, repeat=True, view=[90, 0])

if save:
    fap.save_plot('state', 'position', labels=['x (m)', 'y (m)', 'z (m)'],
        figurewidth='15cm', figureheight='4cm')
    fap.save_plot('velocity', 'velocity', labels=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'],
        figurewidth='15cm', figureheight='4cm')
    fap.save_plot('acceleration', 'acceleration', labels=['ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'],
        figurewidth='15cm', figureheight='4cm')
    fap.save_plot('jerk', 'jerk', labels=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'],
        figurewidth='15cm', figureheight='4cm')

    problem1.save_movie('scene', 'scene1', number_of_frames=40, view=[30, 60], axis=False)
    problem1.save_movie('scene', 'scene2', number_of_frames=40, view=[0, 90], axis=False)
    problem1.save_movie('scene', 'scene3', number_of_frames=40, view=[90, 0], axis=False)

# set new initial and terminal position
# fap.set_initial_conditions([plate['end'][k]+0.5*plate['dim'][k] for k in range(3)])
# fap.set_terminal_conditions([plate['end2'][k]+0.5*plate['dim'][k] for k in range(3)])

# trajectories2 = simulator.run_once()
# problem.plot_movie('scene', number_of_frames=40, repeat=False, view=[30, 60])

# problem.plot_movie('scene', number_of_frame=40, repeat=False, view=[30, 60])




# matplotlib.pyplot.show(block=True)






# # limits on position mid-point plate
# # bounds = {'smin': [None, -0.95, 0.405], 'smax': [None, -0.55, 1.5]}


# bounds = {'smin': [1.4, -0.85, 0.405], 'smax': [8.6, -0.1, 1.155]}
# # shutdown room constraints (we use constraints on position)
# options = {'room_constraints': False}
# # create plate object
# shape = Plate(Rectangle(3., 1.5), 0.01)
# plate = LVD(shape, options=options, bounds=bounds)
# plate.define_knots(knot_intervals=9)

# plate.set_initial_conditions([8.5, -0.75, 0.405])
# plate.set_terminal_conditions([1.5, -0.55, 1.015])

# # create environment
# environment = Environment(
#     room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})

# # obstacles for collision avoidance
# cuboids = False  # False -> a bit faster
# if cuboids:
#     # table
#     shape1 = Cuboid(3.350, 1.7, 1.0)
#     shape1.radius = plate_deflection + table_tolerance
#     position1 = [1.675, -0.65, 0.5]
#     # cover + frame
#     shape2 = Cuboid(0.58, 3.0, 1.0)
#     position2 = [0.29 + 0.1, 0.0, 1.6] # +0.1 for safety
#     # measure
#     shape3 = Cuboid(0.15, 0.15, 1.15)
#     position3 = [6.875, -1.325, 0.575]
# else:
#     # table
#     vertices1 = np.c_[[1.675, -0.85, -0.5], [1.675, 0.85, -0.5],
#                       [1.675, -0.85,  0.5], [1.675, 0.85,  0.5],
#                       [-1.675, -0.85,  0.5], [-1.675, 0.85,  0.5]]
#     radius = plate_deflection + table_tolerance
#     shape1 = Polyhedron3D(vertices1, radius)
#     position1 = [1.675, -0.65, 0.5]
#     # cover + frame
#     vertices2 = np.c_[[0.29, 1.5, -0.5], [0.29, 1.5, 0.5],
#                       [0.29, -1.5, -0.5], [0.29, -1.5, 0.5]]
#     shape2 = Polyhedron3D(vertices2, 0.001)
#     position2 = [0.29 + 0.1, 0.0, 1.6] # +0.1 for safety
#     # measure
#     vertices3 = np.c_[[0.075, -0.075, -0.575], [0.075, 0.075, -0.575],
#                       [0.075, -0.075, 0.575], [0.075, 0.075, 0.575],
#                       [-0.075, -0.075, 0.575], [-0.075, 0.075, 0.575]]
#     shape3 = Polyhedron3D(vertices3, 0.001)
#     position3 = [6.875, -1.325, 0.575]

# obstacle1 = Obstacle({'position': position1}, shape1, {}, {'draw': True})
# obstacle2 = Obstacle({'position': position2}, shape2, {}, {'draw': True})
# obstacle3 = Obstacle({'position': position3}, shape3, {}, {'draw': True})

# environment.add_obstacle([obstacle1, obstacle2, obstacle3])

# # obstacles just for drawing
# frame = Obstacle(
#     {'position': [0.165, 0.0, 1.85]}, Cuboid(0.33, 3.0, 0.9), {}, {'avoid': False})
# table = Obstacle(
#     {'position': [1.5, 0.0, 0.5]}, Cuboid(3., 3., 1.), {}, {'avoid': False})
# cover = Obstacle(
#     {'position': [0.455, 0.7, 1.475]}, Cuboid(0.25, 1., 0.75), {}, {'avoid': False})
# beam = Obstacle(
#     {'position': [1.35, -1.95, 1.2]}, Cuboid(2.7, 0.5, 0.4), {}, {'avoid': False})
# leg = Obstacle(
#     {'position': [3.2, -0.9, 0.35]}, Cuboid(0.3, 1.6, 0.7), {}, {'avoid': False})
# pilar = Obstacle(
#     {'position': [3.2, -1.925, 0.75]}, Cuboid(0.3, 0.45, 1.5), {}, {'avoid': False})
# measure = Obstacle(
#     {'position': [6.875, -1.325, 0.575]}, Cuboid(0.15, 0.15, 1.15), {}, {'avoid': False})
# # environment.add_obstacle([frame, table, cover, beam, leg, pilar, measure])

# # create a point-to-point problem
# problem = Point2point(plate, environment, freeT=True)
# problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
# problem.init()

# # create simulator
# simulator = Simulator(problem, sample_time=0.001)

# # run it!
# trajectories = simulator.run_once()

# # # show results
# plate.plot('state', labels=['x (m)', 'y (m)', 'z (m)'], knots=True)
# plate.plot('velocity', labels=['dx (m/s)', 'dy (m/s)', 'dz (m/s)'])
# plate.plot(
#     'acceleration', labels=['ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'])
# plate.plot(
#     'jerk', labels=['dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'])
# # problem.plot_movie('scene', number_of_frames=40, repeat=False, view=[30, 60], limits=[
# #         [0., 10.], [-4.65, 5.35], [-4.85, 6.15]])

# # problem.plot_movie('scene', number_of_frames=40, repeat=False, view=[30, 60])
# # problem.plot_movie(
# #     'scene', number_of_frames=40, repeat=False, view=[0, 90])
# # problem.plot_movie(
# #     'scene', number_of_frames=40, repeat=True, view=[90, 0])
# # problem.save_plot('scene', 'obstacles', axis=False, view=[60, 45], time=0)

# # save results
# if save:
#     plate.save_plot('state', 'position', labels=[
#         'x (m)', 'y (m)', 'z (m)'], figurewidth='15cm', figureheight='4cm')
#     plate.save_plot('velocity', 'velocity', labels=[
#         'dx (m/s)', 'dy (m/s)', 'dz (m/s)'], figurewidth='15cm', figureheight='4cm')
#     plate.save_plot('acceleration', 'acchttps://gitlab.mech.kuleuven.be/meco/omg-tools.giteleration', labels=[
#         'ddx (m/s^2)', 'ddy (m/s^2)', 'ddz (m/s^2)'], figurewidth='15cm', figureheight='4cm')
#     plate.save_plot('jerk', 'jerk', labels=[
#         'dddx (m/s^3)', 'dddy (m/s^3)', 'dddz (m/s^3)'], figurewidth='15cm', figureheight='4cm')

#     problem.save_movie('scene', 'scene1', number_of_frames=40, view=[30, 60], axis=False)
#     problem.save_movie('scene', 'scene2', number_of_frames=40, view=[0, 90], axis=False)
#     problem.save_movie('scene', 'scene3', number_of_frames=40, view=[90, 0], axis=False)
matplotlib.pyplot.show(block=True)
