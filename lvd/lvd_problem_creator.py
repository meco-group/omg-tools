import sys
import os
sys.path.insert(0, os.getcwd()+'/../')
import matplotlib
from omgtools import *
from omgtools.vehicles.lvd_fap import FAP

def create_FAP_machine(plate, room):
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
    return fap

def create_FAP_obstacles(obstacles_draw, obstacles_avoid):
    obstacles = []
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
        # obstacles.append(Obstacle({'position': pos_obst}, shape_obst, {}, {'draw': False}))

    # create obstacles for drawing
    for obst in obstacles_draw:
        pos_obst = [obst['pos'][k] + 0.5*obst['dim'][k] for k in range(3)]
        shape_obst = Cuboid(obst['dim'][0], obst['dim'][1], obst['dim'][2])
        obstacles.append(Obstacle({'position': pos_obst}, shape_obst, {}, {'avoid': False}))
    return obstacles

def create_FAP_problem(fap, obstacles_draw, obstacles_avoid):

    environment = Environment(room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})

    # 2. create environment

    environment = Environment(room={'shape': Cuboid(10., 3.7, 2.3), 'position': [5., -0.35, 1.15]})

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
        # environment.add_obstacle(Obstacle({'position': pos_obst}, shape_obst, {}, {'draw': False}))

    # create obstacles for drawing
    for obst in obstacles_draw:
        pos_obst = [obst['pos'][k] + 0.5*obst['dim'][k] for k in range(3)]
        shape_obst = Cuboid(obst['dim'][0], obst['dim'][1], obst['dim'][2])
        environment.add_obstacle(Obstacle({'position': pos_obst}, shape_obst, {}, {'avoid': False}))

    # 3. create a point-to-point problem

    problem = Point2point(fap, environment, freeT=True)
    problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}, 'horizon_time': 10.})
    problem.init()
    return problem
