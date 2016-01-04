from vehicle import Vehicle
import numpy as np


def get_fleet_vehicles(var):
    if isinstance(var, Fleet):
        return var, var.vehicles
    elif isinstance(var, list):
        if isinstance(var[0], Vehicle):
            return Fleet(var), var
        if isinstance(var[0], Fleet):
            return var[0], var[0].vehicles
    elif isinstance(var, Vehicle):
        return Fleet(var), [var]


class Fleet:

    def __init__(self, vehicles=[], interconnection='circular'):
        self.vehicles = vehicles if isinstance(vehicles, list) else [vehicles]
        self.interconnection = interconnection
        self.set_neighbors()

    def add_vehicle(self, vehicles):
        self.vehicles.extend(vehicles)
        self.set_neighbors()

    def get_neighbors(self, vehicle):
        return self.nghb_list[vehicle]

    def set_neighbors(self):
        self.N = len(self.vehicles)
        self.nghb_list = {}
        for l, vehicle in enumerate(self.vehicles):
            if self.interconnection == 'circular':
                nghb_ind = [(self.N+l+1) % self.N, (self.N+l-1) % self.N]
            if self.interconnection == 'full':
                nghb_ind = [k for k in range(self.N) if k != l]
            self.nghb_list[vehicle] = [self.vehicles[ind] for ind in nghb_ind]
        self.configuration = [np.zeros(veh.n_y) for veh in self.vehicles]

    def set_configuration(self, **kwargs):
        if 'polyhedron' in kwargs:
            poly = kwargs['polyhedron']
            if self.N != poly.n_faces:
                raise ValueError('Configuration polyhedron shape should have '
                                 'as many vertices as vehicles in the fleet!')
            self.configuration = {
                veh: poly.vertices[:, l] for l, veh in enumerate(self.vehicles)}
        if 'points' in kwargs:
            points = kwargs['points']
            self.configuration = {veh: points[l]
                                  for l, veh in enumerate(self.vehicles)}
        self.rel_pos = {}
        for veh in self.vehicles:
            self.rel_pos[veh] = {}
            for nghb in self.get_neighbors(veh):
                self.rel_pos[veh][nghb] = self.configuration[
                    veh] - self.configuration[nghb]

    def get_rel_pos(self, vehicle):
        return self.rel_pos[vehicle]

    def set_initial_pose(self, pose):
        if isinstance(pose, list) and len(pose) == self.N:
            poses = pose
        else:
            poses = [pose+self.configuration[veh] for veh in self.vehicles]
        for l, veh in enumerate(self.vehicles):
            veh.set_initial_pose(poses[l])

    def set_terminal_pose(self, pose):
        if isinstance(pose, list) and len(pose) == self.N:
            poses = pose
        else:
            poses = [pose+self.configuration[veh] for veh in self.vehicles]
        for l, veh in enumerate(self.vehicles):
            veh.set_terminal_pose(poses[l])
