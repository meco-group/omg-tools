from vehicle import *

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
    def __init__(self, vehicles = [], interconnection = 'circular'):
        self.vehicles = vehicles if isinstance(vehicles, list) else [vehicles]
        self.interconnection = interconnection
        self.setNeighbors()

    def addVehicle(self, vehicles):
        self.vehicles.extend(vehicles)
        self.setNeighbors()

    def getNeighbors(self, l):
        return self.nghb_list[l]

    def setNeighbors(self):
        self.N = len(self.vehicles)
        self.nghb_list = []
        for l in range(self.N):
            if self.interconnection == 'circular':
                nghb_ind    = [(self.N+l+1)%self.N, (self.N+l-1)%self.N]
            if self.interconnection == 'full':
                nghb_ind    = [k for k in range(self.N) if k!= l]
            self.nghb_list.append(nghb_ind)
        self.configuration  = [np.zeros(veh.n_y) for veh in self.vehicles]

    def getVehicleTypes(self):
        vehicle_types  = {}
        for l in range(self.N):
            typename = self.vehicles[l].__class__.__name__
            if typename in vehicle_types:
                vehicle_types[typename].append(l)
            else:
                vehicle_types[typename] = [l]
        return vehicle_types

    def setConfiguration(self, **kwargs):
        if 'polyhedron' in kwargs:
            self.configuration = [kwargs['polyhedron'].vertices[:,l] for l in range(self.N)]
        if 'points' in kwargs:
            self.configuration = kwargs['points']
        self.rel_pos = [[self.configuration[l] - self.configuration[nghb_ind] for nghb_ind in self.getNeighbors(l)] for l in range(self.N)]

    def getRelPos(self, l):
        return self.rel_pos[l]

    def setInitialPosition(self, position):
        if isinstance(position,list) and len(position) == self.N:
            positions = position
        else:
            positions = [position+self.configuration[l] for l in range(self.N)]
        for l in range(self.N):
            self.vehicles[l].setInitialPosition(positions[l])

    def setTerminalPosition(self, position):
        if isinstance(position,list) and len(position) == self.N:
            positions = position
        else:
            positions = [position+self.configuration[l] for l in range(self.N)]
        for l in range(self.N):
            self.vehicles[l].setTerminalPosition(positions[l])
