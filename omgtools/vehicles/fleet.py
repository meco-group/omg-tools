from vehicle import Vehicle


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
            elif self.interconnection == 'full':
                nghb_ind = [k for k in range(self.N) if k != l]
            else:
                raise ValueError('Interconnection type ' + self.interconnection +
                                 ' not understood.')
            self.nghb_list[vehicle] = [self.vehicles[ind] for ind in nghb_ind]

    def set_configuration(self, configuration):
        self.configuration = {}
        if len(configuration) != self.N:
            raise ValueError('You should provide configuration info ' +
                             'for each vehicle.')
        for l, config in enumerate(configuration):
            if isinstance(config, dict):
                self.configuration[self.vehicles[l]] = config
            if isinstance(config, list):
                self.configuration[self.vehicles[l]] = {
                    k: con for k, con in enumerate(config)}
        self.rel_config = {}
        for vehicle in self.vehicles:
            self.rel_config[vehicle] = {}
            ind_veh = sorted(self.configuration[vehicle].keys())
            for nghb in self.get_neighbors(vehicle):
                self.rel_config[vehicle][nghb] = []
                ind_nghb = sorted(self.configuration[nghb].keys())
                if len(ind_veh) != len(ind_nghb):
                    raise ValueError('All vehicles should have same number ' +
                                     'of variables for which the configuration ' +
                                     'is imposed.')
                for k in range(len(ind_veh)):
                    self.rel_config[vehicle][nghb].append(
                        self.configuration[vehicle][ind_veh[k]] -
                        self.configuration[nghb][ind_nghb[k]])

    def get_rel_config(self, vehicle):
        return self.rel_config[vehicle]

    def set_initial_conditions(self, conditions):
        for l, vehicle in enumerate(self.vehicles):
            vehicle.set_initial_conditions(conditions[l])

    def set_terminal_conditions(self, conditions):
        for l, vehicle in enumerate(self.vehicles):
            vehicle.set_terminal_conditions(conditions[l])
