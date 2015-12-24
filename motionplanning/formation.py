from admm import ADMMProblem
from point2point import Point2point


class FormationPoint2point(ADMMProblem):

    def __init__(self, fleet, environment, options={}):
        self.environment = environment
        problems = [Point2point(vehicle, environment, options)
                    for vehicle in fleet.vehicles]
        ADMMProblem.__init__(self, problems, options)
        self.fleet = fleet

        # terminal constraints (stability issue)
        for veh in self.vehicles:
            y = veh.get_variable('y')
            for k in range(veh.n_y):
                for d in range(1, veh.degree+1):
                    self.define_constraint(y[k].derivative(d)(1.), 0., 0.)

        # formation constraints
        _couples = {veh: [] for veh in self.vehicles}
        for veh in self.vehicles:
            rp = self.fleet.get_rel_pos(veh)
            for nghb in self.fleet.get_neighbors(veh):
                if veh not in _couples[nghb]:
                    _couples[veh].append(nghb)
                    y_veh = veh.get_variable('y')
                    y_nghb = nghb.get_variable('y')
                    rel_pos = rp[nghb]
                    for k in range(veh.n_y):
                        self.define_constraint(
                            y_veh[k] - y_nghb[k] - rel_pos[k], 0., 0.)

