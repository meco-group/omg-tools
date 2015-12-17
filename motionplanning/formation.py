from admm import ADMMProblem
from point2point import Point2point


class FormationPoint2point(ADMMProblem):

    def __init__(self, fleet, environment, options={}):
        problems = [Point2point(vehicle, environment, options)
                    for vehicle in fleet.vehicles]
        ADMMProblem.__init__(self, problems, options)

        y = [vehicle.get_variable('y') for vehicle in self.vehicles]

        Dx = [1., 2.]
        for k in range(self.vehicles[0].n_y):
            self.define_constraint(y[1][k] - y[0][k] - Dx[k], 0., 0.)
            self.define_constraint(y[2][k] - y[1][k] - Dx[k], 0., 0.)
            self.define_constraint(y[3][k] - y[2][k] - Dx[k], 0., 0.)
            self.define_constraint(y[0][k] - y[3][k] - Dx[k], 0., 0.)
