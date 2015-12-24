from admm import ADMMProblem
from point2point import Point2point
import numpy as np


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

    def get_interaction_error(self):
        error = 0.
        for veh in self.vehicles:
            n_samp = veh.path['y'].shape[2]
            end_time = veh.path['time'][-1]
            Ts = veh.options['sample_time']
            rp = self.fleet.get_rel_pos(veh)
            y_veh = veh.path['y'][:, 0, :]
            nghbs = self.fleet.get_neighbors(veh)
            for nghb in nghbs:
                y_nghb = nghb.path['y'][:, 0, :]
                Dy = y_veh - y_nghb
                err_rel = np.zeros(n_samp)
                for k in range(n_samp):
                    Dy_nrm = np.linalg.norm(Dy[:, k])
                    rp_nrm = np.linalg.norm(rp[nghb])
                    err_rel[k] = ((Dy_nrm - rp_nrm)/rp_nrm)**2
                err_rel_int = 0.
                for k in range(n_samp-1):
                    err_rel_int += 0.5*(err_rel[k+1] + err_rel[k])*Ts
                error += np.sqrt(err_rel_int/end_time)/len(nghbs)
        error /= self.fleet.N
        return error

    def final(self):
        ADMMProblem.final(self)
        err = self.get_interaction_error()
        print '%-18s %6g' % ('Formation error:', err*100.)
