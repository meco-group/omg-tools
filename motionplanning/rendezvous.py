from admm import ADMMProblem
from point2point import Point2point
import numpy as np


class RendezVous(ADMMProblem):

    def __init__(self, fleet, environment, options={}):
        self.environment = environment
        options.update({'fixed_yT': [[] for veh in fleet.vehicles]})
        problems = [Point2point(vehicle, environment, options)
                    for vehicle in fleet.vehicles]
        ADMMProblem.__init__(self, problems, options)
        problems_dic = {veh: problems[l] for l, veh in enumerate(fleet.vehicles)}
        self.fleet = fleet

        # define parameters
        rel_poses = {veh: self.define_parameter('rp_'+str(l), veh.n_y, len(self.fleet.get_neighbors(veh))) for l, veh in enumerate(self.vehicles)}

        # end pose constraints
        _couples = {veh: [] for veh in self.vehicles}
        for p, veh in enumerate(self.vehicles):
            rp = rel_poses[veh]
            for l, nghb in enumerate(self.fleet.get_neighbors(veh)):
                if veh not in _couples[nghb]:
                    _couples[veh].append(nghb)
                    yT_veh = problems_dic[veh].get_variable('yTv_0')
                    yT_nghb = problems_dic[nghb].get_variable('yTv_0')
                    rel_pos = rp[:, l]
                    for k in range(veh.n_y):
                        self.define_constraint(
                            yT_veh[k] - yT_nghb[k] - rel_pos[k], 0., 0.)

    def set_parameters(self, current_time):
        parameters = {}
        for l, veh in enumerate(self.vehicles):
            rel_pos, rp_ = self.fleet.get_rel_pos(veh), []
            for nghb in self.fleet.get_neighbors(veh):
                rp_.append(np.c_[rel_pos[nghb]])
            parameters['rp_'+str(l)] = np.hstack(rp_)
        return parameters

    # def get_interaction_error(self):
    #     error = 0.
    #     for veh in self.vehicles:
    #         n_samp = veh.path['y'].shape[2]
    #         end_time = veh.path['time'][-1]
    #         Ts = veh.options['sample_time']
    #         rp = self.fleet.get_rel_pos(veh)
    #         y_veh = veh.path['y'][:, 0, :]
    #         nghbs = self.fleet.get_neighbors(veh)
    #         for nghb in nghbs:
    #             y_nghb = nghb.path['y'][:, 0, :]
    #             Dy = y_veh - y_nghb
    #             err_rel = np.zeros(n_samp)
    #             for k in range(n_samp):
    #                 Dy_nrm = np.linalg.norm(Dy[:, k])
    #                 rp_nrm = np.linalg.norm(rp[nghb])
    #                 err_rel[k] = ((Dy_nrm - rp_nrm)/rp_nrm)**2
    #             err_rel_int = 0.
    #             for k in range(n_samp-1):
    #                 err_rel_int += 0.5*(err_rel[k+1] + err_rel[k])*Ts
    #             error += np.sqrt(err_rel_int/end_time)/len(nghbs)
    #     error /= self.fleet.N
    #     return error

    # def final(self):
    #     ADMMProblem.final(self)
    #     err = self.get_interaction_error()
    #     print '%-18s %6g' % ('Formation error:', err*100.)
