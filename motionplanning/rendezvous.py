from admm import ADMMProblem
from point2point import Point2point
import numpy as np
import copy


class RendezVous(ADMMProblem):

    def __init__(self, fleet, environment, options={}):
        self.environment = environment
        if 'fixed_yT' in options:
            fxd_yT = options['fixed_yT']
        else:
            fxd_yT = [[] for veh in fleet.vehicles]
        problems = []
        for l, veh in enumerate(fleet.vehicles):
            opt = copy.deepcopy(options)
            opt.update({'fixed_yT': [fxd_yT[l]]})
            problems.append(Point2point(veh, environment, opt))

        ADMMProblem.__init__(self, problems, options)
        problems_dic = {veh: problems[l]
                        for l, veh in enumerate(fleet.vehicles)}
        self.fleet = fleet

        # define parameters
        rel_poses = {veh: self.define_parameter(
            'rp_'+str(l), veh.n_y-len(fxd_yT[l]),
            len(self.fleet.get_neighbors(veh)))
        for l, veh in enumerate(self.vehicles)}

        # end pose constraints
        _couples = {veh: [] for veh in self.vehicles}
        self.free_yT = {veh: [] for veh in self.vehicles}
        for l, veh in enumerate(self.vehicles):
            for k in range(veh.n_y):
                if k not in fxd_yT[l]:
                    self.free_yT[veh].append(k)
            if l == 0:
                self.n_free = len(self.free_yT[veh])
            else:
                if self.n_free != len(self.free_yT[veh]):
                    raise ValueError(('Number of free entries of yT '
                                      'should be equal for all vehicles!'))
        for p, veh in enumerate(self.vehicles):
            rp = rel_poses[veh]
            for l, nghb in enumerate(self.fleet.get_neighbors(veh)):
                if veh not in _couples[nghb]:
                    _couples[veh].append(nghb)
                    yT_veh = problems_dic[veh].get_variable('yTv_0')
                    yT_nghb = problems_dic[nghb].get_variable('yTv_0')
                    rel_pos = rp[:, l]
                    for k in range(self.n_free):
                        ind_veh = self.free_yT[veh][k]
                        ind_nghb = self.free_yT[nghb][k]
                        self.define_constraint(
                            yT_veh[ind_veh] - yT_nghb[ind_nghb] - rel_pos[k],
                            0., 0.)

    def set_parameters(self, current_time):
        parameters = {}
        for l, veh in enumerate(self.vehicles):
            rel_pos, rp_ = self.fleet.get_rel_pos(veh), []
            for nghb in self.fleet.get_neighbors(veh):
                rp_.append(np.c_[rel_pos[nghb]])
            parameters['rp_'+str(l)] = np.hstack(rp_)
        return parameters

    def stop_criterium(self):
        res = 0.
        for veh in self.vehicles:
            rel_pos = self.fleet.get_rel_pos(veh)
            for nghb in self.fleet.get_neighbors(veh):
                for k in range(self.n_free):
                    ind_veh = self.free_yT[veh][k]
                    ind_nghb = self.free_yT[nghb][k]
                    rp = rel_pos[nghb]
                    rp = rp if isinstance(rp, float) else rp[k]
                    res += np.linalg.norm(veh.trajectory['y'][ind_veh, 0, 0]-
                                          nghb.trajectory['y'][ind_nghb, 0, 0]-
                                          rp)**2
        if np.sqrt(res) > 5.e-2:
            return False
        return True
