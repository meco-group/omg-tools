# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from .admm import ADMMProblem
from .point2point import FreeEndPoint2point
from ..export.export_rendezvous import ExportRendezVous
import numpy as np


class RendezVous(ADMMProblem):

    def __init__(self, fleet, environment, options=None):
        problems = []
        for veh in fleet.vehicles:
            free_ind = list(fleet.configuration[veh].keys())
            problems.append(
                FreeEndPoint2point(veh, environment.copy(), options, {veh: free_ind}))
        ADMMProblem.__init__(self, fleet, environment, problems, options)

    def construct(self):
        config = self.fleet.configuration
        rel_pos_c = {}
        for veh in self.vehicles:
            ind_veh = sorted(config[veh].keys())
            rel_pos_c[veh] = veh.define_parameter('rel_pos_c', len(ind_veh))
        problems_dic = {veh: self.problems[l] for l, veh in enumerate(self.fleet.vehicles)}
        # get fleet center as seen by vehicles
        centra = {}
        for veh in self.vehicles:
            conT = problems_dic[veh].define_symbol('conT0', len(problems_dic[veh].free_ind[veh]))
            centra[veh] = veh.get_fleet_center([conT], [rel_pos_c[veh]])[0]
        # rendez-vous constraints
        couples = {veh: [] for veh in self.vehicles}
        for veh in self.vehicles:
            ind_veh = sorted(config[veh].keys())
            pos_c_veh = centra[veh]
            for nghb in self.fleet.get_neighbors(veh):
                if veh not in couples[nghb] and nghb not in couples[veh]:
                    couples[veh].append(nghb)
                    ind_nghb = sorted(config[nghb].keys())
                    pos_c_nghb = centra[nghb]
                    for ind_v, ind_n in zip(ind_veh, ind_nghb):
                        self.define_constraint(pos_c_veh[ind_v] - pos_c_nghb[ind_n], 0., 0.)
        ADMMProblem.construct(self)

    def set_parameters(self, current_time):
        parameters = {}
        for veh in self.vehicles:
            parameters[veh] = {'rel_pos_c': veh.rel_pos_c}
        return parameters

    def stop_criterium(self, current_time, update_time):
        res = 0.
        for veh in self.vehicles:
            ind_veh = sorted(self.fleet.configuration[veh].keys())
            rel_conT = self.fleet.get_rel_config(veh)
            for nghb in self.fleet.get_neighbors(veh):
                ind_nghb = sorted(self.fleet.configuration[nghb].keys())
                for k, (ind_v, ind_n) in enumerate(zip(ind_veh, ind_nghb)):
                    rcT = rel_conT[nghb]
                    rcT = rcT if isinstance(rcT, float) else rcT[k]
                    res += np.linalg.norm(veh.trajectories['splines'][ind_v, 0] -
                                          nghb.trajectories['splines'][ind_n, 0] -
                                          rcT)**2
        if np.sqrt(res) > 5.e-2:
            return False
        return True

    def export(self, options=None):
        options = options or {}
        if not hasattr(self, 'father'):
            self.init()
        ExportRendezVous(self, options)
