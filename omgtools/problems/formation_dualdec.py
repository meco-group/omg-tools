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

from .dualdecomposition import DDProblem
from .point2point import Point2point
import numpy as np


class FormationPoint2pointDualDecomposition(DDProblem):

    def __init__(self, fleet, environment, options=None):
        problems = [Point2point(vehicle, environment.copy(), options)
                    for vehicle in fleet.vehicles]
        DDProblem.__init__(self, fleet, environment, problems, options)

    def construct(self):
        config = self.fleet.configuration
        rel_pos_c = {}
        for veh in self.vehicles:
            ind_veh = sorted(config[veh].keys())
            rel_pos_c[veh] = veh.define_parameter('rel_pos_c', len(ind_veh))
        # get formation center as seen by vehicles
        centra = {}
        splines = {}
        for veh in self.vehicles:
            splines[veh] = veh.define_spline_symbol('splines_seg0', veh.n_spl)
            ind_veh = sorted(config[veh].keys())
            centra[veh] = veh.get_fleet_center(splines[veh], rel_pos_c[veh])
        # formation constraints
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
        # terminal constraints (stability issue)
        for veh in self.vehicles:
            for spline in centra[veh]:
                for d in range(1, spline.basis.degree+1):
                    # constraints imposed on distributedproblem instance will be
                    # invoked on the z-variables (because it is interpreted as
                    # 'interconnection constraint')
                    self.define_constraint(spline.derivative(d)(1.), 0., 0.)
        DDProblem.construct(self)

    def set_parameters(self, current_time):
        parameters = {}
        for veh in self.vehicles:
            parameters[veh] = {'rel_pos_c': veh.rel_pos_c}
        return parameters

    def get_interaction_error(self):
        # compute average deviation of an agent's fleet center wrt to the real center
        pos_c_veh = []
        center_veh = []
        rel_pos_c = []
        for veh in self.vehicles:
            pos_c_veh.append(veh.signals['fleet_center'])
            center_veh.append(veh.signals['pose'][:veh.n_dim])
            config = self.fleet.configuration[veh]
            ind_veh = sorted(config.keys())
            rel_pos_c.append(np.array([config[ind_v] for ind_v in ind_veh]))
            n_samp = veh.signals['time'].shape[1]
            end_time = veh.signals['time'][:, -1]
            Ts = veh.signals['time'][0, 1] - veh.signals['time'][0, 0]
        center = np.zeros((center_veh[0].shape))
        for k in range(n_samp):
            for l in range(center_veh[0].shape[0]):
                center[l, k] = np.mean([pos[l, k] for pos in center_veh])
        error = np.zeros(n_samp)
        for pos_c, rp_c in zip(pos_c_veh, rel_pos_c):
            deviation = center - pos_c
            for k in range(n_samp):
                error[k] += np.linalg.norm(deviation[:, k])/np.linalg.norm(rp_c)
        error /= self.fleet.N
        error_int = 0.
        for k in range(n_samp-1):
            error_int += 0.5*(error[k+1]+error[k])*Ts
        return error_int/end_time

    def final(self):
        DDProblem.final(self)
        err = self.get_interaction_error()
        print('%-18s %6g %%' % ('Formation error:', err))
