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

from point2point import FixedTPoint2point


class FormationPoint2pointCentral(FixedTPoint2point):

    def __init__(self, fleet, environment, options=None):
        FixedTPoint2point.__init__(self, fleet, environment, options)

    def construct(self):
        FixedTPoint2point.construct(self)
        # define parameters
        rel_splines = {veh: {nghb: self.define_parameter('rs'+str(l)+str(n), len(self.fleet.configuration[veh].keys())) for n, nghb in enumerate(self.fleet.get_neighbors(veh))} for l, veh in enumerate(self.vehicles)}
        # define constraints
        couples = {veh: [] for veh in self.vehicles}
        for veh in self.vehicles:
            ind_veh = sorted(self.fleet.configuration[veh].keys())
            rs = rel_splines[veh]
            for nghb in self.fleet.get_neighbors(veh):
                ind_nghb = sorted(self.fleet.configuration[nghb].keys())
                if veh not in couples[nghb] and nghb not in couples[veh]:
                    couples[veh].append(nghb)
                    spl_veh = self.father.get_variables(veh, 'splines0', symbolic=True)
                    spl_nghb = self.father.get_variables(nghb, 'splines0', symbolic=True)
                    for ind_v, ind_n, rel_spl in zip(ind_veh, ind_nghb, rs[nghb]):
                        self.define_constraint(
                            spl_veh[ind_v] - spl_nghb[ind_n] - rel_spl, 0., 0.)

    def set_parameters(self, current_time):
        parameters = FixedTPoint2point.set_parameters(self, current_time)
        for l, veh in enumerate(self.vehicles):
            rel_spl = self.fleet.get_rel_config(veh)
            for n, nghb in enumerate(self.fleet.get_neighbors(veh)):
                parameters['rs'+str(l)+str(n)] = rel_spl[nghb]
        return parameters

    def update(self, current_time, update_time, sample_time):
        FixedTPoint2point.update(self, current_time, update_time, sample_time)
        self.fleet.update_plots()
