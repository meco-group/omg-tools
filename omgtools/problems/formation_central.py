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
        config = self.fleet.configuration
        rel_pos_c = {}
        for veh in self.vehicles:
            ind_veh = sorted(config[veh].keys())
            rel_pos_c[veh] = veh.define_parameter('rel_pos_c', len(ind_veh))
        FixedTPoint2point.construct(self)
        # get formation center as seen by vehicles
        centra = {}
        for veh in self.vehicles:
            splines = self.father.get_variables(veh, 'splines0', symbolic=True)
            ind_veh = sorted(config[veh].keys())
            centra[veh] = veh.get_fleet_center(splines, rel_pos_c[veh], substitute=False)
        # formation constraints
        couples = {veh: [] for veh in self.vehicles}
        for veh in self.vehicles:
            for nghb in self.fleet.get_neighbors(veh):
                if veh not in couples[nghb] and nghb not in couples[veh]:
                    couples[veh].append(nghb)
        if self.fleet.interconnection == 'circular':
            couples.pop(self.vehicles[-1], None)
            couples.pop(self.vehicles[-2], None)
        for veh, nghbs in couples.items():
            ind_veh = sorted(config[veh].keys())
            pos_c_veh = centra[veh]
            for nghb in nghbs:
                ind_nghb = sorted(config[nghb].keys())
                pos_c_nghb = centra[nghb]
                for ind_v, ind_n in zip(ind_veh, ind_nghb):
                    self.define_constraint(pos_c_veh[ind_v] - pos_c_nghb[ind_n], 0., 0.)

    def update(self, current_time, update_time, sample_time):
        FixedTPoint2point.update(self, current_time, update_time, sample_time)
        self.fleet.update_plots()
