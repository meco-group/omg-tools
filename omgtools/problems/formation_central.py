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
from ..basics.spline_extra import definite_integral
from casadi import inf


class FormationPoint2pointCentral(FixedTPoint2point):

    def __init__(self, fleet, environment, options=None):
        FixedTPoint2point.__init__(self, fleet, environment, options)

    def set_default_options(self):
        FixedTPoint2point.set_default_options(self)
        self.options['soft_formation'] = False
        self.options['soft_formation_weight'] = 10
        self.options['max_formation_deviation'] = inf

    def construct(self):
        config = self.fleet.configuration
        rel_pos_c = {}
        for veh in self.vehicles:
            ind_veh = sorted(config[veh].keys())
            rel_pos_c[veh] = veh.define_parameter('rel_pos_c', len(ind_veh))
        FixedTPoint2point.construct(self)
        # get formation center as seen by vehicles
        centra = {}
        splines = {}
        for veh in self.vehicles:
            splines[veh] = self.father.get_variables(veh, 'splines0', symbolic=True)
            ind_veh = sorted(config[veh].keys())
            centra[veh] = veh.get_fleet_center(splines[veh], rel_pos_c[veh], substitute=False)
        # formation constraints
        couples = {veh: [] for veh in self.vehicles}
        for veh in self.vehicles:
            for nghb in self.fleet.get_neighbors(veh):
                if veh not in couples[nghb] and nghb not in couples[veh]:
                    couples[veh].append(nghb)
        if self.fleet.interconnection == 'circular':
            couples.pop(self.vehicles[-1], None)
            couples.pop(self.vehicles[-2], None)

        t = self.define_symbol('t')
        T = self.define_symbol('T')

        for veh, nghbs in couples.items():
            ind_veh = sorted(config[veh].keys())
            pos_c_veh = centra[veh]
            for nghb in nghbs:
                ind_nghb = sorted(config[nghb].keys())
                pos_c_nghb = centra[nghb]
                for ind_v, ind_n in zip(ind_veh, ind_nghb):
                    if self.options['soft_formation']:
                        weight = self.options['soft_formation_weight']
                        eps = self.define_spline_variable('eps_form_'+str(ind_v)+str(ind_n), basis=veh.basis)[0]
                        obj = weight*definite_integral(eps, t/T, 1.)
                        self.define_objective(obj)
                        self.define_constraint(pos_c_veh[ind_v] - pos_c_nghb[ind_n] - eps, -inf, 0.)
                        self.define_constraint(-pos_c_veh[ind_v] + pos_c_nghb[ind_n] - eps, -inf, 0.)
                        if self.options['max_formation_deviation'] != inf:
                            max_dev = abs(self.options['max_formation_deviation'])
                            self.define_constraint(eps, -max_dev, max_dev)
                    else:
                        self.define_constraint(pos_c_veh[ind_v] - pos_c_nghb[ind_n], 0., 0.)

    def update(self, current_time, update_time, sample_time):
        FixedTPoint2point.update(self, current_time, update_time, sample_time)
        self.fleet.update_plots()

    def set_parameters(self, current_time):
        parameters = FixedTPoint2point.set_parameters(self, current_time)
        for veh in self.vehicles:
            if veh not in parameters:
                parameters[veh] = {}
            parameters[veh].update({'rel_pos_c': veh.rel_pos_c})
        return parameters
