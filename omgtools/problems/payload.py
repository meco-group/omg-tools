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

from admm_payload import ADMMProblem
from distributedPayload import DistributedPayload
from ..export.export_payload import ExportPayloadTransport
import numpy as np


class PayloadTransport(ADMMProblem):

    def __init__(self, fleet, environment, options=None):
        problems = [DistributedPayload(vehicle, environment.copy(), options)
                    for vehicle in fleet.vehicles]
        #For each vehicle, all this is anyway required -> Variables, Parameters, Constraints, etc
        ADMMProblem.__init__(self, fleet, environment, problems, options)

    def construct(self):
        ADMMProblem.construct(self)

        ################################
        #Constraints for Consensus
        #Make [-inf,inf] constraint on x11-x22 -> This will introduce copies of them on z -> Used for cummunication
        neighbor_j_copies = {}
        neighbor_k_copies = {}
        home_spline_x = {}
        home_spline_y = {}
        for veh in self.vehicles:
            splines = self.father.get_variables(veh, 'splines0', symbolic=True)
            neighbor_j_copies[veh], neighbor_k_copies[veh], home_spline_x[veh], home_spline_y[veh] = veh.get_spline_copies(splines)
            self.define_constraint(neighbor_j_copies[veh][0], -np.inf, np.inf)
            self.define_constraint(neighbor_j_copies[veh][1], -np.inf, np.inf)
            self.define_constraint(neighbor_k_copies[veh][0], -np.inf, np.inf)
            self.define_constraint(neighbor_k_copies[veh][1], -np.inf, np.inf)

        couples = {veh: [] for veh in self.vehicles}

        for veh in self.vehicles:
            enum = 0
            for nghb in self.fleet.get_neighbors(veh):
                if enum==0:
                    self.define_constraint(home_spline_x[veh]-home_spline_x[nghb], -np.inf, np.inf)
                    self.define_constraint(home_spline_y[veh]-home_spline_y[nghb], -np.inf, np.inf)
                else:
                    self.define_constraint(home_spline_x[veh]-home_spline_x[nghb], -np.inf, np.inf)
                    self.define_constraint(home_spline_y[veh]-home_spline_y[nghb], -np.inf, np.inf)
                enum+=1

    ###############################################################################################

     ###############################################################################################
    def update_payload(self,current_time,update_time,sample_time):
        horizon_time = self.options['horizon_time']
        if self.problems[0].init_time is None:
            # y_coeffs represents coefficients of a spline, for which a part of
            # its time horizon lies in the past. Therefore, we need to pass the
            # current time relatively to the begin of this time horizon. In this
            # way, only the future, relevant, part will be saved/plotted.
            rel_current_time = np.round(current_time-self.problems[0].start_time, 6) % self.problems[0].knot_time
        else:
            rel_current_time = self.problems[0].init_time
        if horizon_time - rel_current_time < update_time:
            update_time = horizon_time - rel_current_time
        for problem in self.problems:
            problem.compute_partial_objective(current_time, update_time)

        n_samp = int(round((horizon_time-rel_current_time)/sample_time, 3)) + 1
        time_axis = np.linspace(rel_current_time, rel_current_time + (n_samp-1)*sample_time, n_samp)
        spline_segments = {}
        for problem in self.problems:
            for vehicle in problem.vehicles:
                spline_segments[str(vehicle)] = [problem.father.get_variables(vehicle, 'splines'+str(k)) for k in range(vehicle.n_seg)][0]
                #We can pick the updater from any of the vehicles. We should pass all the splines to it
                vehicle.update_payload(current_time, update_time, sample_time, spline_segments[str(vehicle)], horizon_time, time_axis)
         ###############################################################################################

    def final(self):
        ADMMProblem.final(self)

    def export(self, options=None):
        options = options or {}
        if not hasattr(self, 'father'):
            self.init()
        ExportPayloadTransport(self, options)
