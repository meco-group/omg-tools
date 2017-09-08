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

from problem import Problem

class Problem_HolonomicF(Problem):

    def __init__(self, fleet, environment, options=None, label='problem'):
        Problem.__init__(self, fleet, environment, options, label='problem')

    # ========================================================================
    # Plot related functions
    # ========================================================================
    def init_plot(self, argument, **kwargs):
        print '#############################################################'
        if argument == 'scene':
            if not hasattr(self.vehicles[0], 'signals'):
                return None
            info = self.environment.init_plot(None, **kwargs)
            labels = kwargs['labels'] if 'labels' in kwargs else [
                '' for _ in range(self.environment.n_dim)]
            n_colors = len(self.colors)
            indices = [int([''.join(g) for _, g in groupby(
                v.label, str.isalpha)][-1]) % n_colors for v in self.vehicles]
            for v in range(len(self.vehicles)):
                info[0][0]['lines'].append(
                    {'linestyle': '-', 'color': self.colors_w[indices[v]]})

            #2 more lines in total, same color for the centralized payload problem###########
            for vehicle in self.vehicles:
                info[0][0]['lines'].append(
                        {'linestyle': '-', 'color': self.colors_w[indices[v]]})
                info[0][0]['lines'].append(
                        {'linestyle': '-', 'color': self.colors_w[indices[v]]})
            ##################################################

            #Extra lines in scene plot for payload lines##############
            for v,vehicle in enumerate(self.vehicles):
                info[0][0]['lines'].append(
                    {'linewidth': 0.9,'linestyle': ':', 'color': self.colors_w[indices[v]]}) #Future part
                info[0][0]['lines'].append({'linewidth': 0.9,'linestyle': '--','color': self.colors[indices[v]]}) #Past part
            ##################################################

            #Extra lines in scene plot for Neighbor 1 expectation lines##############
            for v,vehicle in enumerate(self.vehicles):
                info[0][0]['lines'].append(
                    {'linewidth': 0.7,'linestyle': '-.', 'color': self.colors_w[indices[v]]}) #Future part
                info[0][0]['lines'].append({'linewidth': 0.8, 'linestyle': '-.','color': self.colors[indices[v]]}) #Past part
            ##################################################

            #Extra lines in scene plot for Neighbor 2 expectation lines##############
            for v,vehicle in enumerate(self.vehicles):
                info[0][0]['lines'].append(
                    {'linewidth': 0.7,'linestyle': '-.', 'color': self.colors_w[indices[v]]}) #Future part
                info[0][0]['lines'].append({'linewidth': 0.8, 'linestyle': '-.','color': self.colors[indices[v]]}) #Past part
            ##################################################

            for v, vehicle in enumerate(self.vehicles):
                info[0][0]['lines'].append({'color': self.colors[indices[v]]})

            for v, vehicle in enumerate(self.vehicles):
            #With Added shapes for payload and springs
                for _ in vehicle.draw_HolonomicF():
                    info[0][0]['lines'].append({'linewidth': 2,'color': self.colors[indices[v]]})
            #######################################

            info[0][0]['labels'] = labels
            return info
        else:
            return None

    def update_plot(self, argument, t, **kwargs):
        if argument == 'scene':
            if not hasattr(self.vehicles[0], 'signals'):
                return None
            data = self.environment.update_plot(None, t, **kwargs)
            for vehicle in self.vehicles:
                    indices_to_plot = [2,10]
                    payload_indices = [0,8]
                    #length = np.sqrt(np.power(vehicle.traj_storage['pose'][t][indices_to_plot[0], :]-vehicle.traj_storage['pose'][t][payload_indices[0], :],2) + np.power(vehicle.traj_storage['pose'][t][indices_to_plot[1], :]-vehicle.traj_storage['pose'][t][payload_indices[1], :],2))
                    #x_pos_new = vehicle.traj_storage['pose'][t][indices_to_plot[0], :] + vehicle.Lfree*(vehicle.traj_storage['pose'][t][indices_to_plot[0], :] - vehicle.traj_storage['pose'][t][payload_indices[0], :])/length
                    #y_pos_new = vehicle.traj_storage['pose'][t][indices_to_plot[1], :] + vehicle.Lfree*(vehicle.traj_storage['pose'][t][indices_to_plot[1], :] - vehicle.traj_storage['pose'][t][payload_indices[1], :])/length
                    data[0][0].append(
                        [vehicle.traj_storage['pose'][t][indices_to_plot[k], :] for k in range(vehicle.n_dim)])
                    #data[0][0].append([x_pos_new,y_pos_new])

            #Plot what each vehicle thinks of the payload#################################
            payload_indices = [0,8]
            for vehicle in self.vehicles:
                data[0][0].append(
                    [vehicle.traj_storage['pose'][t][payload_indices[k], :] for k in range(vehicle.n_dim)])
                if t == -1:
                    data[0][0].append(
                        [vehicle.signals['pose'][payload_indices[k], :] for k in range(vehicle.n_dim)])
                else:
                    data[0][0].append(
                        [vehicle.signals['pose'][payload_indices[k], :t+1] for k in range(vehicle.n_dim)])
            ########################################################################

            #Plot what each vehicle thinks of the Neighbor 1#################################
            neigh_j_indices = [4,12]
            #length = np.sqrt(np.power(vehicle.traj_storage['pose'][t][neigh_j_indices[0], :]-vehicle.traj_storage['pose'][t][payload_indices[0], :],2) + np.power(vehicle.traj_storage['pose'][t][neigh_j_indices[1], :]-vehicle.traj_storage['pose'][t][payload_indices[1], :],2))
            #x_pos_new = vehicle.traj_storage['pose'][t][neigh_j_indices[0], :] + vehicle.Lfree*(vehicle.traj_storage['pose'][t][neigh_j_indices[0], :] - vehicle.traj_storage['pose'][t][payload_indices[0], :])/length
            #y_pos_new = vehicle.traj_storage['pose'][t][neigh_j_indices[1], :] + vehicle.Lfree*(vehicle.traj_storage['pose'][t][neigh_j_indices[1], :] - vehicle.traj_storage['pose'][t][payload_indices[1], :])/length
            for vehicle in self.vehicles:
                data[0][0].append(
                    [vehicle.traj_storage['pose'][t][neigh_j_indices[k], :] for k in range(vehicle.n_dim)])
                #data[0][0].append([x_pos_new,y_pos_new])
                if t == -1:
                    data[0][0].append(
                        [vehicle.signals['pose'][neigh_j_indices[k], :] for k in range(vehicle.n_dim)])
                else:
                    data[0][0].append(
                        [vehicle.signals['pose'][neigh_j_indices[k], :t+1] for k in range(vehicle.n_dim)])
            ########################################################################

            #Plot what each vehicle thinks of the Neighbor 2#################################
            neigh_k_indices = [6,14]
            #length = np.sqrt(np.power(vehicle.traj_storage['pose'][t][neigh_k_indices[0], :]-vehicle.traj_storage['pose'][t][payload_indices[0], :],2) + np.power(vehicle.traj_storage['pose'][t][neigh_k_indices[1], :]-vehicle.traj_storage['pose'][t][payload_indices[1], :],2))
            #x_pos_new = vehicle.traj_storage['pose'][t][neigh_k_indices[0], :] + vehicle.Lfree*(vehicle.traj_storage['pose'][t][neigh_k_indices[0], :] - vehicle.traj_storage['pose'][t][payload_indices[0], :])/length
            #y_pos_new = vehicle.traj_storage['pose'][t][neigh_k_indices[1], :] + vehicle.Lfree*(vehicle.traj_storage['pose'][t][neigh_k_indices[1], :] - vehicle.traj_storage['pose'][t][payload_indices[1], :])/length
            for vehicle in self.vehicles:
                data[0][0].append(
                    [vehicle.traj_storage['pose'][t][neigh_k_indices[k], :] for k in range(vehicle.n_dim)])
                #data[0][0].append([x_pos_new,y_pos_new])
                if t == -1:
                    data[0][0].append(
                        [vehicle.signals['pose'][neigh_k_indices[k], :] for k in range(vehicle.n_dim)])
                else:
                    data[0][0].append(
                        [vehicle.signals['pose'][neigh_k_indices[k], :t+1] for k in range(vehicle.n_dim)])
            ########################################################################

            for vehicle in self.vehicles:
                indices_to_plot = [2,10]
                payload_indices = [0,8]
                length = np.sqrt(np.power(vehicle.signals['pose'][indices_to_plot[0], :]-vehicle.signals['pose'][payload_indices[0], :],2) + np.power(vehicle.signals['pose'][indices_to_plot[1], :]-vehicle.signals['pose'][payload_indices[1], :],2))
                x_pos_new = vehicle.signals['pose'][indices_to_plot[0], :] + vehicle.Lfree*(vehicle.signals['pose'][indices_to_plot[0], :] - vehicle.signals['pose'][payload_indices[0], :])/length
                y_pos_new = vehicle.signals['pose'][indices_to_plot[1], :] + vehicle.Lfree*(vehicle.signals['pose'][indices_to_plot[1], :] - vehicle.signals['pose'][payload_indices[1], :])/length
                if t == -1:
                    #data[0][0].append([x_pos_new,y_pos_new])
                    data[0][0].append(
                    [vehicle.signals['pose'][indices_to_plot[k], :] for k in range(vehicle.n_dim)])
                else:
                    #print '##################################'
                    #print vehicle.signals['pose'][indices_to_plot[0], :t+1]
                    #print vehicle.signals['pose'][indices_to_plot[1], :t+1]
                    #print '##################################'
                    data[0][0].append(
                    [vehicle.signals['pose'][indices_to_plot[k], :t+1] for k in range(vehicle.n_dim)])

            for vehicle in self.vehicles:
                ##Extra lines for HolonomicF
                for l in vehicle.draw_HolonomicF(t):
                    data[0][0].append([l[k, :] for k in range(vehicle.n_dim)])

            return data
        else:
            return None
