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

from shape import Circle
from shape import RegularPolyhedron
import numpy as np

class shape_HolonomicF(Circle):

    def __init__(self, radius):
        Circle.__init__(self, radius)

    #############################################
    def draw_HolonomicF(self, pose=np.zeros(3)):
        Lfree = 0.42
        veh_x = pose[2]
        veh_y = pose[10]
        payload_x = pose[0]
        payload_y = pose[8]
        length = np.sqrt(np.power(veh_x-payload_x,2) + np.power(veh_y-payload_y,2))
        x_pos_new = veh_x + Lfree*(veh_x - payload_x)/length
        y_pos_new = veh_y + Lfree*(veh_y - payload_y)/length
                    #data[0][0].append(
        return [np.c_[[pose[2],pose[10]]] + line for line in self.plt_lines]
        #return [np.c_[[x_pos_new,y_pos_new]] + line for line in self.plt_lines]

    def draw_HolonomicF_payload(self, pose=np.zeros(3)):
        self = RegularPolyhedron(radius = 0.1, n_vert =4)
        return [np.c_[[pose[0],pose[8]]] + line for line in self.plt_lines]
    #############################################
