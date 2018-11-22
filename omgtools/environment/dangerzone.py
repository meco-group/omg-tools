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


import numpy as np
from obstacle import Obstacle2D


class DangerZone(Obstacle2D):

    def __init__(self, initial, shape, bounds, simulation=None, options=None):
        simulation = simulation or {}
        options = options or {}
        
        self.set_default_options()
        self.set_options(options)

        Obstacle2D.__init__(self, initial, shape, simulation, options)
        self.bounds = bounds

    def draw(self, t=-1):
        if not self.options['draw']:
            return []
        pose = np.zeros(2*self.n_dim)
        pose[:self.n_dim] = self.signals['position'][:, t]
        pose[2] = self.signals['orientation'][:, t]
        return self.shape.draw(pose)