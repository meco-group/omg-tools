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

from vehicle import Vehicle
from ..basics.shape import Rectangle
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np


class Vehicle1D(Vehicle):

    def __init__(self, width=0.7, height=0.1, time_constant=0.4, options=None, bounds=None):
        bounds = bounds or {}
        Vehicle.__init__(self, n_spl=1, degree=3, shapes=Rectangle(width, height), options=options)
        self.time_constant = time_constant
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -25.
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 25.
        self.umin = bounds['umin'] if 'umin' in bounds else -1000.
        self.umax = bounds['umax'] if 'umax' in bounds else 1000.
        # time horizon
        self.T = self.define_symbol('T')

    def define_trajectory_constraints(self, splines):
        x = splines[0]
        dx, ddx, dddx = x.derivative(), x.derivative(2), x.derivative(3)
        tau_tf = self.time_constant*self.T
        u = tau_tf*dddx + ddx
        self.define_constraint(-dx + self.T*self.vmin, -inf, 0.)
        self.define_constraint(dx - self.T*self.vmax, -inf, 0.)
        self.define_constraint(-u + (self.T**2)*self.umin, -inf, 0.)
        self.define_constraint(u - (self.T**2)*self.umax, -inf, 0.)
        # self.define_constraint(-ddx + (self.T**2)*self.umin, -inf, 0.)
        # self.define_constraint(ddx - (self.T**2)*self.umax, -inf, 0.)

    def get_initial_constraints(self, splines):
        spl0 = self.define_parameter('spl0')
        dspl0 = self.define_parameter('dspl0')
        ddspl0 = self.define_parameter('ddspl0')
        dddspl0 = self.define_parameter('dddspl0')
        x = splines[0]
        dx, ddx, dddx = x.derivative(), x.derivative(2), x.derivative(3)
        # return [(x, spl0[0]), (dx, self.T*dspl0[0]),
        #         (ddx, (self.T**2)*ddspl0[0]), (dddx, (self.T**3)*dddspl0[0])]
        return [(x, spl0[0]), (dx, self.T*dspl0[0])]

    def get_terminal_constraints(self, splines):
        position = self.define_parameter('positionT')
        x = splines[0]
        term_con = [(x, position[0])]
        term_con_der = []
        for d in range(1, self.degree+1):
            term_con_der.extend([(x.derivative(d), 0.)])
        return [term_con, term_con_der]

    def set_initial_conditions(self, position):
        self.prediction['state'] = np.r_[position, np.zeros(2)].T
        self.prediction['dspl'] = 0.
        self.prediction['ddspl'] = 0.
        self.prediction['dddspl'] = 0.

    def set_terminal_conditions(self, position):
        self.positionT = position

    def get_init_spline_value(self):
        pos0 = self.prediction['state'][0]
        posT = self.positionT[0]
        # init_value = np.r_[pos0[0]*np.ones(self.degree), np.linspace(pos0[0], posT[0], len(self.basis) - 2*self.degree), posT[0]*np.ones(self.degree)]
        init_value = np.linspace(pos0, posT, len(self.basis))
        return init_value

    def check_terminal_conditions(self):
        if (np.linalg.norm(self.signals['pose'][:1, -1] - self.positionT) > 1.e-3 or
                np.linalg.norm(self.signals['input'][:, -1])) > 1.e-3:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = {}
        parameters['spl0'] = self.prediction['state'][0]
        parameters['dspl0'] = self.prediction['state'][1]
        parameters['ddspl0'] = self.prediction['state'][2]
        parameters['dddspl0'] = self.prediction['dddspl']
        parameters['positionT'] = self.positionT
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines):
        pass

    def splines2signals(self, splines, time):
        signals = {}
        x = splines[0]
        dx, ddx, dddx = x.derivative(), x.derivative(2), x.derivative(3)
        x_s = sample_splines(x, time)
        dx_s = sample_splines(dx, time)
        ddx_s = sample_splines(ddx, time)
        dddx_s = sample_splines(dddx, time)
        u = self.time_constant*dddx + ddx
        u_s = sample_splines(u, time)
        signals['state'] = np.c_[x_s, dx_s, ddx_s].T
        signals['input'] = np.c_[u_s].T
        signals['pose'] = np.c_[signals['state'][0, :], np.zeros((len(time), 2))].T
        signals['dddspl'] = np.c_[dddx_s].T
        return signals

    def ode(self, state, input):
        dx, ddx = state[1], state[2]
        return np.r_[dx, ddx, (1./self.time_constant)*(-ddx+input)]
