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
from ..basics.shape import Circle
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np


class Holonomic(Vehicle):

    def __init__(self, shapes=Circle(0.1), options={}, bounds={}):
        Vehicle.__init__(
            self, n_spl=2, degree=3, shapes=shapes, options=options)
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -0.5
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amin = bounds['amin'] if 'amin' in bounds else -1.
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        # time horizon
        self.T = self.define_symbol('T')

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'syslimit': 'norm_inf'})

    def define_trajectory_constraints(self, splines):
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        if self.options['syslimit'] is 'norm_2':
            self.define_constraint(
                (dx**2+dy**2) - (self.T**2)*self.vmax**2, -inf, 0.)
            self.define_constraint(
                (ddx**2+ddy**2) - (self.T**4)*self.amax**2, -inf, 0.)
        elif self.options['syslimit'] is 'norm_inf':
            self.define_constraint(-dx + self.T*self.vmin, -inf, 0.)
            self.define_constraint(-dy + self.T*self.vmin, -inf, 0.)
            self.define_constraint(dx - self.T*self.vmax, -inf, 0.)
            self.define_constraint(dy - self.T*self.vmax, -inf, 0.)

            self.define_constraint(-ddx + (self.T**2)*self.amin, -inf, 0.)
            self.define_constraint(-ddy + (self.T**2)*self.amin, -inf, 0.)
            self.define_constraint(ddx - (self.T**2)*self.amax, -inf, 0.)
            self.define_constraint(ddy - (self.T**2)*self.amax, -inf, 0.)
        else:
            raise ValueError(
                'Only norm_2 and norm_inf are defined as system limit.')

    def get_initial_constraints(self, splines):
        state0 = self.define_parameter('state0', 2)
        input0 = self.define_parameter('input0', 2)
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        return [(x, state0[0]), (y, state0[1]),
                (dx, self.T*input0[0]), (dy, self.T*input0[1])]

    def get_terminal_constraints(self, splines):
        position = self.define_parameter('positionT', 2)
        x, y = splines
        return [(x, position[0]), (y, position[1])]

    def set_initial_conditions(self, position, input=np.zeros(2)):
        self.prediction['state'] = position
        self.prediction['input'] = input

    def set_terminal_conditions(self, position):
        self.positionT = position

    def get_init_spline_value(self):
        init_value = np.zeros((len(self.basis), 2))
        pos0 = self.prediction['state']
        posT = self.positionT
        for k in range(2):
            # init_value[:, k] = np.r_[pos0[k]*np.ones(self.degree), np.linspace(
            #     pos0[k], posT[k], len(self.basis) - 2*self.degree), posT[k]*np.ones(self.degree)]
            init_value[:, k] = np.linspace(pos0[k], posT[k], len(self.basis))
        return init_value

    def check_terminal_conditions(self):
        if (np.linalg.norm(self.signals['state'][:, -1] - self.positionT) > 1.e-3 or
                np.linalg.norm(self.signals['input'][:, -1])) > 1.e-3:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = {}
        parameters['state0'] = self.prediction['state']
        parameters['input0'] = self.prediction['input']
        parameters['positionT'] = self.positionT
        return parameters

    def define_collision_constraints(self, hyperplanes, room_lim, splines):
        x, y = splines[0], splines[1]
        self.define_collision_constraints_2d(hyperplanes, room_lim, [x, y])

    def splines2signals(self, splines, time):
        signals = {}
        x, y = splines[0], splines[1]
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        input = np.c_[sample_splines([dx, dy], time)]
        signals['state'] = np.c_[sample_splines([x, y], time)]
        signals['input'] = input
        signals['pose'] = np.r_[signals['state'], np.zeros((1, len(time)))]
        signals['v_tot'] = np.sqrt(input[0, :]**2 + input[1, :]**2)
        signals['a'] = np.c_[sample_splines([ddx, ddy], time)]
        return signals

    def ode(self, state, input):
        return input
