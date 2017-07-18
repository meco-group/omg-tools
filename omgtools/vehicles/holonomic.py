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

    def __init__(self, shapes=Circle(0.1), options=None, bounds=None):
        bounds = bounds or {}
        Vehicle.__init__(
            self, n_spl=2, degree=3, shapes=shapes, options=options)

        if ((not 'syslimit' in self.options) or  # default choose norm_inf
                (self.options['syslimit'] is 'norm_inf')):
            # user specified separate velocities for x and y
            self.vxmin = bounds['vxmin'] if 'vxmin' in bounds else -0.5
            self.vymin = bounds['vymin'] if 'vymin' in bounds else -0.5
            self.vxmax = bounds['vxmax'] if 'vxmax' in bounds else 0.5
            self.vymax = bounds['vymax'] if 'vymax' in bounds else 0.5
            self.axmin = bounds['axmin'] if 'axmin' in bounds else -1.
            self.aymin = bounds['aymin'] if 'aymin' in bounds else -1.
            self.axmax = bounds['axmax'] if 'axmax' in bounds else 1.
            self.aymax = bounds['aymax'] if 'aymax' in bounds else 1.
            # user specified a single velocity for x and y
            if 'vmin' in bounds:
                self.vxmin = self.vymin = bounds['vmin']
            if 'vmax' in bounds:
                self.vxmax = self.vymax = bounds['vmax']
            if 'amin' in bounds:
                self.axmin = self.aymin = bounds['amin']
            if 'amax' in bounds:
                self.axmax = self.aymax = bounds['amax']
        elif self.options['syslimit'] is 'norm_2':
            self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
            self.amax = bounds['amax'] if 'amax' in bounds else 1.

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'syslimit': 'norm_inf'})

    # def init(self):
    #     pass
    #     # time horizon
    #     # self.T = self.define_symbol('T')

    def define_trajectory_constraints(self, splines, horizon_time):
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        # constrain total velocity
        if self.options['syslimit'] is 'norm_2':
            self.define_constraint(
                (dx**2+dy**2) - (horizon_time**2)*self.vmax**2, -inf, 0.)
            self.define_constraint(
                (ddx**2+ddy**2) - (horizon_time**4)*self.amax**2, -inf, 0.)
        # constrain local velocity
        elif self.options['syslimit'] is 'norm_inf':
            self.define_constraint(-dx + horizon_time*self.vxmin, -inf, 0.)
            self.define_constraint(-dy + horizon_time*self.vymin, -inf, 0.)
            self.define_constraint(dx - horizon_time*self.vxmax, -inf, 0.)
            self.define_constraint(dy - horizon_time*self.vymax, -inf, 0.)

            self.define_constraint(-ddx + (horizon_time**2)*self.axmin, -inf, 0.)
            self.define_constraint(-ddy + (horizon_time**2)*self.aymin, -inf, 0.)
            self.define_constraint(ddx - (horizon_time**2)*self.axmax, -inf, 0.)
            self.define_constraint(ddy - (horizon_time**2)*self.aymax, -inf, 0.)
        else:
            raise ValueError(
                'Only norm_2 and norm_inf are defined as system limit.')

    def get_initial_constraints(self, splines, horizon_time):
        state0 = self.define_parameter('state0', 2)
        input0 = self.define_parameter('input0', 2)
        x, y = splines
        dx, dy = x.derivative(), y.derivative()
        return [(x, state0[0]), (y, state0[1]),
                (dx, horizon_time*input0[0]), (dy, horizon_time*input0[1])]

    def get_terminal_constraints(self, splines):
        position = self.define_parameter('positionT', 2)
        x, y = splines
        term_con = [(x, position[0]), (y, position[1])]
        term_con_der = []
        for d in range(1, self.degree+1):
            term_con_der.extend([(x.derivative(d), 0.), (y.derivative(d), 0.)])
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None):
        if input is None:
            input = np.zeros(2)
        # list all predictions that are used in set_parameters
        self.prediction['state'] = state
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
        tol = self.options['stop_tol']
        if (np.linalg.norm(self.signals['state'][:, -1] - self.positionT) > tol or
                np.linalg.norm(self.signals['input'][:, -1])) > tol:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = Vehicle.set_parameters(self, current_time)
        parameters[self]['state0'] = self.prediction['state']
        parameters[self]['input0'] = self.prediction['input']
        parameters[self]['positionT'] = self.positionT
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines, horizon_time):
        x, y = splines[0], splines[1]
        self.define_collision_constraints_2d(hyperplanes, environment, [x, y], horizon_time)

    def splines2signals(self, splines, time):
        signals = {}
        x, y = splines[0], splines[1]
        dx, dy = x.derivative(), y.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        input = np.c_[sample_splines([dx, dy], time)]
        signals['state'] = np.c_[sample_splines([x, y], time)]
        signals['input'] = input
        signals['v_tot'] = np.sqrt(input[0, :]**2 + input[1, :]**2)
        signals['a'] = np.c_[sample_splines([ddx, ddy], time)]
        return signals

    def state2pose(self, state):
        return np.r_[state, 0.]

    def ode(self, state, input):
        return input
