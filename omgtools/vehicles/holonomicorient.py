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

from .vehicle import Vehicle
from ..basics.shape import Rectangle
from ..basics.spline_extra import sample_splines, definite_integral
from casadi import inf
import numpy as np


class HolonomicOrient(Vehicle):

    def __init__(self, shapes=Rectangle(width=0.2, height=0.4), options=None, bounds=None):
        bounds = bounds or {}
        Vehicle.__init__(
            self, n_spl=3, degree=3, shapes=shapes, options=options)
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -0.5
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amin = bounds['amin'] if 'amin' in bounds else -1.
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        self.wmin = bounds['wmin'] if 'wmin' in bounds else -np.pi/6. # in rad/s
        self.wmax = bounds['wmax'] if 'wmax' in bounds else np.pi/6.

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'syslimit': 'norm_inf'})
        self.options.update({'reg_type': None})  # no reg by default

    def init(self):
        # time horizon
        self.t = self.define_symbol('t')

    def define_trajectory_constraints(self, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
        x, y, tg_ha = splines
        dx, dy, dtg_ha = x.derivative(), y.derivative(), tg_ha.derivative()
        ddx, ddy = x.derivative(2), y.derivative(2)
        if self.options['syslimit'] is 'norm_2':
            self.define_constraint(
                (dx**2+dy**2) - (horizon_time**2)*self.vmax**2, -inf, 0.)
            self.define_constraint(
                (ddx**2+ddy**2) - (horizon_time**4)*self.amax**2, -inf, 0.)
        elif self.options['syslimit'] is 'norm_inf':
            self.define_constraint(-dx + horizon_time*self.vmin, -inf, 0.)
            self.define_constraint(-dy + horizon_time*self.vmin, -inf, 0.)
            self.define_constraint(dx - horizon_time*self.vmax, -inf, 0.)
            self.define_constraint(dy - horizon_time*self.vmax, -inf, 0.)

            self.define_constraint(-ddx + (horizon_time**2)*self.amin, -inf, 0.)
            self.define_constraint(-ddy + (horizon_time**2)*self.amin, -inf, 0.)
            self.define_constraint(ddx - (horizon_time**2)*self.amax, -inf, 0.)
            self.define_constraint(ddy - (horizon_time**2)*self.amax, -inf, 0.)
        else:
            raise ValueError(
                'Only norm_2 and norm_inf are defined as system limit.')
        # add constraints on change in orientation
        self.define_constraint(2*dtg_ha - (1+tg_ha**2)*horizon_time*self.wmax, -inf, 0.)
        self.define_constraint(-2*dtg_ha + (1+tg_ha**2)*horizon_time*self.wmin, -inf, 0.)
        # add regularization on dtg_ha
        if (self.options['reg_type'] == 'norm_1' and self.options['reg_weight'] != 0.0):
            dtg_ha = tg_ha.derivative()
            g_reg = self.define_spline_variable(
                        'g_reg', 1, basis=dtg_ha.basis)[0]
            objective = definite_integral(g_reg, self.t/horizon_time, 1.)
            self.define_constraint(dtg_ha - g_reg, -inf, 0.)
            self.define_constraint(-dtg_ha - g_reg, -inf, 0.)
            self.define_objective(self.options['reg_weight']*objective)
        if (self.options['reg_type'] == 'norm_2'and self.options['reg_weight'] != 0.0):
            dtg_ha = tg_ha.derivative()
            objective = definite_integral(dtg_ha**2, self.t/horizon_time, 1.)
            self.define_objective(self.options['reg_weight']*objective)

    def get_initial_constraints(self, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
        pos0 = self.define_parameter('pos0', 2)  # x, y
        tg_ha0 = self.define_parameter('tg_ha0', 1)
        vel0 = self.define_parameter('vel0', 2)  # dx, dy
        dtg_ha0 = self.define_parameter('dtg_ha0', 1)
        x, y, tg_ha = splines
        dx, dy, dtg_ha = x.derivative(), y.derivative(), tg_ha.derivative()
        return [(x, pos0[0]), (y, pos0[1]), (tg_ha, tg_ha0),
                (dx, horizon_time*vel0[0]), (dy, horizon_time*vel0[1]), (dtg_ha, horizon_time*dtg_ha0)]

    def get_terminal_constraints(self, splines, horizon_time=None):
        posT = self.define_parameter('posT', 2)
        tg_haT = self.define_parameter('tg_haT', 1)
        x, y, tg_ha = splines
        term_con = [(x, posT[0]), (y, posT[1]), (tg_ha, tg_haT)]
        term_con_der = []
        for d in range(1, self.degree+1):
            term_con_der.extend([(x.derivative(d), 0.), (y.derivative(d), 0.), (tg_ha.derivative(d), 0.)])
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None):
        if input is None:
            input = np.zeros(3)
        # list all predictions that are used in set_parameters
        self.prediction['state'] = state
        self.prediction['input'] = input

    def set_terminal_conditions(self, pose):
        self.poseT = pose

    def get_init_spline_value(self):
        # for the optimization problem so use tg_ha
        init_value = np.zeros((len(self.basis), 3))
        pose0 = np.zeros(3)
        pose0[:2] = self.prediction['state'][:2]  # x,y,theta[rad]
        pose0[2] =  np.tan(self.prediction['state'][2]/2)  # tg_ha
        poseT = np.zeros(3)
        poseT[:2] = self.poseT[:2]  # x,y,theta[rad]
        poseT[2] =  np.tan(self.poseT[2]/2)  # tg_ha
        for k in range(2):
            # init_value[:, k] = np.r_[pos0[k]*np.ones(self.degree), np.linspace(
            #     pos0[k], posT[k], len(self.basis) - 2*self.degree), posT[k]*np.ones(self.degree)]
            init_value[:, k] = np.linspace(pose0[k], poseT[k], len(self.basis))
        init_value = [init_value]
        return init_value

    def check_terminal_conditions(self):
        tol = self.options['stop_tol']
        if (np.linalg.norm(self.signals['state'][:, -1] - self.poseT) > tol or
                np.linalg.norm(self.signals['input'][:, -1])) > tol:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        # for the optimization problem
        # convert theta to tg_ha here
        parameters = Vehicle.set_parameters(self, current_time)
        parameters[self]['pos0'] = self.prediction['state'][:2]  # x, y
        parameters[self]['tg_ha0'] = np.tan(self.prediction['state'][2]/2)
        parameters[self]['vel0'] = self.prediction['input'][:2]  # dx, dy
        parameters[self]['dtg_ha0'] = 0.5*self.prediction['input'][2]*(1+parameters[self]['tg_ha0']**2)
        parameters[self]['posT'] = self.poseT[:2]  # x, y
        parameters[self]['tg_haT'] = np.tan(self.poseT[2]/2)
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines, horizon_time=None):
        x, y, tg_ha = splines[0], splines[1], splines[2]
        self.define_collision_constraints_2d(hyperplanes, environment, [x, y], horizon_time, tg_ha=tg_ha)

    def splines2signals(self, splines, time):
        # for plotting and logging
        signals = {}
        x, y, tg_ha = splines[0], splines[1], splines[2]
        dx, dy, dtg_ha = x.derivative(), y.derivative(), tg_ha.derivative()
        theta = 2*np.arctan2(sample_splines([tg_ha], time),1)
        dtheta = 2*np.array(sample_splines([dtg_ha],time))/(1+np.array(sample_splines([tg_ha], time))**2)
        ddx, ddy = x.derivative(2), y.derivative(2)
        input = np.c_[sample_splines([dx, dy], time)]
        input = np.r_[input,dtheta]
        signals['state'] = np.c_[sample_splines([x, y], time)]
        signals['state'] = np.r_[signals['state'], theta]
        signals['input'] = input
        signals['v_tot'] = np.sqrt(input[0, :]**2 + input[1, :]**2)
        signals['a'] = np.c_[sample_splines([ddx, ddy], time)]
        return signals

    def state2pose(self, state):
        return state

    def ode(self, state, input):
        return input
