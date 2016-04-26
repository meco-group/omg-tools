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
from dubins import Dubins
from ..basics.shape import Circle
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np


class Trailer(Vehicle):

    def __init__(self, lead_veh=Dubins(Circle(radius=0.4)), shapes=Circle(0.2), l_hitch=0.2, options={}, bounds={}):
        Vehicle.__init__(
            self, n_spl=1 + lead_veh.n_spl, degree=2, shapes=shapes, options=options)
        # n_spl contains all splines of lead_veh and trailer
        # being: tg_ha_trailer, v_til_veh, tg_ha_veh
        self.lead_veh = lead_veh  # vehicle which pulls the trailer
        self.l_hitch = l_hitch  # distance between rear axle of trailer and connection point on the car

    def set_default_options(self):
        Vehicle.set_default_options(self)

    def define_trajectory_constraints(self, splines):
        T = self.define_symbol('T')
        tg_ha_tr = splines[0]
        dtg_ha_tr = tg_ha_tr.derivative()
        v_til_veh, tg_ha_veh = splines[1:]  
        V_veh = v_til_veh*(1+tg_ha_veh**2)  # inputs to the trailer: V_veh and tg_ha_veh
        eps = 1e-3
        # change in orientation of the trailer is due to velocity of vehicle
        # relaxed the equality constraint
        self.define_constraint(2*dtg_ha_tr*self.l_hitch*(1+tg_ha_veh**2) -
                               T*V_veh*(2*tg_ha_veh*(1-tg_ha_tr**2)-((1-tg_ha_veh**2)**4)*2*tg_ha_tr) - T*eps,
                               -inf, 0.)
        self.define_constraint(-2*dtg_ha_tr*self.l_hitch*(1+tg_ha_veh**2) +
                               T*V_veh*(2*tg_ha_veh*(1-tg_ha_tr**2)-((1-tg_ha_veh**2)**4)*2*tg_ha_tr) - T*eps,
                               -inf, 0.)
        self.lead_veh.define_trajectory_constraints(splines[1: ])

    def get_initial_constraints(self, splines):
        # trailer has a certain theta0 --> trailer position follows from this
        tg_ha0 = self.define_parameter('tg_ha_tr0', 1)
        tg_ha = splines[0]
        con_tr = [(tg_ha, tg_ha0[0])]
        con_veh = self.lead_veh.get_initial_constraints(splines[1: ])
        return con_tr + con_veh  # put in one list

    def get_terminal_constraints(self, splines):
        # Only impose if self.theta_trT exists, e.g. if parking a trailer
        if hasattr(self, 'theta_trT'):
            tg_ha = splines[0]
            tg_haT = self.define_parameter('tg_ha_trT', 1)
            term_con_tr = [(tg_ha, tg_haT)]
            term_con_der_tr = []
            con_veh = self.lead_veh.get_terminal_constraints(splines[1: ])
            term_con_veh = con_veh[0]
            term_con_der_veh = con_veh[1]
            term_con = term_con_tr + term_con_veh
            term_con_der = term_con_der_tr + term_con_der_veh
        else:
            term_con, term_con_der = [], []
        return [term_con, term_con_der]

    def set_initial_conditions(self, theta, input=np.zeros(4)):
        # todo: add complete state to prediction: x,y,theta,x,y,theta for tr and veh?
        self.prediction['state'] = theta[0]  # theta 
        self.prediction['input'] = input  # velocity vector from leading vehicle [V, tg_ha_veh] + lead_veh inputs
        # The velocity in which the car pulls will always be parallel to the orientation of the vehicle,
        # except for the case in which you have a holonomic vehicle (which doesn't have an orientation).

    def set_terminal_conditions(self, theta):
        # Optional, e.g. only for parking a trailer
        self.theta_trT = theta[0]

    def get_init_spline_value(self):
        init_value_tr = np.zeros((len(self.basis), 1))
        theta_tr0 = self.prediction['state']
        if hasattr(self, 'theta_trT'):
            theta_trT = self.theta_trT
        else:
            theta_trT = theta_tr0
        init_value_tr[:, 0] = np.linspace(theta_tr0, theta_trT, len(self.basis))
        init_value_veh = self.lead_veh.get_init_spline_value()
        init_value = np.c_[init_value_tr, init_value_veh]
        return init_value

    def check_terminal_conditions(self):
        # Two options: move vehicle with a trailer, or position/park the trailer somewhere.
        # Move vehicle with trailer: no theta_trT specified
        # Park vehicle with trailer: theta_trT specified, True if vehicle and trailer pose reached.
        if hasattr(self, 'theta_trT'):
            if (np.linalg.norm(self.signals['state'][2, -1] - self.theta_trT) > 1.e-3):
                result = False
        else:
            result = True
        result = (self.lead_veh.check_terminal_conditions() and result)
        return result

    def set_parameters(self, current_time):
        parameters = {}
        parameters_tr = {}
        parameters_tr['tg_ha_tr0'] = self.prediction['state']
        parameters_tr['tg_ha_trT'] = self.thetaT
        parameters_veh = self.lead_veh.set_parameters(current_time)
        parameters.update(parameters_tr)
        parameters.update(parameters_veh)
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines):
        tg_ha = splines[0]
        x_veh, y_veh = self.lead_veh.get_pos_splines(splines[1: ])
        self.define_collision_constraints_2d(hyperplanes, environment, [x_veh, y_veh], tg_ha, -self.l_hitch)
        self.lead_veh.define_collision_constraints(hyperplanes, environment, splines[1: ])

    def splines2signals(self, splines, time):
        signals = {}
        tg_ha = splines[0]
        tg_ha = np.array(sample_splines([tg_ha], time))
        theta = 2*np.arctan2(tg_ha, 1)
        signals_veh = self.lead_veh.splines2signals(splines[1: ], time)        
        x = (signals_veh['state'][0, :]*(1+tg_ha) - self.l_hitch*(1-tg_ha))/(1+tg_ha)
        y = (signals_veh['state'][1, :]*(1+tg_ha) - self.l_hitch*(2*tg_ha))/(1+tg_ha)
        input = np.r_[signals_veh['input'][0, :], signals_veh['state'][2, :]]  # V_veh, theta_veh
        signals['state'] = np.r_[x, y, theta, signals_veh['state']]  # trailer state
        signals['pose'] = signals['state']
        signals['input'] = np.r_[input, signals_veh['input']]
        return signals

    def ode(self, state, input):
        # state = [x_tr, y_tr, theta_tr, x_veh, y_veh, theta_veh]
        # input = [V_veh, theta_veh, input_veh]
        # state: theta_tr
        # input: V_veh, theta_veh
        # ode: dtheta = V_veh/l_hitch*sin(theta_veh-theta)
        u1, u2 = input[-2], input[-1]
        ode_veh = self.lead_veh.ode(state[3:], input[2:])
        ode_trailer = np.r_[u1/self.l_hitch*np.sin(u2-state[2])].T
        ode = np.r_[ode_trailer, ode_veh]
        return ode

    def draw(self, t=-1):
        ret = []
        for shape in self.shapes:
            ret += shape.draw(self.signals['pose'][:, t])
            # plot connection between car and trailer
            pt1 = self.signals['pose'][:, t] + shape.width/2*np.array([np.cos(self.signals['pose'][2, t]),
                                                                       np.sin(self.signals['pose'][2, t]), 0])
            pt2 = self.signals['pose'][:, t] + shape.width/2*np.array([np.cos(self.signals['pose'][2, t]),
                                                                       np.sin(self.signals['pose'][2, t]), 0]) + self.l_hitch - shape.width
            ret += np.array([pt1, pt2])
        ret += self.lead_veh.draw(t)
        return ret
