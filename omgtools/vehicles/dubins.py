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
from ..basics.shape import Square, Rectangle, Circle
from ..basics.spline_extra import sample_splines, definite_integral
from ..basics.spline_extra import evalspline, running_integral
from casadi import inf
import numpy as np

# Elaboration of the vehicle model:
# dx = V*cos(theta)
# dy = V*sin(theta)
# dtheta = dtheta

# Use tangent half angle substitution: tg_ha = tan(theta/2)
# sin(theta) = (2*tg_ha)/(1+tg_ha**2)
# cos(theta) = (1-tg_ha**2)/(1+tg_ha**2)
# This gives: 
# dx = V/(1+tg_ha**2)*(1-tg_ha**2)
# dy = V/(1+tg_ha**2)*(2*tg_ha)
# Substitute: v_til = V/(1+tg_ha**2)

# dx = v_til*(1-tg_ha**2)
# dy = v_til*(2*tg_ha)
# Spline variables of the problem: v_til and tg_ha

class Dubins(Vehicle):

    def __init__(self, shapes=Circle(0.1), options={}, bounds={}):
        Vehicle.__init__(
            self, n_spl=2, degree=2, shapes=shapes, options=options)
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        self.wmin = bounds['wmin'] if 'wmin' in bounds else -30.  # in deg/s
        self.wmax = bounds['wmax'] if 'wmax' in bounds else 30.
        # time horizon
        self.T = self.define_symbol('T')  # motion time
        self.t = self.define_symbol('t')  # current time of first knot
        self.pos0 = self.define_symbol('pos0', 2)  # current position

    def set_default_options(self):
        Vehicle.set_default_options(self)

    def define_trajectory_constraints(self, splines):
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        self.define_constraint(
            v_til*(1+tg_ha**2) - self.vmax, -inf, 0.)
        # self.define_constraint(
        #     dv_til*(1+tg_ha**2) + 2*v_til*tg_ha*dtg_ha - self.T*self.amax, -inf, 0.)

        # Alternative:
        # dx = v_til*(1-tg_ha**2)
        # dy = v_til*(2*tg_ha)
        # ddx, ddy = dx.derivative(), dy.derivative()
        # self.define_constraint(
        #     (dx**2+dy**2) - (self.T**2)*self.vmax**2, -inf, 0.)
        # self.define_constraint(
        #     (ddx**2+ddy**2) - (self.T**4)*self.amax**2, -inf, 0.)

        # add constraints on change in orientation
        self.define_constraint(2*dtg_ha - (1+tg_ha**2)*self.T*np.radians(self.wmax), -inf, 0.)
        self.define_constraint(-2*dtg_ha + (1+tg_ha**2)*self.T*np.radians(self.wmin), -inf, 0.)
        self.define_constraint(-v_til, -inf, 0)  # positive v_tilde

    def get_initial_constraints(self, splines):
        # these make sure you get continuity along different iterations
        # inputs are function of v_til, tg_ha and dtg_ha so impose constraints on these
        v_til0 = self.define_parameter('v_til0', 1)
        tg_ha0 = self.define_parameter('tg_ha0', 1)
        dtg_ha0 = self.define_parameter('dtg_ha0', 1)
        v_til, tg_ha = splines
        dtg_ha = tg_ha.derivative()
        return [(v_til, v_til0), (tg_ha, tg_ha0),
                (dtg_ha, self.T*dtg_ha0)]

    def get_terminal_constraints(self, splines):
        posT = self.define_parameter('posT', 2)
        tg_haT = self.define_parameter('tg_haT', 1)
        v_tilT = self.define_parameter('v_tilT', 1)
        dtg_haT = self.define_parameter('dtg_haT', 1)       
        self.define_parameter('pos0', 2)  # starting position for integration
        v_til, tg_ha = splines
        dtg_ha = tg_ha.derivative(1)
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        x_int, y_int = self.T*running_integral(dx), self.T*running_integral(dy)
        x = x_int-evalspline(x_int, self.t/self.T) + self.pos0[0]  # self.pos0 was already defined in init
        y = y_int-evalspline(y_int, self.t/self.T) + self.pos0[1]
        term_con = [(x, posT[0]), (y, posT[1]), (tg_ha, tg_haT)]
        term_con_der = [(v_til, v_tilT), (dtg_ha, self.T*dtg_haT)]
        return [term_con, term_con_der]

    def set_initial_conditions(self, pose, input=np.zeros(2)):
        # comes from the user so theta is in deg
        pose[2] = np.radians(pose[2])
        self.prediction['state'] = pose  # x, y, theta[rad]
        self.pose0 = pose # for use in first iteration of splines2signals()
        self.prediction['input'] = input  # V, dtheta

    def set_terminal_conditions(self, pose):
        # comes from the user so theta is in deg
        pose[2] = np.radians(pose[2])
        self.poseT = pose  # x, y, theta[rad]

    def get_init_spline_value(self):
        # generate initial guess for spline variables
        init_value = np.zeros((len(self.basis), 2))
        v_til0 = np.zeros(len(self.basis))
        tg_ha0 = np.tan(self.prediction['state'][2]/2)
        tg_haT = np.tan(self.poseT[2]/2)
        init_value[:, 0] = v_til0
        init_value[:, 1] = np.linspace(tg_ha0, tg_haT, len(self.basis))
        return init_value

    def check_terminal_conditions(self):
        if (np.linalg.norm(self.signals['state'][:, -1] - self.poseT) > 1.e-3 or
                np.linalg.norm(self.signals['input'][:, -1])) > 1.e-3:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        # for the optimization problem
        # convert theta to tg_ha here
        parameters = {}
        parameters['tg_ha0'] = np.tan(self.prediction['state'][2]/2)
        parameters['v_til0'] = self.prediction['input'][0]/(1+parameters['tg_ha0']**2) 
        parameters['dtg_ha0'] = 0.5*self.prediction['input'][1]*(1+parameters['tg_ha0']**2)  # dtg_ha
        parameters['pos0'] = self.prediction['state'][:2]
        parameters['posT'] = self.poseT[:2]  # x,y
        parameters['tg_haT'] = np.tan(self.poseT[2]/2)
        parameters['dtg_haT'] = 0.
        parameters['v_tilT'] = 0.
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines):
        v_til, tg_ha = splines[0], splines[1]
        dtg_ha = tg_ha.derivative(1)
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        x_int, y_int = self.T*running_integral(dx), self.T*running_integral(dy)
        x = x_int-evalspline(x_int, self.t/self.T) + self.pos0[0]  # todo: or is it self.t?
        y = y_int-evalspline(y_int, self.t/self.T) + self.pos0[1]
        self.define_collision_constraints_2d(hyperplanes, environment, [x, y], tg_ha)

    def splines2signals(self, splines, time):
        # for plotting and logging
        # note: here the splines are not dimensionless anymore
        signals = {}
        v_til, tg_ha = splines[0], splines[1]
        dtg_ha = tg_ha.derivative()
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        if not hasattr(self, 'signals'):  # first iteration
            dx_int, dy_int = running_integral(dx), running_integral(dy)
            x = dx_int - dx_int(time[0]) + self.pose0[0]
            y = dy_int - dy_int(time[0]) + self.pose0[1]
        else:
            dx_int, dy_int = running_integral(dx), running_integral(dy)  # current state
            x = dx_int - dx_int(time[0]) + self.signals['state'][0, -1]
            y = dy_int - dy_int(time[0]) + self.signals['state'][1, -1]
        theta = 2*np.arctan2(sample_splines([tg_ha], time),1)
        dtheta = 2*np.array(sample_splines([dtg_ha],time))/(1+np.array(sample_splines([tg_ha], time))**2)
        input = np.c_[sample_splines([v_til*(1+tg_ha**2)], time)]
        input = np.r_[input, dtheta]
        signals['state'] = np.c_[sample_splines([x, y], time)]
        signals['state'] = np.r_[signals['state'], theta]
        signals['input'] = input
        signals['pose'] = signals['state']
        signals['v_tot'] = input[0, :]
        return signals

    def ode(self, state, input):
        # state: x, y, theta
        # inputs: V, dtheta
        # find relation between dstate and state, inputs: dx = Ax+Bu
        # dstate = dx, dy, dtheta
        # dstate[2] = input[1]
        u1, u2 = input[0], input[1]
        return np.r_[u1*np.cos(state[2]), u1*np.sin(state[2]), u2].T

    def draw(self, t=-1):
        ret = []
        for shape in self.shapes:
            if isinstance(shape, Circle):
                wheel = Square(shape.radius/3)
                front = Circle(shape.radius/8)
                ret += shape.draw(self.signals['pose'][:, t])
                ret += wheel.draw(self.signals['pose'][:, t]+
                                  (shape.radius/2)*np.array([np.cos(self.signals['pose'][2, t]-np.pi/2),
                                                             np.sin(self.signals['pose'][2, t]-np.pi/2),
                                                             self.signals['pose'][2, t]]))
                ret += wheel.draw(self.signals['pose'][:, t]+
                                  (shape.radius/2)*np.array([np.cos(self.signals['pose'][2, t]+np.pi/2),
                                                             np.sin(self.signals['pose'][2, t]+np.pi/2),
                                                             self.signals['pose'][2, t]]))
                ret += front.draw(self.signals['pose'][:, t]+
                                  (shape.radius/1.5)*np.array([np.cos(self.signals['pose'][2, t]),
                                                               np.sin(self.signals['pose'][2, t]),
                                                               self.signals['pose'][2, t]]))
            else:
                ret += shape.draw(self.signals['pose'][:, t])
        return ret
