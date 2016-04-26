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
from ..basics.shape import Rectangle, Circle
from ..basics.spline_extra import sample_splines, evalspline
from ..basics.spline_extra import running_integral
from casadi import inf
import numpy as np

# Elaboration of the vehicle model:
# dx = V*cos(theta)
# dy = V*sin(theta)
# dtheta = -V/L*tan(delta) with L = distance between front and back wheel

# Use tangent half angle substitution: tg_ha = tan(theta/2)
# sin(theta) = (2*tg_ha)/(1+tg_ha**2)
# cos(theta) = (1-tg_ha**2)/(1+tg_ha**2)
# This gives: 
# dx = V/(1+tg_ha**2)*(1-tg_ha**2)
# dy = V/(1+tg_ha**2)*(2*tg_ha)
# Substitute: v_til = V/(1+tg_ha**2)

# dx = v_til*(1-tg_ha**2)
# dy = v_til*(2*tg_ha)
# dtheta = -v_til/L*(1+tg_ha**2)*tan(delta)

# And since theta = 2*arctan(tg_ha)
# dtheta = 2*dtg_ha/(1+tg_ha**2)
# So tan(delta) = (-2*dtg_ha*L)/(v_til*(1+tg_ha**2)**2)
# Spline variables of the problem: v_til and tg_ha
# delta follows from v_til, tg_ha, dtg_ha

class AGV(Vehicle):

    def __init__(self, length=0.4, options={}, bounds={}):
        # shapes e.g. Rectangle(width=0.8, height=0.2) or Circle(length/2.)
        Vehicle.__init__(
            self, n_spl=2, degree=2, shapes=Rectangle(width=0.8, height=0.2), options=options)
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        self.dmin = bounds['dmin'] if 'dmin' in bounds else -30.  # steering angle [deg]
        self.dmax = bounds['dmax'] if 'dmax' in bounds else 30.
        self.ddmin = bounds['ddmin'] if 'ddmin' in bounds else -45.  # dsteering angle [deg/s]
        self.ddmax = bounds['ddmax'] if 'ddmax' in bounds else 45.
        self.length = length
        # time horizon
        self.T = self.define_symbol('T')  # motion time
        self.t = self.define_symbol('t')  # current time of first knot
        self.pos0 = self.define_symbol('pos0', 2)  # current position

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'plot_type': 'bicycle'})  # by default plot a bicycle

    def define_trajectory_constraints(self, splines):
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        ddtg_ha = tg_ha.derivative(2)
        self.define_constraint(
            v_til*(1+tg_ha**2) - self.vmax, -inf, 0.)
        self.define_constraint(
            dv_til*(1+tg_ha**2) + 2*v_til*tg_ha*dtg_ha - self.T*self.amax, -inf, 0.)
        # Alternative:
        # dx = v_til*(1-tg_ha**2)
        # dy = v_til*(2*tg_ha)
        # ddx, ddy = dx.derivative(), dy.derivative()
        # self.define_constraint(
        #     (dx**2+dy**2) - (self.T**2)*self.vmax**2, -inf, 0.)
        # self.define_constraint(
        #     (ddx**2+ddy**2) - (self.T**4)*self.amax**2, -inf, 0.)

        # limit steering angle
        self.define_constraint(
             -2*dtg_ha*self.length - v_til*(1+tg_ha**2)**2*np.tan(np.radians(self.dmax))*self.T, -inf, 0.)
        self.define_constraint(
             +2*dtg_ha*self.length + v_til*(1+tg_ha**2)**2*np.tan(np.radians(self.dmin))*self.T, -inf, 0.)
    # limit rate of change of steering angle
        self.define_constraint(
             -2*self.length*ddtg_ha*(v_til*(1+tg_ha**2)**2)
             +2*self.length*dtg_ha*(dv_til*(1+tg_ha**2)**2
             +v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) - ((self.T**2)*v_til**2*(1+tg_ha**2)**4
             +(2*self.length*dtg_ha)**2)*np.radians(self.ddmax), -inf, 0.)
        self.define_constraint(
             2*self.length*ddtg_ha*(v_til*(1+tg_ha**2)**2)
             -2*self.length*dtg_ha*(dv_til*(1+tg_ha**2)**2
             +v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) + ((self.T**2)*v_til**2*(1+tg_ha**2)**4
             +(2*self.length*dtg_ha)**2)*np.radians(self.ddmin), -inf, 0.)
        self.define_constraint(-v_til, -inf, 0)  # model requires positive V, so positive v_tilde

    def get_initial_constraints(self, splines):
        # these make sure you get continuity along different iterations
        # states are function of v_til, tg_ha and dtg_ha so impose constraints on these
        v_til0 = self.define_parameter('v_til0', 1)
        tg_ha0 = self.define_parameter('tg_ha0', 1)
        dtg_ha0 = self.define_parameter('dtg_ha0', 1)
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        ddtg_ha = tg_ha.derivative(2)

        hop0 = self.define_parameter('hop0', 1)
        tdelta0 = self.define_parameter('tdelta0', 1)  # tan(delta)
        self.define_constraint(hop0*(-2.*evalspline(ddtg_ha, self.t/self.T)*self.length
                               -tdelta0*(evalspline(dv_til, self.t/self.T)*(1.+tg_ha0**2)**2)*self.T), 0., 0.)
        # This constraint is obtained by using l'Hopital's rule on the expression of
        # tan(delta) = 2*dtg_ha*self.length / (v_til*(1+tg_ha**2)**2)
        # this constraint is used to impose a specific steering angle when v_til = 0, e.g. when
        # starting from standstill. Note that a part of the denomerator is removed because
        # it contains evalspline(v_til, self.t/self.T)*... which is zero.
        # When v_til is not 0 the steering angle is implicitly imposed due to the fact that tan(delta)
        # is only a function of v_til, tg_ha, dtg_ha. If these variables are smooth the steering angle
        # will also be smooth. Furthermore the steering angle and its rate of change are limited by the 
        # extra constraints above. 

        # Impose final steering angle
        # tdeltaT = self.define_parameter('tdeltaT', 1)  # tan(delta)
        # tg_haT = self.define_parameter('tg_haT', 1)
        # self.define_constraint((2.*ddtg_ha(1.)*self.length
        #                        -tdeltaT*(dv_til(1.)*(1.+tg_haT**2)**2)*self.T), 0., 0.)

        # Further initial constraints are not necessary, these 3 are sufficient to get continuity of the state
        return [(v_til, v_til0), (tg_ha, tg_ha0),
                (dtg_ha, self.T*dtg_ha0)]

    def get_terminal_constraints(self, splines):
        posT = self.define_parameter('posT', 2)
        v_tilT = self.define_parameter('v_tilT', 1)
        dv_tilT = self.define_parameter('dv_tilT', 1)
        tg_haT = self.define_parameter('tg_haT', 1)
        dtg_haT = self.define_parameter('dtg_haT', 1)
        ddtg_haT = self.define_parameter('ddtg_haT', 1)
        self.define_parameter('pos0', 2)  # starting position for integration
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        ddtg_ha = tg_ha.derivative(2)
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        x_int, y_int = self.T*running_integral(dx), self.T*running_integral(dy)
        x = x_int-evalspline(x_int, self.t/self.T) + self.pos0[0]  # self.pos0 was already defined in init
        y = y_int-evalspline(y_int, self.t/self.T) + self.pos0[1]
        term_con = [(x, posT[0]), (y, posT[1]), (tg_ha, tg_haT)]
        term_con_der = [(v_til, v_tilT), (dtg_ha, self.T*dtg_haT),
                        (dv_til, dv_tilT), (ddtg_ha, self.T**2*ddtg_haT)]
        return [term_con, term_con_der]

    def set_initial_conditions(self, pose, delta, input=np.zeros(2)):
        # comes from the user so theta[deg]
        pose[2] = np.radians(pose[2])  # theta
        delta = np.radians(delta)  # delta
        self.prediction['state'] = np.r_[pose, delta]  # x, y, theta[rad], delta[rad]
        self.pose0 = pose # for use in first iteration of splines2signals()
        self.delta0 = delta
        self.prediction['input'] = input  # V, ddelta

    def set_terminal_conditions(self, pose):
        # comes from the user so theta[deg]
        pose[2] = np.radians(pose[2])  # theta
        # pose[3] = np.radians(pose[3])  # delta
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
        # todo: kicked out state[3] since you cannot impose a steerT for now
        if (np.linalg.norm(self.signals['state'][:3, -1] - self.poseT) > 1.e-3 or
            np.linalg.norm(self.signals['input'][:, -1])) > 1.e-3:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        # for the optimization problem
        parameters = {}
        parameters['tg_ha0'] = np.tan(self.prediction['state'][2]/2)
        parameters['v_til0'] = self.prediction['input'][0]/(1+parameters['tg_ha0']**2) 
        parameters['pos0'] = self.prediction['state'][:2]
        parameters['posT'] = self.poseT[:2]  # x, y
        parameters['v_tilT'] = 0.
        parameters['dv_tilT'] = 0.
        parameters['tg_haT'] = np.tan(self.poseT[2]/2)
        parameters['dtg_haT'] = 0.
        parameters['ddtg_haT'] = 0.
        # parameters['tdeltaT'] = np.tan(self.poseT[3])
        if (parameters['v_til0'] <= 1e-4):  # use l'Hopital's rule
            parameters['hop0'] = 1.
            parameters['v_til0'] = 0.  # put exactly = 0.
            parameters['dtg_ha0'] = 0.  # otherwise you don't get 0./0.
            parameters['tdelta0'] = np.tan(self.prediction['state'][3])
        else:  # no need for l'Hopital's rule
            parameters['hop0'] = 0.
            parameters['dtg_ha0'] = -np.tan(self.prediction['state'][3])*parameters['v_til0']* \
                                           (1+parameters['tg_ha0']**2)**2/(2*self.length)
            # tdelta0 is only used when hop0 = 1, so no need to assign here
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines):
        v_til, tg_ha = splines[0], splines[1]
        dtg_ha = tg_ha.derivative(1)
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        x_int, y_int = self.T*running_integral(dx), self.T*running_integral(dy)
        x = x_int-evalspline(x_int, self.t/self.T) + self.pos0[0]
        y = y_int-evalspline(y_int, self.t/self.T) + self.pos0[1]
        self.define_collision_constraints_2d(hyperplanes, environment, [x, y], tg_ha)

    def splines2signals(self, splines, time):
        # for plotting and logging
        # note: here the splines are not dimensionless anymore
        signals = {}
        v_til, tg_ha = splines[0], splines[1]
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        ddtg_ha = tg_ha.derivative(2)
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
        # sample splines
        tg_ha = np.array(sample_splines([tg_ha], time))
        v_til = np.array(sample_splines([v_til], time))
        dtg_ha = np.array(sample_splines([dtg_ha], time))
        dv_til = np.array(sample_splines([dv_til], time))
        ddtg_ha = np.array(sample_splines([ddtg_ha], time))
        theta = 2*np.arctan2(tg_ha, 1)
        dtheta = 2*dtg_ha/(1+tg_ha**2)
        delta = np.arctan2(-2*dtg_ha*self.length, v_til*(1+tg_ha**2)**2)
        ddelta = -(2*ddtg_ha*self.length*(v_til*(1+tg_ha**2)**2)-2*dtg_ha*self.length*(dv_til*(1+tg_ha**2)**2 + v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha))/(v_til**2*(1+tg_ha**2)**4+(2*dtg_ha*self.length)**2)
        # check if you needed to use l'Hopital's rule to find delta and ddelta above, if so adapt signals
        # start
        if (v_til[0, 0] <= 1e-4 and dtg_ha[0, 0] <= 1e-4):
            delta[0, 0] = np.arctan2(-2*ddtg_ha[0, 0]*self.length , (dv_til[0, 0]*(1+tg_ha[0, 0]**2)**2))
            ddelta[0, 0] = ddelta[0, 1]  # choose next input
        # middle
        for k in range(1,len(time)-1):
            if (v_til[0, k] <= 1e-3 and dtg_ha[0, k] <= 1e-3):
                if (ddtg_ha[0, k] <= 1e-4 and dv_til[0, k] <= 1e-4):  # l'Hopital won't work
                    delta[0, k] = delta[0, k-1]  # choose previous steering angle
                else:
                    delta[0, k] = np.arctan2(-2*ddtg_ha[0, k]*self.length , (dv_til[0, k]*(1+tg_ha[0, k]**2)**2))
                ddelta[0, k] = ddelta[0, k-1]  # choose previous input
        # end
        if (v_til[0, -1] <= 1e-4 and dtg_ha[0, -1] <= 1e-4):  # correct at end point
            delta[0, -1] = delta[0, -2]
            ddelta[0, -1] = ddelta[0, -2]
        input = np.c_[v_til*(1+tg_ha**2)]  # V
        input = np.r_[input, ddelta]
        signals['state'] = np.c_[sample_splines([x, y], time)]
        signals['state'] = np.r_[signals['state'], theta, delta]
        signals['input'] = input
        signals['pose'] = signals['state'][:3]
        signals['delta'] = delta
        return signals

    def ode(self, state, input):
        # state: x, y, theta, delta
        # inputs: V, ddelta
        # find relation between dstate and state, inputs: dx = Ax+Bu
        # dstate = dx, dy, dtheta, ddelta
        # dstate[3] = input[1]
        u1, u2 = input[0], input[1]
        return np.r_[u1*np.cos(state[2]), u1*np.sin(state[2]), -u1/self.length*np.tan(state[3]) , u2].T

    def draw(self, t=-1):
        ret = []
        if self.options['plot_type'] is 'car':
            car = Rectangle(width=self.length, height=self.length/4.)
            wheel = Rectangle(width=self.length/8., height=self.length/10.)
            ret += car.draw(self.signals['pose'][:3, t])  # vehicle
            pos_front_l = self.signals['pose'][:2, t] + np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]+np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]+np.radians(30.))])
            pos_front_r = self.signals['pose'][:2, t] + np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]-np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]-np.radians(30.))])
            orient_front = self.signals['pose'][2, t]
            pos_back_l = self.signals['pose'][:2, t] - np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]+np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]+np.radians(30.))])
            pos_back_r = self.signals['pose'][:2, t] - np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]-np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]-np.radians(30.))])
            orient_back = self.signals['pose'][2, t] + self.signals['delta'][0, t]
            ret += wheel.draw(np.r_[pos_front_l, orient_front])  # front wheel left
            ret += wheel.draw(np.r_[pos_front_r, orient_front])  # front wheel right
            ret += wheel.draw(np.r_[pos_back_l, orient_back])  # back wheel left
            ret += wheel.draw(np.r_[pos_back_r, orient_back])  # back wheel right

        if self.options['plot_type'] is 'agv':
            # plot approximation of vehicle
            # real = Circle(self.length/2.)
            # ret += real.draw(self.signals['pose'][:2,t])
            car = Rectangle(width=self.length, height=self.length/4.)
            wheel = Rectangle(width=self.length/10., height=self.length/14.)
            ret += car.draw(self.signals['pose'][:3, t])  # vehicle
            pos_front = self.signals['pose'][:2, t] + (self.length/3)*np.array([np.cos(self.signals['pose'][2, t]),
                                                         np.sin(self.signals['pose'][2, t])])
            orient_front = self.signals['pose'][2, t]
            pos_back = self.signals['pose'][:2, t] - (self.length/3)*np.array([np.cos(self.signals['pose'][2, t]),
                                                         np.sin(self.signals['pose'][2, t])])
            orient_back = self.signals['pose'][2, t] + self.signals['delta'][0, t]
            ret += wheel.draw(np.r_[pos_front, orient_front])  # front wheel
            ret += wheel.draw(np.r_[pos_back, orient_back])  # back wheel
        return ret
