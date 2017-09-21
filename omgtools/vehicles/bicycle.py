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
from ..problems.point2point import FreeTPoint2point, FixedTPoint2point
from ..basics.shape import Rectangle, Circle
from ..basics.spline_extra import sample_splines, evalspline, concat_splines
from ..basics.spline_extra import running_integral
from ..basics.spline import BSplineBasis
from casadi import inf, SX, MX
import numpy as np

# Elaboration of the vehicle model:
# dx = V*cos(theta)
# dy = V*sin(theta)
# dtheta = V/L*tan(delta) with L = distance between front and back wheel

# Use tangent half angle substitution: tg_ha = tan(theta/2)
# sin(theta) = (2*tg_ha)/(1+tg_ha**2)
# cos(theta) = (1-tg_ha**2)/(1+tg_ha**2)
# This gives:
# dx = V/(1+tg_ha**2)*(1-tg_ha**2)
# dy = V/(1+tg_ha**2)*(2*tg_ha)
# Substitute: v_til = V/(1+tg_ha**2)

# dx = v_til*(1-tg_ha**2)
# dy = v_til*(2*tg_ha)
# dtheta = v_til/L*(1+tg_ha**2)*tan(delta)

# And since theta = 2*arctan(tg_ha)
# dtheta = 2*dtg_ha/(1+tg_ha**2)
# So tan(delta) = (2*dtg_ha*L)/(v_til*(1+tg_ha**2)**2)
# Spline variables of the problem: v_til and tg_ha
# delta follows from v_til, tg_ha, dtg_ha


class Bicycle(Vehicle):

    def __init__(self, length=0.4, options=None, bounds=None):
        bounds = bounds or {}
        Vehicle.__init__(
            self, n_spl=2, degree=2, shapes=Circle(length/2.), options=options)
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.8
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        self.dmin = bounds['dmin'] if 'dmin' in bounds else -np.pi/6.  # steering angle [rad]
        self.dmax = bounds['dmax'] if 'dmax' in bounds else np.pi/6.
        self.ddmin = bounds['ddmin'] if 'ddmin' in bounds else -np.pi/4.  # dsteering angle [rad/s]
        self.ddmax = bounds['ddmax'] if 'ddmax' in bounds else np.pi/4.
        self.length = length

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'plot_type': 'bicycle'})  # by default plot a bicycle
        self.options.update({'substitution' : False})
        self.options.update({'exact_substitution' : False})

    def init(self):
        self.t = self.define_symbol('t')  # current time of first knot
        self.pos0 = self.define_symbol('pos0', 2)  # current position

    def define_trajectory_constraints(self, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        ddtg_ha = tg_ha.derivative(2)
        self.define_constraint(
            v_til*(1+tg_ha**2) - self.vmax, -inf, 0.)
        self.define_constraint(
            dv_til*(1+tg_ha**2) + 2*v_til*tg_ha*dtg_ha - horizon_time*self.amax, -inf, 0.)
        # Alternative:
        # dx = v_til*(1-tg_ha**2)
        # dy = v_til*(2*tg_ha)
        # ddx, ddy = dx.derivative(), dy.derivative()
        # self.define_constraint(
        #     (dx**2+dy**2) - (horizon_time**2)*self.vmax**2, -inf, 0.)
        # self.define_constraint(
        #     (ddx**2+ddy**2) - (horizon_time**4)*self.amax**2, -inf, 0.)

        if self.options['substitution']:
            # substitute velocity and introduce equality constraints
            dx = v_til*(1-tg_ha**2)
            dy = v_til*(2*tg_ha)
            if self.options['exact_substitution']:
                self.dx = self.define_spline_variable('dx', 1, 1, basis=dx.basis)[0]
                self.dy = self.define_spline_variable('dy', 1, 1, basis=dy.basis)[0]
                self.define_constraint(self.dx-dx, 0., 0.)
                self.define_constraint(self.dy-dy, 0., 0.)
                self.x = self.integrate_once(self.dx, self.pos0[0], self.t, horizon_time)
                self.y = self.integrate_once(self.dy, self.pos0[1], self.t, horizon_time)
            else:
                degree = 2
                knots = np.r_[np.zeros(degree), np.linspace(0., 1., 10+1), np.ones(degree)]
                basis = BSplineBasis(knots, degree)
                self.dx = self.define_spline_variable('dx', 1, 1, basis=basis)[0]
                self.dy = self.define_spline_variable('dy', 1, 1, basis=basis)[0]
                self.x = self.integrate_once(self.dx, self.pos0[0], self.t, horizon_time)
                self.y = self.integrate_once(self.dy, self.pos0[1], self.t, horizon_time)
                x = self.integrate_once(dx, self.pos0[0], self.t, horizon_time)
                y = self.integrate_once(dy, self.pos0[1], self.t, horizon_time)
                eps = 1e-2
                self.define_constraint(self.x-x, -eps, eps)
                self.define_constraint(self.y-y, -eps, eps)

        # limit steering angle
        self.define_constraint(
             2*dtg_ha*self.length - v_til*(1+tg_ha**2)**2*np.tan(self.dmax)*horizon_time, -inf, 0.)
        self.define_constraint(
             -2*dtg_ha*self.length + v_til*(1+tg_ha**2)**2*np.tan(self.dmin)*horizon_time, -inf, 0.)
        # limit rate of change of steering angle
        self.define_constraint(
             2*self.length*ddtg_ha*(v_til*(1+tg_ha**2)**2)
             -2*self.length*dtg_ha*(dv_til*(1+tg_ha**2)**2
             +v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) - ((horizon_time**2)*v_til**2*(1+tg_ha**2)**4
             +(2*self.length*dtg_ha)**2)*self.ddmax, -inf, 0.)
        self.define_constraint(
             -2*self.length*ddtg_ha*(v_til*(1+tg_ha**2)**2)
             +2*self.length*dtg_ha*(dv_til*(1+tg_ha**2)**2
             +v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) + ((horizon_time**2)*v_til**2*(1+tg_ha**2)**4
             +(2*self.length*dtg_ha)**2)*self.ddmin, -inf, 0.)
        self.define_constraint(-v_til, -inf, 0)  # model requires positive V, so positive v_tilde

    def get_initial_constraints(self, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
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
        self.define_constraint(hop0*(2.*evalspline(ddtg_ha, self.t/horizon_time)*self.length
                               -tdelta0*(evalspline(dv_til, self.t/horizon_time)*(1.+tg_ha0**2)**2)*horizon_time), 0., 0.)
        # This constraint is obtained by using l'Hopital's rule on the expression of
        # tan(delta) = 2*dtg_ha*self.length / (v_til*(1+tg_ha**2)**2)
        # this constraint is used to impose a specific steering angle when v_til = 0, e.g. when
        # starting from standstill. Note that a part of the denomerator is removed because
        # it contains evalspline(v_til, self.t/horizon_time)*... which is zero.
        # When v_til is not 0 the steering angle is implicitly imposed due to the fact that tan(delta)
        # is only a function of v_til, tg_ha, dtg_ha. If these variables are smooth the steering angle
        # will also be smooth. Furthermore the steering angle and its rate of change are limited by the
        # extra constraints above.

        # Impose final steering angle
        # tdeltaT = self.define_parameter('tdeltaT', 1)  # tan(delta)
        # tg_haT = self.define_parameter('tg_haT', 1)
        # self.define_constraint((2.*ddtg_ha(1.)*self.length
        #                        -tdeltaT*(dv_til(1.)*(1.+tg_haT**2)**2)*horizon_time), 0., 0.)

        # Further initial constraints are not necessary, these 3 are sufficient to get continuity of the state
        return [(v_til, v_til0), (tg_ha, tg_ha0),
                (dtg_ha, horizon_time*dtg_ha0)]

    def get_terminal_constraints(self, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
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
        if self.options['substitution']:
            x, y = self.x, self.y
        else:
            dx = v_til*(1-tg_ha**2)
            dy = v_til*(2*tg_ha)
            x = self.integrate_once(dx, self.pos0[0], self.t, horizon_time)
            y = self.integrate_once(dy, self.pos0[1], self.t, horizon_time)
            # x_int, y_int = horizon_time*running_integral(dx), horizon_time*running_integral(dy)
            # x = x_int-evalspline(x_int, self.t/horizon_time) + self.pos0[0]  # self.pos0 was already defined in init
            # y = y_int-evalspline(y_int, self.t/horizon_time) + self.pos0[1]
        term_con = [(x, posT[0]), (y, posT[1]), (tg_ha, tg_haT)]
        term_con_der = [(v_til, v_tilT), (dtg_ha, horizon_time*dtg_haT),
                        (dv_til, dv_tilT), (ddtg_ha, horizon_time**2*ddtg_haT)]
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None):
        if input is None:
            input = np.zeros(2)
        self.prediction['state'] = state
        self.prediction['input'] = input
        self.pose0 = state[:3]
        self.delta0 = state[3]

    def set_terminal_conditions(self, pose):
        self.poseT = pose

    def get_init_spline_value(self):
        # generate initial guess for spline variables
        init_value = np.zeros((len(self.basis), 2))
        v_til0 = np.zeros(len(self.basis))
        tg_ha0 = np.tan(self.prediction['state'][2]/2)
        tg_haT = np.tan(self.poseT[2]/2)
        init_value[:, 0] = v_til0
        init_value[:, 1] = np.linspace(tg_ha0, tg_haT, len(self.basis))
        init_value = [init_value]
        return init_value

    def check_terminal_conditions(self):
        # todo: kicked out state[3] since you cannot impose a steerT for now
        tol = self.options['stop_tol']
        if (np.linalg.norm(self.signals['state'][:3, -1] - self.poseT) > tol or
            np.linalg.norm(self.signals['input'][:, -1])) > tol:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        # for the optimization problem
        parameters = Vehicle.set_parameters(self, current_time)
        parameters[self]['tg_ha0'] = np.tan(self.prediction['state'][2]/2)
        parameters[self]['v_til0'] = self.prediction['input'][0]/(1+parameters[self]['tg_ha0']**2)
        parameters[self]['pos0'] = self.prediction['state'][:2]
        parameters[self]['posT'] = self.poseT[:2]  # x, y
        parameters[self]['v_tilT'] = 0.
        parameters[self]['dv_tilT'] = 0.
        parameters[self]['tg_haT'] = np.tan(self.poseT[2]/2)
        parameters[self]['dtg_haT'] = 0.
        parameters[self]['ddtg_haT'] = 0.
        # parameters['tdeltaT'] = np.tan(self.poseT[3])
        if (parameters[self]['v_til0'] <= 1e-4):  # use l'Hopital's rule
            parameters[self]['hop0'] = 1.
            parameters[self]['v_til0'] = 0.  # put exactly = 0.
            parameters[self]['dtg_ha0'] = 0.  # otherwise you don't get 0./0.
            parameters[self]['tdelta0'] = np.tan(self.prediction['state'][3])
        else:  # no need for l'Hopital's rule
            parameters[self]['hop0'] = 0.
            parameters[self]['dtg_ha0'] = np.tan(self.prediction['state'][3])*parameters[self]['v_til0']* \
                                           (1+parameters[self]['tg_ha0']**2)**2/(2*self.length)
            # tdelta0 is only used when hop0 = 1, so no need to assign here
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines, horizon_time=None):
        if horizon_time is None:
            horizon_time = self.define_symbol('T')  # motion time
        v_til, tg_ha = splines[0], splines[1]
        if self.options['substitution']:
            x, y = self.x, self.y
        else:
            dx = v_til*(1-tg_ha**2)
            dy = v_til*(2*tg_ha)
            x = self.integrate_once(dx, self.pos0[0], self.t, horizon_time)
            y = self.integrate_once(dy, self.pos0[1], self.t, horizon_time)
            # x_int, y_int = horizon_time*running_integral(dx), horizon_time*running_integral(dy)
            # x = x_int-evalspline(x_int, self.t/horizon_time) + self.pos0[0]
            # y = y_int-evalspline(y_int, self.t/horizon_time) + self.pos0[1]
        self.define_collision_constraints_2d(hyperplanes, environment, [x, y], tg_ha, horizon_time)

    def integrate_once(self, dx, x0, t, T=1.):
        dx_int = T*running_integral(dx)
        if isinstance(t, (SX, MX)):
            x = dx_int-evalspline(dx_int, t/T) + x0
        else:
            x = dx_int-dx_int(t/T) + x0
        return x

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
            x = self.integrate_once(dx, self.pose0[0], time[0])
            y = self.integrate_once(dy, self.pose0[1], time[0])
            # dx_int, dy_int = running_integral(dx), running_integral(dy)
            # x = dx_int - dx_int(time[0]) + self.pose0[0]
            # y = dy_int - dy_int(time[0]) + self.pose0[1]
        else:
            x = self.integrate_once(dx, self.signals['state'][0, -1], time[0])
            y = self.integrate_once(dy, self.signals['state'][1, -1], time[0])
            # dx_int, dy_int = running_integral(dx), running_integral(dy)  # current state
            # x = dx_int - dx_int(time[0]) + self.signals['state'][0, -1]
            # y = dy_int - dy_int(time[0]) + self.signals['state'][1, -1]
        # sample splines
        tg_ha = np.array(sample_splines([tg_ha], time))
        v_til = np.array(sample_splines([v_til], time))
        dtg_ha = np.array(sample_splines([dtg_ha], time))
        dv_til = np.array(sample_splines([dv_til], time))
        ddtg_ha = np.array(sample_splines([ddtg_ha], time))
        theta = 2*np.arctan2(tg_ha, 1)
        delta = np.arctan2(2*dtg_ha*self.length, v_til*(1+tg_ha**2)**2)
        ddelta = (2*ddtg_ha*self.length*(v_til*(1+tg_ha**2)**2)-2*dtg_ha*self.length*(dv_til*(1+tg_ha**2)**2 + v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha))/(v_til**2*(1+tg_ha**2)**4+(2*dtg_ha*self.length)**2)
        # check if you needed to use l'Hopital's rule to find delta and ddelta above, if so adapt signals
        # start
        if (v_til[0, 0] <= 1e-4 and dtg_ha[0, 0] <= 1e-4):
            delta[0, 0] = np.arctan2(2*ddtg_ha[0, 0]*self.length , (dv_til[0, 0]*(1+tg_ha[0, 0]**2)**2))
            ddelta[0, 0] = ddelta[0, 1]  # choose next input
        # middle
        for k in range(1,len(time)-1):
            if (v_til[0, k] <= 1e-3 and dtg_ha[0, k] <= 1e-3):
                if (ddtg_ha[0, k] <= 1e-4 and dv_til[0, k] <= 1e-4):  # l'Hopital won't work
                    delta[0, k] = delta[0, k-1]  # choose previous steering angle
                else:
                    delta[0, k] = np.arctan2(2*ddtg_ha[0, k]*self.length , (dv_til[0, k]*(1+tg_ha[0, k]**2)**2))
                ddelta[0, k] = ddelta[0, k-1]  # choose previous input
        # end
        if (v_til[0, -1] <= 1e-4 and dtg_ha[0, -1] <= 1e-4):  # correct at end point
            delta[0, -1] = delta[0, -2]
            ddelta[0, -1] = ddelta[0, -2]
        input = np.c_[v_til*(1+tg_ha**2)]  # V
        input = np.r_[input, ddelta]
        x_s, y_s = sample_splines([x, y], time)
        signals['state'] = np.c_[x_s, y_s].T
        signals['state'] = np.r_[signals['state'], theta, delta]
        signals['input'] = input
        signals['delta'] = delta

        if (self.options['substitution']):# and not self.options['exact_substitution']):  # don't plot error for exact_subs
            dx2 = self.problem.father.get_variables(self, 'dx')
            dy2 = self.problem.father.get_variables(self, 'dy')
            # select horizon_time
            if isinstance(self.problem, FreeTPoint2point):
                horizon_time = self.problem.father.get_variables(self.problem, 'T')[0][0]
            elif isinstance(self.problem, FixedTPoint2point):
                horizon_time = self.problem.options['horizon_time']
            dx2 = concat_splines([dx2], [horizon_time])[0]
            dy2 = concat_splines([dy2], [horizon_time])[0]
            if not hasattr(self, 'signals'): # first iteration
                x2 = self.integrate_once(dx2, self.pose0[0], time[0])
                y2 = self.integrate_once(dy2, self.pose0[1], time[0])
            else:
                x2 = self.integrate_once(dx2, self.signals['state'][0, -1], time[0])
                y2 = self.integrate_once(dy2, self.signals['state'][1, -1], time[0])
            dx_s, dy_s = sample_splines([dx, dy], time)
            x_s2, y_s2, dx_s2, dy_s2 = sample_splines([x2, y2, dx2, dy2], time)
            signals['err_dpos'] = np.c_[dx_s-dx_s2, dy_s-dy_s2].T
            signals['err_pos'] = np.c_[x_s-x_s2, y_s-y_s2].T
        return signals

    def state2pose(self, state):
        return state[:3]

    def ode(self, state, input):
        # state: x, y, theta, delta
        # inputs: V, ddelta
        # find relation between dstate and state, inputs: dx = Ax+Bu
        # dstate = dx, dy, dtheta, ddelta
        # dstate[3] = input[1]
        u1, u2 = input[0], input[1]
        return np.r_[u1*np.cos(state[2]), u1*np.sin(state[2]), u1/self.length*np.tan(state[3]) , u2].T

    def draw(self, t=-1):
        surfaces = []
        if self.options['plot_type'] is 'car':
            car = Rectangle(width=self.length, height=self.length/4.)
            wheel = Rectangle(self.length/8., self.length/10.)
            surfaces += car.draw(self.signals['pose'][:3, t])[0]  # vehicle
            pos_front_l = self.signals['pose'][:2, t] + np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]+np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]+np.radians(30.))])
            pos_front_r = self.signals['pose'][:2, t] + np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]-np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]-np.radians(30.))])
            orient_front = self.signals['pose'][2, t] + self.signals['delta'][0, t]
            pos_back_l = self.signals['pose'][:2, t] - np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]+np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]+np.radians(30.))])
            pos_back_r = self.signals['pose'][:2, t] - np.array([(self.length/2.5)*np.cos(self.signals['pose'][2, t]-np.radians(30.)),
                                                         (self.length/2.5)*np.sin(self.signals['pose'][2, t]-np.radians(30.))])
            orient_back = self.signals['pose'][2, t]
            surfaces += wheel.draw(np.r_[pos_front_l, orient_front])[0]  # front wheel left
            surfaces += wheel.draw(np.r_[pos_front_r, orient_front])[0]  # front wheel right
            surfaces += wheel.draw(np.r_[pos_back_l, orient_back])[0]  # back wheel left
            surfaces += wheel.draw(np.r_[pos_back_r, orient_back])[0]  # back wheel right

        if self.options['plot_type'] is 'bicycle':
            car = Rectangle(width=self.length, height=self.length/4.)
            wheel = Rectangle(self.length/2., self.length/6.)
            surfaces += car.draw(self.signals['pose'][:3, t])[0]  # vehicle
            pos_front = self.signals['pose'][:2, t] + (self.length/3)*np.array([np.cos(self.signals['pose'][2, t]),
                                                         np.sin(self.signals['pose'][2, t])])
            orient_front = self.signals['pose'][2, t] + self.signals['delta'][0, t]
            pos_back = self.signals['pose'][:2, t] - (self.length/3)*np.array([np.cos(self.signals['pose'][2, t]),
                                                         np.sin(self.signals['pose'][2, t])])
            orient_back = self.signals['pose'][2, t]
            surfaces += wheel.draw(np.r_[pos_front, orient_front])[0]  # front wheel
            surfaces += wheel.draw(np.r_[pos_back, orient_back])[0]  # back wheel
        return surfaces, []
