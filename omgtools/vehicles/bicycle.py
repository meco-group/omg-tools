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
from ..basics.spline_extra import sample_splines, definite_integral
from ..basics.spline_extra import evalspline, running_integral
from casadi import inf
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

    def __init__(self, shapes=Rectangle(width=0.4, height=0.1), options={}, bounds={}):
        Vehicle.__init__(
            self, n_spl=2, degree=2, shapes=shapes, options=options)
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        self.dmin = bounds['dmin'] if 'dmin' in bounds else -30.  # steering angle [deg]
        self.dmax = bounds['dmax'] if 'dmax' in bounds else 30.
        self.ddmin = bounds['ddmin'] if 'ddmin' in bounds else -45.  # dsteering angle [deg/s]
        self.ddmax = bounds['ddmax'] if 'ddmax' in bounds else 45.
        # time horizon
        self.T = self.define_symbol('T')  # motion time
        self.t = self.define_symbol('t')  # current time of first knot
        self.pos0 = self.define_symbol('pos0', 2)  # current position

    def set_default_options(self):
        Vehicle.set_default_options(self)

    def define_trajectory_constraints(self, splines):
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        ddtg_ha = tg_ha.derivative(2)
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        ddx, ddy = dx.derivative(), dy.derivative()
        self.define_constraint(
            (dx**2+dy**2) - (self.T**2)*self.vmax**2, -inf, 0.)
        self.define_constraint(
            (ddx**2+ddy**2) - (self.T**4)*self.amax**2, -inf, 0.)
        # limit steering angle
        for shape in self.shapes:
            self.define_constraint(
                 2*dtg_ha*shape.width - v_til*(1+tg_ha**2)**2*np.tan(self.dmax), -inf, 0.)
            self.define_constraint(
                 -2*dtg_ha*shape.width + v_til*(1+tg_ha**2)**2*np.tan(self.dmin), -inf, 0.)
        # limit rate of change of steering angle
            self.define_constraint(
                 2*shape.width*ddtg_ha*(v_til*(1+tg_ha**2)**2)
                 -2*shape.width*dtg_ha*(dv_til*(1+tg_ha**2)**2
                 +v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) - (v_til**2*(1+tg_ha**2)**4
                 +(2*shape.width*dtg_ha)**2)*self.ddmax*self.T, -inf, 0.)
            self.define_constraint(
                 -2*shape.width*ddtg_ha*(v_til*(1+tg_ha**2)**2)
                 +2*shape.width*dtg_ha*(dv_til*(1+tg_ha**2)**2
                 +v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) + (v_til**2*(1+tg_ha**2)**4
                 +(2*shape.width*dtg_ha)**2)*self.ddmin*self.T, -inf, 0.)
        self.define_constraint(-v_til, -inf, 0)  # positive v_tilde

        hop0 = self.define_symbol('hop0', 1)
        tdelta0 = self.define_symbol('tdelta0', 1)
        self.define_constraint((1-hop0)*(tdelta0*v_til(0)*(1+tg_ha(0)**2)**2 - 2*dtg_ha(0)*self.shapes[0].width), 0., 0.)
        self.define_constraint(hop0*(tdelta0*(dv_til(0)*(1+tg_ha(0)**2)**2+v_til(0)*(4*tg_ha(0)+4*tg_ha(0)**3)*dtg_ha(0))-(2*ddtg_ha(0)*self.shapes[0].width)), 0., 0.)

    def get_initial_constraints(self, splines):
        # these make sure you get continuity along different iterations
        # inputs are function of v_til, tg_ha and dtg_ha so impose constraints on these
        v_til0 = self.define_parameter('v_til0', 1)
        dv_til0 = self.define_parameter('dv_til0', 1)
        tg_ha0 = self.define_parameter('tg_ha0', 1)
        dtg_ha0 = self.define_parameter('dtg_ha0', 1)
        ddtg_ha0 = self.define_parameter('ddtg_ha0', 1)
        tdelta0 = self.define_parameter('tdelta0', 1)  # tan(delta)
        hop0 = self.define_parameter('hop0', 1)
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        ddtg_ha = tg_ha.derivative(2)
        return [(v_til, v_til0), (tg_ha, tg_ha0),
                (dv_til, dv_til0), (dtg_ha, self.T*dtg_ha0),
                (ddtg_ha, self.T**2*ddtg_ha0)]

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
        term_con_der = [(v_til, v_tilT), (dv_til, dv_tilT), 
                        (dtg_ha, self.T*dtg_haT), (ddtg_ha, self.T**2*ddtg_haT)]
        return [term_con, term_con_der]

    def set_initial_conditions(self, pose, input=np.zeros(2), ders=0.1*np.ones(3)):
        # comes from the user so theta is in deg
        pose[2] = np.radians(pose[2])  # theta
        pose[3] = np.radians(pose[3])  # delta
        self.prediction['state'] = pose  # x, y, theta[rad], delta[rad]
        self.pose0 = pose # for use in first iteration of splines2signals()
        self.prediction['input'] = input  # V, ddelta
        self.prediction['ders'] = ders  # dV

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
        if (np.linalg.norm(self.signals['state'][:3, -1] - self.poseT) > 1.e-3 or  # todo: kicked out state[3] sinc you cannot impose a steerT for now
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
        parameters['dtg_ha0'] = np.tan(self.prediction['state'][3])*parameters['v_til0']*(1+parameters['tg_ha0']**2)**2/(2*self.shapes[0].width)
        parameters['dv_til0'] = self.prediction['ders'][1]
        # parameters['dv_til0'] = (self.prediction['ders'][0] - 2*parameters['v_til0']*parameters['tg_ha0']*parameters['dtg_ha0'])/(1+parameters['tg_ha0']**2)
        # parameters['ddtg_ha0'] = (self.prediction['input'][1]*(parameters['v_til0']**2*(1+parameters['tg_ha0']**2)**4 + (2*parameters['dtg_ha0']*self.shapes[0].width)**2) + 2*parameters['dtg_ha0']*self.shapes[0].width*(parameters['dv_til0']*(1+parameters['tg_ha0']**2)**2+parameters['v_til0']*(4*parameters['tg_ha0']+4*parameters['tg_ha0']**3)*parameters['dtg_ha0']))/(2*self.shapes[0].width*(parameters['v_til0']*(1+parameters['tg_ha0']**2)**2))
        # parameters['ddtg_ha0'] = 1/(2*self.shapes[0].width)*(parameters['dv_til0']*(1+parameters['tg_ha0']**2)**2*np.tan(self.prediction['input'][1]) + parameters['v_til0']*(2*parameters['tg_ha0']*parameters['dtg_ha0'])*np.tan(self.prediction['input'][1]) + parameters['v_til0']*(1+parameters['tg_ha0']**2)*1/(np.cos(self.prediction['input'][1])))
        parameters['ddtg_ha0'] = self.prediction['ders'][2]
        parameters['pos0'] = self.prediction['state'][:2]
        parameters['posT'] = self.poseT[:2]  # x, y
        parameters['v_tilT'] = 0.
        parameters['dv_tilT'] = 0.
        parameters['tg_haT'] = np.tan(self.poseT[2]/2)
        parameters['dtg_haT'] = 0.
        parameters['ddtg_haT'] = 0.
        if (parameters['v_til0'] == 0.):  # use l'Hopital's rule
            parameters['hop0'] = 1.
            parameters['dtg_ha0'] = 0.  # otherwise you don't get 0./0.
            import pdb; pdb.set_trace()  # breakpoint 5eb9c596 //
            parameters['tdelta0'] = (2*parameters['ddtg_ha0']*self.shapes[0].width)/(parameters['dv_til0']*(1+parameters['tg_ha0']**2)**2 + parameters['v_til0']*(4*parameters['tg_ha0']+4*parameters['tg_ha0']**3)*parameters['dtg_ha0'])
        else:  # no need for l'Hopital's rule
            parameters['hop0'] = 0.
            parameters['tdelta0'] = (2*parameters['dtg_ha0']*self.shapes[0].width)/(parameters['v_til0']*(1+parameters['tg_ha0']**2)**2)
            # todo: add [0] at the end above?
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
        tg_ha = np.array(sample_splines([tg_ha], time))
        v_til = np.array(sample_splines([v_til], time))
        dtg_ha = np.array(sample_splines([dtg_ha], time))
        dv_til = np.array(sample_splines([dv_til], time))
        ddtg_ha = np.array(sample_splines([ddtg_ha], time))
        theta = 2*np.arctan2(tg_ha, 1)
        dtheta = 2*dtg_ha/(1+tg_ha)**2
        delta = np.arctan2(2*dtg_ha*self.shapes[0].width,v_til*(1+tg_ha**2)**2)
        ddelta = (2*ddtg_ha*self.shapes[0].width*(v_til*(1+tg_ha**2)**2)-2*dtg_ha*self.shapes[0].width*(dv_til*(1+tg_ha**2)**2 + v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha))/(v_til**2*(1+tg_ha**2)**4+(2*dtg_ha*self.shapes[0].width)**2)
        input = np.c_[v_til*(1+tg_ha**2)]  # V
        input = np.r_[input, ddelta]
        ders = np.r_[dv_til*(1+tg_ha**2)+2*v_til*tg_ha*dtg_ha, dv_til, ddtg_ha]  #dV
        signals['state'] = np.c_[sample_splines([x, y], time)]
        signals['state'] = np.r_[signals['state'], theta, delta]
        signals['input'] = input
        signals['ders'] = ders
        signals['pose'] = signals['state']
        signals['v_tot'] = input[0, :]
        return signals

    def ode(self, state, input):
        # state: x, y, theta, delta
        # inputs: V, ddelta
        # find relation between dstate and state, inputs: dx = Ax+Bu
        # dstate = dx, dy, dtheta, ddelta
        # dstate[3] = input[1]
        u1, u2 = input[0], input[1]
        return np.r_[u1*np.cos(state[2]), u1*np.sin(state[2]), u1/self.shapes[0].width*np.tan(state[3]) , u2].T

    def draw(self, t=-1):
        ret = []
        for shape in self.shapes:
            if isinstance(shape, Circle):
                wheel = Square(shape.radius/3)  # front and back
                ret += shape.draw(self.signals['pose'][:, t])
                ret += wheel.draw(self.signals['pose'][:, t]+
                                  (shape.radius/2)*np.array([np.cos(self.signals['pose'][2, t]+self.signals['pose'][3, t]),
                                                             np.sin(self.signals['pose'][2, t]+self.signals['pose'][3, t]),
                                                             self.signals['pose'][2, t]]))
                ret += wheel.draw(self.signals['pose'][:, t]+
                                  (shape.radius/2)*np.array([np.cos(self.signals['pose'][2, t]),
                                                             np.sin(self.signals['pose'][2, t]),
                                                             self.signals['pose'][2, t]]))
            else:
                ret += shape.draw(self.signals['pose'][:, t])
        return ret
