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
from ..basics.shape import Square, Circle
from ..basics.spline_extra import sample_splines
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

    def __init__(self, shapes=Circle(0.1), options=None, bounds=None):
        bounds = bounds or {}
        if options is not None and 'degree' in options:
            degree = options['degree']
        else:
            degree = 3
        Vehicle.__init__(
            self, n_spl=2, degree=degree, shapes=shapes, options=options)
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amax = bounds['amax'] if 'amax' in bounds else 1.
        self.wmin = bounds['wmin'] if 'wmin' in bounds else -np.pi/6. # in rad/s
        self.wmax = bounds['wmax'] if 'wmax' in bounds else np.pi/6.

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options['stop_tol'] = 1.e-2

    def init(self):
        self.T = self.define_symbol('T')  # motion time
        self.t = self.define_symbol('t')  # current time of first knot
        self.pos0 = self.define_parameter('pos0', 2)  # current position

    def define_trajectory_constraints(self, splines):
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        self.define_constraint(
            v_til*(1+tg_ha**2) - self.vmax, -inf, 0.)
        # self.define_constraint(-v_til*(1+tg_ha**2) - self.vmax, -inf, 0)  # only forward driving

        # self.define_constraint(
        #     dv_til*(1+tg_ha**2) + 2*v_til*tg_ha*dtg_ha - self.T*self.amax, -inf, 0.)

        # dx = v_til*(1-tg_ha**2)
        # dy = v_til*(2*tg_ha)        
        # self.dx = self.define_spline_variable('dx', 1, 1, basis=dx.basis)[0]
        # self.dy = self.define_spline_variable('dy', 1, 1, basis=dy.basis)[0]

        # x = self.T*running_integral(dx)
        # y = self.T*running_integral(dy)
        # self.x = self.T*running_integral(self.dx)
        # self.y = self.T*running_integral(self.dy)

        # eps = 1e-3
        # self.define_constraint(self.x-x, -eps, eps)
        # self.define_constraint(self.y-y, -eps, eps)

        # Alternative:
        # dx = v_til*(1-tg_ha**2)
        # dy = v_til*(2*tg_ha)
        # ddx, ddy = dx.derivative(), dy.derivative()
        # self.define_constraint(
        #     (dx**2+dy**2) - (self.T**2)*self.vmax**2, -inf, 0.)
        # self.define_constraint(
        #     (ddx**2+ddy**2) - (self.T**4)*self.amax**2, -inf, 0.)

        # add constraints on change in orientation
        self.define_constraint(2*dtg_ha - (1+tg_ha**2)*self.T*self.wmax, -inf, 0.)
        self.define_constraint(-2*dtg_ha + (1+tg_ha**2)*self.T*self.wmin, -inf, 0.)

        # self.define_constraint(-v_til, -inf, 0)  # only forward driving, positive v_tilde


    def get_fleet_center(self, splines, rel_pos, substitute=True):
        T = self.define_symbol('T')
        t = self.define_symbol('t')
        pos0 = self.define_parameter('pos0', 2)
        v_til, tg_ha = splines
        dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        x_int, y_int = T*running_integral(dx), T*running_integral(dy)
        x = x_int-evalspline(x_int, t/T) + pos0[0]
        y = y_int-evalspline(y_int, t/T) + pos0[1]
        eps = 1.e-2
        center = self.define_spline_variable('formation_center', self.n_dim)
        self.define_constraint((x-center[0])*(1+tg_ha**2) + rel_pos[0]*2*tg_ha + rel_pos[1]*(1-tg_ha**2), -eps, eps)
        self.define_constraint((y-center[1])*(1+tg_ha**2) + rel_pos[1]*2*tg_ha - rel_pos[0]*(1-tg_ha**2), -eps, eps)
        for d in range(1, self.degree+1):
            for c in center:
                self.define_constraint(c.derivative(d)(1.), 0., 0.)
        return center
        # center = []
        # if substitute:
        #     center_tf = [x*(1+tg_ha**2) + rel_pos[0]*2*tg_ha + rel_pos[1]*(1-tg_ha**2), y*(1+tg_ha**2) + rel_pos[1]*2*tg_ha - rel_pos[0]*(1-tg_ha**2)]
        #     center = self.define_substitute('fleet_center', center_tf)
        #     center.append(tg_ha)
        #     return center
        # else:
        #     if splines[0] is (MX, SX):
        #         center_tf = [x*(1+tg_ha**2) + rel_pos[0]*2*tg_ha + rel_pos[1]*(1-tg_ha**2), y*(1+tg_ha**2) + rel_pos[1]*2*tg_ha - rel_pos[0]*(1-tg_ha**2)]
        #         center_tf.append(tg_ha)
        #         return center_tf

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
        v_til, tg_ha = splines
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        x_int, y_int = self.T*running_integral(dx), self.T*running_integral(dy)
        x = x_int-evalspline(x_int, self.t/self.T) + self.pos0[0]  # self.pos0 was already defined in init
        y = y_int-evalspline(y_int, self.t/self.T) + self.pos0[1]
        term_con = [(x, posT[0]), (y, posT[1]), (tg_ha, tg_haT)]
        term_con_der = [(v_til, 0.), (tg_ha.derivative(), 0.)]
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None):
        if input is None:
            input = np.zeros(2)
        self.prediction['state'] = state
        self.prediction['input'] = input
        self.pose0 = state

    def set_terminal_conditions(self, pose):
        self.poseT = pose

    def get_init_spline_value(self):
        # generate initial guess for spline variables
        init_value = np.zeros((len(self.basis), 2))
        v_til0 = np.zeros(len(self.basis))
        tg_ha0 = np.tan(self.prediction['state'][2]/2.)
        tg_haT = np.tan(self.poseT[2]/2.)
        init_value[:, 0] = v_til0
        init_value[:, 1] = np.linspace(tg_ha0, tg_haT, len(self.basis))
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
        parameters['tg_ha0'] = np.tan(self.prediction['state'][2]/2.)
        parameters['v_til0'] = self.prediction['input'][0]/(1+parameters['tg_ha0']**2)
        parameters['dtg_ha0'] = 0.5*self.prediction['input'][1]*(1+parameters['tg_ha0']**2)  # dtg_ha
        parameters['pos0'] = self.prediction['state'][:2]
        parameters['posT'] = self.poseT[:2]  # x,y
        parameters['tg_haT'] = np.tan(self.poseT[2]/2.)
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines):
        v_til, tg_ha = splines[0], splines[1]
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
        x_s, y_s, v_til_s, tg_ha_s, dtg_ha_s = sample_splines([x, y, v_til, tg_ha, dtg_ha], time)
        den = sample_splines([(1+tg_ha**2)], time)[0]
        theta = 2*np.arctan2(tg_ha_s,1)
        dtheta = 2*np.array(dtg_ha_s)/(1.+np.array(tg_ha_s)**2)
        v_s = v_til_s*den
        signals['state'] = np.c_[x_s, y_s, theta.T].T
        signals['input'] = np.c_[v_s, dtheta.T].T
        if hasattr(self, 'rel_pos_c'):
            x_c = x_s + sample_splines([self.rel_pos_c[0]*2*tg_ha + self.rel_pos_c[1]*(1-tg_ha**2)], time)[0]/den
            y_c = y_s + sample_splines([self.rel_pos_c[1]*2*tg_ha - self.rel_pos_c[0]*(1-tg_ha**2)], time)[0]/den
            signals['fleet_center'] = np.c_[x_c, y_c].T
        return signals

    def state2pose(self, state):
        return state

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
                wheel = Square(shape.radius/3.)
                front = Circle(shape.radius/8.)
                ret += shape.draw(self.signals['pose'][:, t])
                ret += wheel.draw(self.signals['pose'][:, t]+
                                  (shape.radius/2.)*np.array([np.cos(self.signals['pose'][2, t]-np.pi/2.),
                                                             np.sin(self.signals['pose'][2, t]-np.pi/2.),
                                                             self.signals['pose'][2, t]]))
                ret += wheel.draw(self.signals['pose'][:, t]+
                                  (shape.radius/2.)*np.array([np.cos(self.signals['pose'][2, t]+np.pi/2.),
                                                             np.sin(self.signals['pose'][2, t]+np.pi/2.),
                                                             self.signals['pose'][2, t]]))
                ret += front.draw(self.signals['pose'][:, t]+
                                  (shape.radius/1.5)*np.array([np.cos(self.signals['pose'][2, t]),
                                                               np.sin(self.signals['pose'][2, t]),
                                                               self.signals['pose'][2, t]]))
            else:
                ret += shape.draw(self.signals['pose'][:, t])
        return ret

    def get_pos_splines(self, splines):
        T = self.define_symbol('T')  # motion time
        t = self.define_symbol('t')  # current time of first knot
        pos0 = self.define_parameter('pos0', 2)  # current position
        v_til, tg_ha = splines
        dx = v_til*(1-tg_ha**2)
        dy = v_til*(2*tg_ha)
        x_int, y_int = T*running_integral(dx), T*running_integral(dy)
        x = x_int-evalspline(x_int, t/T) + pos0[0]  # self.pos0 was already defined in init
        y = y_int-evalspline(y_int, t/T) + pos0[1]
        return [x, y]

    # Next two functions are required if vehicle is not passed to problem, but is still used in the optimization
    # problem e.g. when considering a vehicle with a trailer. You manually have to update signals and prediction,
    # here the inputs are coming from e.g. the trailer class.
    def update_signals(self, signals):
        self.signals = signals

    def update_prediction(self, prediction):
        self.prediction = prediction
