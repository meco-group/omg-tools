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
from ..basics.shape import Circle, Ring, Rectangle, Square
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np


class Tool(Vehicle):

    def __init__(self, tolerance, options=None, bounds=None):
        # Todo: for now tool is a 2D shape, with 3D splines, because
        # locally the movement is all in 2D, but sometimes there is a single
        # 3D movement... Make this more general
        # Don't make tool as large as tolerance, this removes all freedom
        self.shapes= [Circle(0.01*tolerance)]
        self.tolerance = tolerance
        bounds = bounds or {}
        # three movement directions (3 splines)
        # impose jerk limits (degree 3)
        Vehicle.__init__(
            self, n_spl=3, degree=3, shapes=self.shapes, options=options)

        # user specified separate velocities for x, y and z
        self.vxmin = bounds['vxmin'] if 'vxmin' in bounds else -0.5
        self.vymin = bounds['vymin'] if 'vymin' in bounds else -0.5
        self.vzmin = bounds['vzmin'] if 'vzmin' in bounds else -0.5
        self.vxmax = bounds['vxmax'] if 'vxmax' in bounds else 0.5
        self.vymax = bounds['vymax'] if 'vymax' in bounds else 0.5
        self.vzmax = bounds['vzmax'] if 'vzmax' in bounds else 0.5
        self.axmin = bounds['axmin'] if 'axmin' in bounds else -1.
        self.aymin = bounds['aymin'] if 'aymin' in bounds else -1.
        self.azmin = bounds['azmin'] if 'azmin' in bounds else -1.
        self.axmax = bounds['axmax'] if 'axmax' in bounds else 1.
        self.aymax = bounds['aymax'] if 'aymax' in bounds else 1.
        self.azmax = bounds['azmax'] if 'azmax' in bounds else 1.
        self.jxmin = bounds['jxmin'] if 'jxmin' in bounds else -2.
        self.jymin = bounds['jymin'] if 'jymin' in bounds else -2.
        self.jzmin = bounds['jzmin'] if 'jzmin' in bounds else -2.
        self.jxmax = bounds['jxmax'] if 'jxmax' in bounds else 2.
        self.jymax = bounds['jymax'] if 'jymax' in bounds else 2.
        self.jzmax = bounds['jzmax'] if 'jzmax' in bounds else 2.
        # user specified a single velocity for x, y and z
        if 'vmin' in bounds:
            self.vxmin = self.vymin  = self.vzmin = bounds['vmin']
        if 'vmax' in bounds:
            self.vxmax = self.vymax = self.vzmax = bounds['vmax']
        if 'amin' in bounds:
            self.axmin = self.aymin = self.azmin = bounds['amin']
        if 'amax' in bounds:
            self.axmax = self.aymax = self.azmax = bounds['amax']
        if 'jmin' in bounds:
            self.jxmin = self.jymin  = self.jzmin = bounds['jmin']
        if 'jmax' in bounds:
            self.jxmax = self.jymax = self.jzmax = bounds['jmax']

    def set_default_options(self):
        Vehicle.set_default_options(self)

    def define_trajectory_constraints(self, splines, horizon_time):
        x, y, z = splines
        dx, dy, dz = x.derivative(), y.derivative(), z.derivative()  # velocity
        ddx, ddy, ddz = x.derivative(2), y.derivative(2), z.derivative(2)  # acceleration
        dddx, dddy, dddz = x.derivative(3), y.derivative(3), z.derivative(3)  # jerk
        # constrain local velocity
        self.define_constraint(-dx + horizon_time*self.vxmin, -inf, 0.)
        self.define_constraint(-dy + horizon_time*self.vymin, -inf, 0.)
        # self.define_constraint(-dz + horizon_time*self.vzmin, -inf, 0.)
        self.define_constraint(dx - horizon_time*self.vxmax, -inf, 0.)
        self.define_constraint(dy - horizon_time*self.vymax, -inf, 0.)
        # self.define_constraint(dz - horizon_time*self.vzmax, -inf, 0.)

        self.define_constraint(-ddx + (horizon_time**2)*self.axmin, -inf, 0.)
        self.define_constraint(-ddy + (horizon_time**2)*self.aymin, -inf, 0.)
        # self.define_constraint(-ddz + (horizon_time**2)*self.azmin, -inf, 0.)
        self.define_constraint(ddx - (horizon_time**2)*self.axmax, -inf, 0.)
        self.define_constraint(ddy - (horizon_time**2)*self.aymax, -inf, 0.)
        # self.define_constraint(ddz - (horizon_time**2)*self.azmax, -inf, 0.)

        self.define_constraint(-dddx + (horizon_time**3)*self.jxmin, -inf, 0.)
        self.define_constraint(-dddy + (horizon_time**3)*self.jymin, -inf, 0.)
        # self.define_constraint(-dddz + (horizon_time**3)*self.jzmin, -inf, 0.)
        self.define_constraint(dddx - (horizon_time**3)*self.jxmax, -inf, 0.)
        self.define_constraint(dddy - (horizon_time**3)*self.jymax, -inf, 0.)
        # self.define_constraint(dddz - (horizon_time**3)*self.jzmax, -inf, 0.)

        self.define_constraint(z-1e-5, -inf,0.)
        self.define_constraint(-z-1e-5, -inf,0.)

        # don't reach maximum velocity at end of a segment, to get a more safe transition
        # self.define_constraint(-dx(1.) + 0.95*horizon_time*self.vxmin, -inf, 0.)
        # self.define_constraint(-dy(1.) + 0.95*horizon_time*self.vymin, -inf, 0.)
        # self.define_constraint(-dz(1.) + 0.95*horizon_time*self.vzmin, -inf, 0.)
        # self.define_constraint(dx(1.) - 0.95*horizon_time*self.vxmax, -inf, 0.)
        # self.define_constraint(dy(1.) - 0.95*horizon_time*self.vymax, -inf, 0.)
        # self.define_constraint(dz(1.) - 0.95*horizon_time*self.vzmax, -inf, 0.)

        # self.define_constraint(-ddx(1.) + 0.95*(horizon_time**2)*self.axmin, -inf, 0.)
        # self.define_constraint(-ddy(1.) + 0.95*(horizon_time**2)*self.aymin, -inf, 0.)
        # self.define_constraint(-ddz(1.) + 0.95*(horizon_time**2)*self.azmin, -inf, 0.)
        # self.define_constraint(ddx(1.) - 0.95*(horizon_time**2)*self.axmax, -inf, 0.)
        # self.define_constraint(ddy(1.) - 0.95*(horizon_time**2)*self.aymax, -inf, 0.)
        # self.define_constraint(ddz(1.) - 0.95*(horizon_time**2)*self.azmax, -inf, 0.)

    def get_initial_constraints(self, splines, horizon_time):
        state0 = self.define_parameter('state0', 3)
        input0 = self.define_parameter('input0', 3)
        dinput0 = self.define_parameter('dinput0', 3)
        x, y, z = splines
        dx, dy, dz = x.derivative(), y.derivative(), z.derivative()
        ddx, ddy, ddz = x.derivative(2), y.derivative(2), z.derivative(2)
        return [(x, state0[0]), (y, state0[1]), (z, state0[2]),
                (dx, horizon_time*input0[0]), (dy, horizon_time*input0[1]), (dz, horizon_time*input0[2]),
                (ddx, horizon_time**2*dinput0[0]), (ddy, horizon_time**2*dinput0[1]), (ddz, horizon_time**2*dinput0[2])]

    def get_terminal_constraints(self, splines):
        position = self.define_parameter('poseT', 3)
        x, y, z = splines
        term_con = [(x, position[0]), (y, position[1]), (z, position[2])]
        term_con_der = []
        for d in range(1, self.degree+1):
            term_con_der.extend([(x.derivative(d), 0.), (y.derivative(d), 0.), (z.derivative(d), 0.)])
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None, dinput=None, ddinput=None):
        if input is None:
            input = np.zeros(3)
        if dinput is None:
            dinput = np.zeros(3)
        if ddinput is None:
            ddinput = np.zeros(3)
        # list all predictions that are used in set_parameters
        self.prediction['state'] = state
        self.prediction['input'] = input
        self.prediction['dinput'] = dinput
        self.prediction['ddinput'] = ddinput

    def set_terminal_conditions(self, position):
        self.poseT = position

    def get_init_spline_value(self, subgoals=None):
        pos0 = self.prediction['state']
        posT = self.poseT
        if self.n_seg == 1:  # default
            init_value = np.zeros((len(self.basis), 3))
            for k in range(3):
                # init_value[:, k] = np.r_[pos0[k]*np.ones(self.degree), np.linspace(
                #     pos0[k], posT[k], len(self.basis) - 2*self.degree), posT[k]*np.ones(self.degree)]
                init_value[:, k] = np.linspace(pos0[k], posT[k], len(self.basis))
            init_value = [init_value]  # use same format as in n_seg > 1
        else:  # multiple segments
            if subgoals is None:
                raise AttributeError('No subgoal given, while there are multiple segments, '
                                    +'cannot compute initial guess')
            else:
                init_value = []
                subgoals.insert(0, pos0)  # add initial pos
                subgoals.append(posT)  # add goal pos
                for l in range(len(subgoals)-1):
                    init_val = np.zeros((len(self.basis), 3))
                    for k in range(3):
                        # init_value[:, k] = np.r_[pos0[k]*np.ones(self.degree), np.linspace(
                        #     pos0[k], posT[k], len(self.basis) - 2*self.degree), posT[k]*np.ones(self.degree)]
                        init_val[:, k] = np.linspace(subgoals[l][k], subgoals[l+1][k], len(self.basis))
                    init_value.append(init_val)
        return init_value

    def check_terminal_conditions(self):
        tol = self.options['stop_tol']
        if (np.linalg.norm(self.signals['state'][:, -1] - self.poseT) > tol or
                np.linalg.norm(self.signals['input'][:, -1])) > tol:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = Vehicle.set_parameters(self, current_time)
        parameters[self]['state0'] = self.prediction['state']
        parameters[self]['input0'] = self.prediction['input']
        parameters[self]['dinput0'] = self.prediction['dinput']
        parameters[self]['ddinput0'] = self.prediction['ddinput']
        parameters[self]['poseT'] = self.poseT
        return parameters

    def define_collision_constraints(self, segment, splines, horizon_time):
        # set up the constraints that ensure that spline stays inside segment shape
        x, y, z = splines
        position = [x, y]  # 2D for now
        # Todo: use 3D or 2D collision avoidance?

        # check room shape and orientation,
        # check vehicle shape and orientation
        # these determine the formulation of the collision avoidance constraints
        shape = self.shapes[0]  # tool shape
        shape_size = shape.radius  # tool size
        checkpoints, rad = shape.get_checkpoints()  # tool checkpoints
        if ((isinstance(segment['shape'], (Rectangle, Square)) and
            segment['shape'].orientation in [0.0, np.pi/2, np.pi, 2*np.pi, -np.pi/2, -np.pi, -2*np.pi]) and
            (isinstance(shape, Circle) or
            (isinstance(shape, (Rectangle, Square)) and
             shape.orientation == 0))):
            # we have a horizontal or vertical straight line segment and
            # a Circular or rectangular tool

            lims = segment['shape'].get_canvas_limits()
            room_limits = []
            room_limits += [lims[k]+segment['pose'][k] for k in range(self.n_dim)]
            for chck in checkpoints:
                for k in range(2):
                    self.define_constraint(-(chck[k]+position[k]) + room_limits[k][0] + rad[0], -inf, 0.)
                    self.define_constraint((chck[k]+position[k]) - room_limits[k][1] + rad[0], -inf, 0.)
        elif (isinstance(segment['shape'], (Rectangle, Square)) and
            (isinstance(shape, Circle))):
            # we have a diagonal line segment

            # in that case for any point [x, y] on the (infinite) line, the following equation must hold:
            # y = ax+b
            # and y - ax - b = 0
            # with a = the slope, and b = the offset
            # then we can relax this to <= tol and >= tol

            # Todo: now we impose that position must lie somewhere on the connection, not that it must lie between
            # start and end. Adapt?
            # Todo: use hyperplanes?
            # hyp_room = segment['shape'].get_hyperplanes(position = segment['position'])

            x1, y1, z1 = segment['start']
            x2, y2, z2 = segment['end']
            if x1 != x2:
                a = (y2-y1)/(x2-x1)
            else:
                raise ValueError('Trying to compute the slope of a vertical line,'
                               + ' impose constraints with alternative formulation')
            b = y1 - x1*a

            self.define_constraint(a*position[0] + b - position[1] - self.tolerance + shape_size, -inf, 0.)
            self.define_constraint(-a*position[0] - b + position[1] - self.tolerance + shape_size, -inf, 0.)
        elif (isinstance(segment['shape'], (Ring)) and
            (isinstance(shape, Circle))):
            # we have a ring/circle segment
            # we impose that the trajectory/splines must lie within the outside and inside circle

            # Todo: for now we impose that the trajectory must lie inside the complete ring, not that it
            # must lie inside the ring segment. So not sure that the solver picks the right arc...

            center = segment['pose']
            self.define_constraint(-(position[0] - center[0])**2 - (position[1] - center[1])**2 +
                                  (segment['shape'].radius_in + shape_size)**2, -inf, 0.)
            self.define_constraint((position[0] - center[0])**2 + (position[1] - center[1])**2 -
                                  (segment['shape'].radius_out - shape_size)**2, -inf, 0.)
        else:
            raise RuntimeError('Invalid segment obtained when setting up collision avoidance constraints')

        # constrain end point of segment[0] to lie inside a box around its desired end position, because the spline
        # must stay inside the inifinite line segment or complete circle, this can lead to problems if you don't constrain
        # the end position to a box (e.g. a trajectory going outside of the overlap region between segments)

        # Todo: combining this constraint with the fact that the spline must lie inside the tolerance band around the line/ring
        # should give good results?
        # build in a margin to take into account that the overlap region is not a rectangle
        # with orientation 0, this effect has a maximum when orientation is 45 degrees, and has effect of sqrt(2)~1.42
        self.define_constraint(position[0](1.) - segment['end'][0] - self.tolerance*1.42, -inf, 0.)
        self.define_constraint(-position[0](1.) + segment['end'][0] - self.tolerance*1.42, -inf, 0.)
        self.define_constraint(position[1](1.) - segment['end'][1] - self.tolerance*1.42, -inf, 0.)
        self.define_constraint(-position[1](1.) + segment['end'][1] - self.tolerance*1.42, -inf, 0.)


    def splines2signals(self, splines, time):
        signals = {}
        x, y, z = splines
        dx, dy, dz = x.derivative(), y.derivative(), z.derivative()
        ddx, ddy, ddz = x.derivative(2), y.derivative(2), z.derivative(2)
        dddx, dddy, dddz = x.derivative(3), y.derivative(3), z.derivative(3)
        input = np.c_[sample_splines([dx, dy, dz], time)]
        signals['state'] = np.c_[sample_splines([x, y, z], time)]
        signals['input'] = input
        signals['v_tot'] = np.sqrt(input[0, :]**2 + input[1, :]**2 + input[2, :]**2)
        signals['dinput'] = np.c_[sample_splines([ddx, ddy, ddz], time)]
        signals['ddinput'] = np.c_[sample_splines([dddx, dddy, dddz], time)]
        return signals

    def state2pose(self, state):
        return np.r_[state, np.zeros(3)]

    def ode(self, state, input):
        return input
