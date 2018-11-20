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

    def __init__(self, tolerance, options=None, bounds=None, **kwargs):
        # Todo: for now tool is a 2D shape, with 3D splines, because
        # locally the movement is all in 2D, but sometimes there is a single
        # 3D movement... Make this more general
        # Suppose that tool size was already taken into account in preprocessing step
        # GCode gives tool center point position
        self.shapes= [Circle(0)]
        self.tolerance = tolerance
        self.tolerance_small = kwargs['tol_small'] if 'tol_small' in kwargs else 0.
        bounds = bounds or {}
        # three movement directions --> 3 splines
        # impose jerk limits --> degree 3
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
        # user specified a single velocity for x and y
        if 'vmin' in bounds:
            self.vxmin = self.vymin = bounds['vmin']
        if 'vmax' in bounds:
            self.vxmax = self.vymax = bounds['vmax']
        if 'amin' in bounds:
            self.axmin = self.aymin = self.azmin = bounds['amin']
        if 'amax' in bounds:
            self.axmax = self.aymax = self.azmax = bounds['amax']
        if 'jmin' in bounds:
            self.jxmin = self.jymin = self.jzmin = bounds['jmin']
        if 'jmax' in bounds:
            self.jxmax = self.jymax = self.jzmax = bounds['jmax']

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'vel_limit':'machining', 'variable_tolerance':False})

    def define_trajectory_constraints(self, splines, horizon_time, skip=[]):
        x, y, z = splines
        dx, dy, dz = x.derivative(), y.derivative(), z.derivative()  # velocity
        ddx, ddy, ddz = x.derivative(2), y.derivative(2), z.derivative(2)  # acceleration
        dddx, dddy, dddz = x.derivative(3), y.derivative(3), z.derivative(3)  # jerk

        # constrain local velocity
        if self.options['vel_limit'] is 'machining':
            # the machining process is the limiting factor, so limit the total velocity
            # in x- and y-axis combined
            if self.vxmax != 0.:
                # xy-plane movement
                self.define_constraint(
                    (dx**2+dy**2) - (horizon_time**2)*self.vxmax**2, -inf, 0., skip=skip)
            else:
                # z-movement
                self.define_constraint(
                    (dz**2) - (horizon_time**2)*self.vzmax**2, -inf, 0., skip=skip)

        elif self.options['vel_limit'] is 'axes':
            # the axes themselves are the limiting factor, so limit the x- and y- axis separately
            self.define_constraint(-dx + horizon_time*self.vxmin, -inf, 0., skip=skip)
            self.define_constraint(-dy + horizon_time*self.vymin, -inf, 0., skip=skip)
            self.define_constraint(-dz + horizon_time*self.vzmin, -inf, 0., skip=skip)
            self.define_constraint(dx - horizon_time*self.vxmax, -inf, 0., skip=skip)
            self.define_constraint(dy - horizon_time*self.vymax, -inf, 0., skip=skip)
            self.define_constraint(dz - horizon_time*self.vzmax, -inf, 0., skip=skip)
        else:
            raise ValueError(
                'Only machining and axes are defined as velocity limit types.')

        self.define_constraint(-ddx + (horizon_time**2)*self.axmin, -inf, 0., skip=skip)
        self.define_constraint(-ddy + (horizon_time**2)*self.aymin, -inf, 0., skip=skip)
        self.define_constraint(-ddz + (horizon_time**2)*self.azmin, -inf, 0., skip=skip)
        self.define_constraint(ddx - (horizon_time**2)*self.axmax, -inf, 0., skip=skip)
        self.define_constraint(ddy - (horizon_time**2)*self.aymax, -inf, 0., skip=skip)
        self.define_constraint(ddz - (horizon_time**2)*self.azmax, -inf, 0., skip=skip)

        # remove for corner benchmarks
        self.define_constraint(-dddx + (horizon_time**3)*self.jxmin, -inf, 0.)
        self.define_constraint(-dddy + (horizon_time**3)*self.jymin, -inf, 0.)
        self.define_constraint(-dddz + (horizon_time**3)*self.jzmin, -inf, 0.)
        self.define_constraint(dddx - (horizon_time**3)*self.jxmax, -inf, 0.)
        self.define_constraint(dddy - (horizon_time**3)*self.jymax, -inf, 0.)
        self.define_constraint(dddz - (horizon_time**3)*self.jzmax, -inf, 0.)

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
        for d in range(1, self.degree):
        # for d in range(2, self.degree):
            term_con_der.extend([(x.derivative(d), 0.), (y.derivative(d), 0.), (z.derivative(d), 0.)])
        return [term_con, term_con_der]

    def set_initial_conditions(self, state, input=None, dinput=None, ddinput=None):
        if input is None:
            input = np.zeros(3)
            # input[0] = 25.
        if dinput is None:
            dinput = np.zeros(3)
        if ddinput is None:
            ddinput = np.zeros(3)
        # list all predictions that are used in set_parameters
        self.prediction['state'] = state
        self.prediction['input'] = input
        self.prediction['dinput'] = dinput

    def set_terminal_conditions(self, position):
        self.poseT = position

    def check_terminal_conditions(self):
        tol = self.options['stop_tol']
        if (np.linalg.norm(self.signals['state'][:, -1] - self.poseT) > tol or
                np.linalg.norm(self.signals['input'][:, -1]) > tol):
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = Vehicle.set_parameters(self, current_time)
        parameters[self]['state0'] = self.prediction['state']
        parameters[self]['input0'] = self.prediction['input']
        parameters[self]['dinput0'] = self.prediction['dinput']
        parameters[self]['poseT'] = self.poseT
        return parameters

    def define_collision_constraints(self, segment, splines, horizon_time):
        # set up the constraints that ensure that spline stays inside segment shape
        x, y, z = splines
        position = [x, y]  # collision avoidance in xy-plane (2D)

        # check room shape and orientation,
        # check vehicle shape and orientation
        # these determine the formulation of the collision avoidance constraints
        shape = self.shapes[0]  # tool shape
        checkpoints, rad = shape.get_checkpoints()  # tool checkpoints, rad = shape_size
        # used 2*rad[0] below to get a larger margin and avoid numerical errors when checking if tool is inside shape
        if (isinstance(segment['shape'], (Rectangle, Square)) and
            ((segment['shape'].orientation) % (np.pi/2) == 0) and
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

            # -tol <= a'*q - b <= tol
            # with b the offset,a the normalized normal vector, and q = [x,y]'

            x1, y1, z1 = segment['start']
            x2, y2, z2 = segment['end']
            tolerance = segment['shape'].height*0.5

            vector = [x2-x1, y2-y1]  # vector from end to start
            a = np.array([-vector[1],vector[0]])*(1/np.sqrt(vector[0]**2+vector[1]**2))  # normalized normal vector
            b = np.dot(a,np.array([x1, y1]))  # offset

            self.define_constraint(a[0]*position[0] + a[1]*position[1] - b - tolerance + rad[0], -inf, 0.)
            self.define_constraint(-a[0]*position[0] - a[1]*position[1] + b - tolerance + rad[0], -inf, 0.)

        elif (isinstance(segment['shape'], (Ring)) and
            (isinstance(shape, Circle))):
            # we have a Ring/Circle segment
            # we impose that the trajectory/splines must lie within the outside and inside circle

            # Todo: constraint imposes that the trajectory must lie inside the complete ring, not that it
            # may only lie inside the ring segment. Improve?

            center = segment['pose']
            self.define_constraint(-(position[0] - center[0])**2 - (position[1] - center[1])**2 +
                                  (segment['shape'].radius_in + rad[0])**2, -inf, 0.)
            self.define_constraint((position[0] - center[0])**2 + (position[1] - center[1])**2 -
                                  (segment['shape'].radius_out - rad[0])**2, -inf, 0.)
        else:
            raise RuntimeError('Invalid segment obtained when setting up collision avoidance constraints')

        # collision avoidance in z-direction: stay within connection from start to end, with a little margin
        if segment['start'][2] != segment['end'][2]:
            z_min = min(segment['start'][2],segment['end'][2])
            z_max = max(segment['start'][2],segment['end'][2])
            # movement in z-direction
            self.define_constraint(-z + z_min - rad[0], -inf, 0.)
            self.define_constraint(z - z_max  - rad[0], -inf, 0.)

        # Constraints above impose that the spline must stay inside the infinite version of the segment: the
        # complete line, or the complete ring. This can cause a trajectory that is feasible, but still goes outside
        # of the overlap region between two segments. Therefore, we can constrain the end point of segment[0] to lie
        # inside a box around its desired end position. If we choose the box small enough, we get the guarantee
        # that the connection point stays inside the overlap region. However, choosing this box too small may lead to
        # suboptimal results. The overlap region mostly is a kind of rotated rectangle. When the orientation is
        # 45 degrees, the effect is sqrt(2)~1.42. To take this into account we do self.tolerance*'factor'.
        # A good value for 'factor' depends on the tightness of the tolerances: tight tolerance = small 'factor',
        # larger tolerances = larger 'factor'.

        # Warning: the value of this 'factor' may have a big influence on the total machining time!

        # However, simulations show that this box constraint is in general only necessary
        # when using variable tolerances, explaining the if-check below.

        if self.options['variable_tolerance']:
            self.define_constraint(position[0](1.) - segment['end'][0] - self.tolerance*0.9, -inf, 0.)
            self.define_constraint(-position[0](1.) + segment['end'][0] - self.tolerance*0.9, -inf, 0.)
            self.define_constraint(position[1](1.) - segment['end'][1] - self.tolerance*0.9, -inf, 0.)
            self.define_constraint(-position[1](1.) + segment['end'][1] - self.tolerance*0.9, -inf, 0.)

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