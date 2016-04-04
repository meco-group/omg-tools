# Implementation of 3 dof serial plate picker of LVD
# questions: ruben.vanparys@kuleuven.be

from vehicle import Vehicle
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np


class LVD(Vehicle):

    def __init__(self, shapes, options={}, bounds={}):
        Vehicle.__init__(
            self, n_spl=3, degree=3, shapes=shapes, options=options)
        self.smin = bounds['smin'] if 'smin' in bounds else [None, None, None]
        self.smax = bounds['smax'] if 'smax' in bounds else [None, None, None]
        self.vmin = bounds['vmin'] if 'vmin' in bounds else [-1.6, -0.5, -0.3]
        self.vmax = bounds['vmax'] if 'vmax' in bounds else [1.6,  0.5,  0.3]
        self.amin = bounds['amin'] if 'amin' in bounds else [-2.0, -2.0, -1.0]
        self.amax = bounds['amax'] if 'amax' in bounds else [2.0,  2.0,  1.0]
        self.jmin = bounds[
            'jmin'] if 'jmin' in bounds else [-4.0, -100.0, -5.0]
        self.jmax = bounds['jmax'] if 'jmax' in bounds else [4.0,  100.0,  5.0]
        # time horizon
        self.T = self.define_symbol('T')

    def define_trajectory_constraints(self, splines):
        x, y, z = splines
        dsplines = [s.derivative() for s in splines]
        ddsplines = [s.derivative(2) for s in splines]
        dddsplines = [s.derivative(3) for s in splines]

        for k, s in enumerate(splines):
            if self.smin[k] is not None:
                self.define_constraint(-s + self.smin[k], -inf, 0.)
            if self.smax[k] is not None:
                self.define_constraint(s - self.smax[k], -inf, 0.)
        for k, ds in enumerate(dsplines):
            self.define_constraint(-ds + self.T*self.vmin[k], -inf, 0.)
            self.define_constraint(ds - self.T*self.vmax[k], -inf, 0.)
        for k, dds in enumerate(ddsplines):
            self.define_constraint(-dds + (self.T**2)*self.amin[k], -inf, 0.)
            self.define_constraint(dds - (self.T**2)*self.amax[k], -inf, 0.)
        for k, ddds in enumerate(dddsplines):
            self.define_constraint(-ddds + (self.T**3)*self.jmin[k], -inf, 0.)
            self.define_constraint(ddds - (self.T**3)*self.jmax[k], -inf, 0.)

    def get_initial_constraints(self, splines):
        state0 = self.define_parameter('state0', 3)
        input0 = self.define_parameter('input0', 3)
        x, y, z = splines
        dx, dy, dz = x.derivative(), y.derivative(), z.derivative()
        return [(x, state0[0]), (y, state0[1]), (z, state0[2]),
                (dx, self.T*input0[0]), (dy, self.T*input0[1]),
                (dz, self.T*input0[2])]

    def get_terminal_constraints(self, splines):
        position = self.define_parameter('positionT', 3)
        x, y, z = splines
        dx, dy, dz = x.derivative(), y.derivative(), z.derivative()
        term_con = [(x, position[0]), (y, position[1]), (z, position[2])]
        term_con_der = [(dx, 0.), (dy, 0.), (dz, 0.)]
        return [term_con, term_con_der]

    def set_initial_conditions(self, position, input=np.zeros(3)):
        self.prediction['state'] = position
        self.prediction['input'] = input

    def set_terminal_conditions(self, position):
        self.positionT = position

    def get_init_spline_value(self):
        init_value = np.zeros((len(self.basis), 3))
        pos0 = self.prediction['state']
        posT = self.positionT
        breakpoint = self.options['knot_intervals']/3 + 1
        end_cont = 2
        init_value[:, 0] = np.r_[pos0[0]*np.ones(2+breakpoint), np.linspace(
            pos0[0], posT[0], len(self.basis) - end_cont - 2 - breakpoint), posT[0]*np.ones(end_cont)]
        init_value[:, 1] = np.r_[pos0[1]*np.ones(2), np.linspace(
            pos0[1], posT[1], len(self.basis) - end_cont - 2), posT[1]*np.ones(end_cont)]
        init_value[:, 2] = np.r_[pos0[2]*np.ones(2), np.linspace(pos0[2], 1.5*posT[2], breakpoint), np.linspace(
            1.5*posT[2], posT[2], len(self.basis)-breakpoint - end_cont - 2), posT[2]*np.ones(end_cont)]
        return init_value

    def check_terminal_conditions(self):
        if (np.linalg.norm(self.signals['state'][:, -1] - self.positionT) > 1.e-3 or
                np.linalg.norm(self.signals['input'][:, -1])) > 1.e-3:
            return False
        else:
            return True

    def set_parameters(self, current_time):
        parameters = {}
        parameters['state0'] = self.prediction['state']
        parameters['input0'] = self.prediction['input']
        parameters['positionT'] = self.positionT
        return parameters

    def define_collision_constraints(self, hyperplanes, room_lim, splines):
        x, y, z = splines[0], splines[1], splines[2]
        self.define_collision_constraints_3d(hyperplanes, room_lim, [x, y, z])

    def splines2signals(self, splines, time):
        signals = {}
        x, y, z = splines[0], splines[1], splines[2]
        dx, dy, dz = x.derivative(), y.derivative(), z.derivative()
        ddx, ddy, ddz = x.derivative(2), y.derivative(2), z.derivative(2)
        dddx, dddy, dddz = x.derivative(3), y.derivative(3), z.derivative(3)
        input = np.c_[sample_splines([dx, dy, dz], time)]
        signals['state'] = np.c_[sample_splines([x, y, z], time)]
        signals['input'] = input
        signals['pose'] = np.r_[signals['state'], np.zeros((3, len(time)))]
        signals['velocity'] = input
        signals['acceleration'] = np.c_[sample_splines([ddx, ddy, ddz], time)]
        signals['jerk'] = np.c_[sample_splines([dddx, dddy, dddz], time)]
        return signals

    def ode(self, state, input):
        return input
