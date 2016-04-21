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


from ..basics.optilayer import OptiChild
from ..basics.spline_extra import get_interval_T
from ..basics.spline import BSplineBasis, BSpline
from casadi import inf, vertcat, cos, sin
from scipy.interpolate import interp1d
from scipy.integrate import odeint
import numpy as np


class Obstacle(object):

    def __new__(cls, initial, shape, simulation={}, options={}):
        if shape.n_dim == 2:
            return Obstacle2D(initial, shape, simulation, options)
        if shape.n_dim == 3:
            return Obstacle3D(initial, shape, simulation, options)


class ObstaclexD(OptiChild):

    def __init__(self, initial, shape, simulation, options):
        OptiChild.__init__(self, 'obstacle')
        self.simulation = simulation
        self.set_default_options()
        self.set_options(options)
        self.shape = shape
        self.n_dim = shape.n_dim
        self.basis = BSplineBasis([0, 0, 0, 1, 1, 1], 2)
        self.initial = initial
        self.prepare_simulation(initial, simulation)
        self.A = np.array([[0., 1., 0.], [0., 0., 1.], [0., 0., 0.]])
        self.create_splines()

    # ========================================================================
    # Obstacle options
    # ========================================================================

    def set_default_options(self):
        self.options = {'draw': True, 'avoid': True}

    def set_options(self, options):
        self.options.update(options)

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def create_splines(self):
        # pos, vel, acc
        x = self.define_parameter('x', self.n_dim)
        v = self.define_parameter('v', self.n_dim)
        a = self.define_parameter('a', self.n_dim)
        # pos, vel, acc at time zero of time horizon
        self.t = self.define_symbol('t')
        self.T = self.define_symbol('T')
        v0 = v - self.t*a
        x0 = x - self.t*v0 - 0.5*(self.t**2)*a
        a0 = a
        # pos spline over time horizon
        self.pos_spline = [BSpline(self.basis, vertcat(x0[k], 0.5*v0[k]*self.T + x0[k], x0[k] + v0[k]*self.T + 0.5*a0[k]*(self.T**2)))
                           for k in range(self.n_dim)]

    def define_collision_constraints(self, hyperplanes):
        raise ValueError('Please implement this method.')

    def set_parameters(self, current_time):
        parameters = {}
        parameters['x'] = self.signals['position'][:, -1]
        parameters['v'] = self.signals['velocity'][:, -1]
        parameters['a'] = self.signals['acceleration'][:, -1]
        return parameters

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def prepare_simulation(self, initial, simulation):
        # simulation model
        A = np.kron(
            np.array([[0., 1., 0.], [0., 0., 1.], [0., 0., 0.]]), np.eye(self.n_dim))
        B = np.zeros((3*self.n_dim, 1.))
        self.simulation_model = {'A': A, 'B': B}
        if 'model' in simulation:
            if 'A' in simulation['model']:
                self.simulation_model['A'] = simulation['model']['A']
            if 'B' in simulation['model']:
                self.simulation_model['B'] = simulation['model']['B']
        # trajectories
        trajectories = {}
        trajectories['input'] = {'time': np.array([0., 1.]),
                                 'values': np.zeros((self.simulation_model['B'].shape[1], 2))}
        for key in ['position', 'velocity', 'acceleration']:
            trajectories[key] = {'time': np.array([0., 1.]),
                                 'values': np.zeros((self.n_dim, 2))}
        if 'trajectories' in simulation:
            for key, trajectory in simulation['trajectories'].items():
                trajectories[key]['time'] = np.array(
                    simulation['trajectories'][key]['time'])
                trajectories[key]['values'] = np.vstack(
                    simulation['trajectories'][key]['values']).T
                if trajectories[key]['time'].size != trajectories[key]['values'].shape[1]:
                    raise ValueError('Dimension mismatch between time array ' +
                                     'and values for ' + key + ' trajectory.')
        # create interpolation functions
        self.input_interp = interp1d(trajectories['input']['time'],
                                     trajectories['input']['values'],
                                     kind='linear', bounds_error=False,
                                     fill_value=trajectories['input']['values'][:, -1])
        state = np.zeros((3*self.n_dim, 1.))
        time_state = np.array([0.])
        for l, key in enumerate(['position', 'velocity', 'acceleration']):
            for k, time in enumerate(trajectories[key]['time']):
                index = np.where(time_state == time)[0]
                if index.size == 0:
                    time_state = np.r_[time_state, time]
                    state_k = np.zeros((3*self.n_dim))
                    state_k[
                        l*self.n_dim:(l+1)*self.n_dim] += trajectories[key]['values'][:, k]
                    state = np.c_[state, state_k]
                else:
                    index = index[0]
                    state[
                        l*self.n_dim:(l+1)*self.n_dim, index] += trajectories[key]['values'][:, k]
        ind_sorted = np.argsort(time_state)
        state_incr = np.cumsum(state[:, ind_sorted], axis=1)
        time_state = time_state[ind_sorted]
        self.state_incr_interp = interp1d(time_state, state_incr, kind='zero',
                                          bounds_error=False,
                                          fill_value=state_incr[:, -1])
        # initialize signals
        self.signals = {}
        self.signals['time'] = np.array([0.])
        for key in ['position', 'velocity', 'acceleration']:
            if key in initial:
                self.signals[key] = np.c_[initial[key]]
            else:
                self.signals[key] = np.zeros((self.n_dim, 1))

    def _ode(self, state, time):
        input = self.input_interp(time)
        state_incr = self.state_incr_interp(time)
        return self.ode(state, input) + np.r_[state_incr[self.n_dim:],
                                              np.zeros(self.n_dim)]

    def ode(self, state, input):
        A = self.simulation_model['A']
        B = self.simulation_model['B']
        return A.dot(state) + B.dot(input)

    def update(self, update_time, sample_time):
        n_samp = int(update_time/sample_time)+1
        time0 = self.signals['time'][-1]
        time_axis = np.linspace(time0, (n_samp-1)*sample_time+time0, n_samp)
        state0 = np.r_[self.signals['position'][:, -1],
                       self.signals['velocity'][:, -1],
                       self.signals['acceleration'][:, -1]].T
        if time0 != 0.0:
            state0 -= self.state_incr_interp(time0)
        state = odeint(self._ode, state0, time_axis).T
        state += self.state_incr_interp(time_axis)
        self.signals['position'] = np.c_[self.signals['position'],
                                         state[:self.n_dim, 1:n_samp+1]]
        self.signals['velocity'] = np.c_[self.signals['velocity'],
                                         state[self.n_dim:2*self.n_dim, 1:n_samp+1]]
        self.signals['acceleration'] = np.c_[self.signals['acceleration'],
                                             state[2*self.n_dim:3*self.n_dim, 1:n_samp+1]]
        self.signals['time'] = np.r_[
            self.signals['time'], time_axis[1:n_samp+1]]

    def draw(self, t=-1):
        if not self.options['draw']:
            return []
        pose = np.zeros(2*self.n_dim)
        pose[:self.n_dim] = self.signals['position'][:, t]
        return self.shape.draw(pose)


class Obstacle2D(ObstaclexD):

    def __init__(self, initial, shape, simulation, options):
        ObstaclexD.__init__(self, initial, shape, simulation, options)

    # ========================================================================
    # Obstacle options
    # ========================================================================

    def set_default_options(self):
        ObstaclexD.set_default_options(self)
        # horizon time (for nurbs representation of cos & sin)
        self.options['horizon_time'] = None

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def create_splines(self):
        ObstaclexD.create_splines(self)
        if self.signals['angular_velocity'][:, -1] == 0.:
            self.cos = cos(self.signals['orientation'][:, -1][0])
            self.sin = sin(self.signals['orientation'][:, -1][0])
            self.gon_weight = 1.
            return
        # theta, omega
        theta = self.define_parameter('theta', 1)
        omega = self.signals['angular_velocity'][:, -1][0]
        # theta, omega at time zero of time horizon
        theta0 = theta - self.t*omega
        # cos, sin, weight spline over time horizon
        Ts = 2.*np.pi/abs(omega)
        if self.options['horizon_time'] is None:
            raise ValueError(
                'You need to provide a horizon time when using rotating obstacles!')
        else:
            T = self.options['horizon_time']
        n_quarters = int(np.ceil(4*T/Ts))
        knots_theta = np.r_[np.zeros(3), np.hstack(
            [0.25*k*np.ones(2) for k in range(1, n_quarters+1)]), 0.25*n_quarters]*(Ts/T)
        Tf, knots = get_interval_T(BSplineBasis(knots_theta, 2), 0, 1.)
        basis = BSplineBasis(knots, 2)
        # coefficients based on nurbs representation of circle
        cos_cfs = np.r_[1., np.sqrt(
            2.)/2., 0., -np.sqrt(2.)/2., -1., -np.sqrt(2.)/2.,  0.,  np.sqrt(2.)/2., 1.]
        sin_cfs = np.r_[0., np.sqrt(
            2.)/2., 1.,  np.sqrt(2.)/2.,  0., -np.sqrt(2.)/2., -1., -np.sqrt(2.)/2., 0.]
        weight_cfs = np.r_[1., np.sqrt(
            2.)/2., 1.,  np.sqrt(2.)/2.,  1.,  np.sqrt(2.)/2.,  1.,  np.sqrt(2.)/2., 1.]
        cos_cfs = Tf.dot(np.array([cos_cfs[k % 8] for k in range(len(basis))]))
        sin_cfs = Tf.dot(np.array([sin_cfs[k % 8] for k in range(len(basis))]))
        weight_cfs = Tf.dot(
            np.array([weight_cfs[k % 8] for k in range(len(basis))]))

        cos_wt = BSpline(basis, cos_cfs)
        sin_wt = BSpline(basis, sin_cfs)*np.sign(omega)
        self.cos = cos_wt*cos(theta0) - sin_wt*sin(theta0)
        self.sin = cos_wt*sin(theta0) + sin_wt*cos(theta0)
        self.gon_weight = BSpline(basis, weight_cfs)

    def define_collision_constraints(self, hyperplanes):
        for hyperplane in hyperplanes:
            a, b = hyperplane['a'], hyperplane['b']
            checkpoints, rad = self.shape.get_checkpoints()
            for l, chck in enumerate(checkpoints):
                xpos = self.pos_spline[
                    0]*self.gon_weight + chck[0]*self.cos - chck[1]*self.sin
                ypos = self.pos_spline[
                    1]*self.gon_weight + chck[0]*self.sin + chck[1]*self.cos
                self.define_constraint(-(a[0]*xpos + a[1] *
                                         ypos) + self.gon_weight*(b+rad[l]), -inf, 0.)

    def set_parameters(self, current_time):
        parameters = ObstaclexD.set_parameters(self, current_time)
        parameters['theta'] = self.signals['orientation'][:, -1]
        return parameters

    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def prepare_simulation(self, initial, simulation):
        ObstaclexD.prepare_simulation(self, initial, simulation)
        # initialize signals
        for key in ['orientation', 'angular_velocity']:
            if key in initial:
                self.signals[key] = np.c_[initial[key]]
            else:
                self.signals[key] = np.zeros((1, 1))

    def update(self, update_time, sample_time):
        ObstaclexD.update(self, update_time, sample_time)
        n_samp = int(update_time/sample_time)
        for k in range(n_samp):
            theta0 = self.signals['orientation'][:, -1][0]
            omega0 = self.signals['angular_velocity'][:, -1][0]
            theta = theta0 + sample_time*omega0
            omega = omega0
            self.signals['orientation'] = np.c_[
                self.signals['orientation'], theta]
            self.signals['angular_velocity'] = np.c_[
                self.signals['angular_velocity'], omega]

    def draw(self, t=-1):
        if not self.options['draw']:
            return []
        pose = np.zeros(2*self.n_dim)
        pose[:self.n_dim] = self.signals['position'][:, t]
        pose[2] = self.signals['orientation'][:, t]
        return self.shape.draw(pose)


class Obstacle3D(ObstaclexD):

    def __init__(self, initial, shape, simulation, options):
        ObstaclexD.__init__(self, initial, shape, simulation, options)

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def define_collision_constraints(self, hyperplanes):
        for hyperplane in hyperplanes:
            a, b = hyperplane['a'], hyperplane['b']
            checkpoints, rad = self.shape.get_checkpoints()
            for l, chck in enumerate(checkpoints):
                self.define_constraint(-sum([a[k]*(chck[k]+self.pos_spline[k])
                                             for k in range(self.n_dim)]) + b + rad[l], -inf, 0.)
