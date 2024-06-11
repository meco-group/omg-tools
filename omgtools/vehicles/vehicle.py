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
from ..basics.spline import BSplineBasis
from ..basics.spline_extra import concat_splines, definite_integral, sample_splines
from ..basics.shape import Rectangle, Square, Circle
from ..execution.plotlayer import PlotLayer
from casadi import inf, vertsplit
from scipy.signal import filtfilt, butter
from scipy.interpolate import interp1d
from scipy.integrate import odeint
from numpy.random import normal
from itertools import groupby
import numpy as np
import sys

if sys.version_info >= (3,):
  long = int

class Vehicle(OptiChild, PlotLayer):

    def __init__(self, n_spl, degree, shapes, options=None):
        options = options or {}
        OptiChild.__init__(self, 'vehicle')
        PlotLayer.__init__(self)
        self.shapes = shapes if isinstance(shapes, list) else [shapes]
        self.n_dim = self.shapes[0].n_dim
        for shape in self.shapes:
            if shape.n_dim == self.n_dim:
                self.n_dim = shape.n_dim
            else:
                raise ValueError('All vehicle shapes should have same spatial' +
                                 'dimension.')

        self.prediction = {}
        self.init_spline_values = None
        self.degree = degree

        self.to_simulate = True

        # set options
        self.set_default_options()
        self.set_options(options)

        # create default spline basis
        self.define_knots(knot_intervals=10)
        self.n_spl = n_spl

    # ========================================================================
    # Vehicle options
    # ========================================================================

    def set_default_options(self):
        self.options = {'safety_distance': 0., 'safety_weight': 10.,
                        'room_constraints': True, 'stop_tol': 1.e-3,
                        'ideal_prediction': False, 'ideal_update': False,
                        '1storder_delay': False, 'time_constant': 0.1,
                        'input_disturbance': None}

    def set_options(self, options):
        self.options.update(options)

    def define_knots(self, **kwargs):
        if 'knot_intervals' in kwargs:
            self.knot_intervals = kwargs['knot_intervals']
            self.knots = np.r_[np.zeros(self.degree), np.linspace(
                0., 1., self.knot_intervals+1), np.ones(self.degree)]
        if 'knots' in kwargs:
            self.knots = kwargs['knots']
        self.basis = BSplineBasis(self.knots, self.degree)

    def set_init_spline_values(self, values, n_seg=1):
        # first initialize as empty list
        self.init_spline_values = [0]*n_seg
        # then assign input values
        for k in range(n_seg):
            if values[k].shape == (len(self.basis), self.n_spl):
                self.init_spline_values[k] = values[k]
            else:
                raise ValueError('Initial guess has wrong dimensions for spline ' +
                    str(k) + ', required: ' + str((len(self.basis), self.n_spl)) +
                    ' while you gave: ' + str(values[k].shape))

    # ========================================================================
    # Optimization modelling related functions
    # ========================================================================

    def define_splines(self, n_seg=1):
        self.n_seg = n_seg
        self.splines = []
        if self.init_spline_values is not None:
            init = self.init_spline_values
            self.init_spline_values = None
        else:
            try:
                init = self.get_init_spline_values()
            except AttributeError as exc:
                init = [None]*n_seg
        for k in range(self.n_seg):
            spline = self.define_spline_variable(
                'splines_seg'+str(k), self.n_spl, value=init[k])
            self.splines.append(spline)
        return self.splines

    def define_collision_constraints_2d(self, hyperplanes,
        room, positions, horizon_time, tg_ha=0, offset=0):
        t = self.define_symbol('t')
        # T = self.define_symbol('T')
        safety_distance = self.options['safety_distance']
        safety_weight = self.options['safety_weight']
        positions = [positions] if not isinstance(
            positions[0], list) else positions
        for s, shape in enumerate(self.shapes):
            position = positions[s]
            checkpoints, rad = shape.get_checkpoints()
            # obstacle avoidance
            if shape in hyperplanes:
                for k, hyperplane in enumerate(hyperplanes[shape]):
                    a, b = hyperplane['a'], hyperplane['b']
                    sl = 1 if 'slack' not in hyperplane else hyperplane['slack']
                    if safety_distance > 0.:
                        eps = self.define_spline_variable(
                            'eps_'+str(s)+str(k))[0]
                        obj = safety_weight*definite_integral(eps, t/horizon_time, 1.)
                        self.define_objective(obj)
                        self.define_constraint(eps - safety_distance, -inf, 0.)
                        self.define_constraint(-eps, -inf, 0.)
                    else:
                        eps = 0.
                    for l, chck in enumerate(checkpoints):
                        con = 0
                        con += (a[0]*chck[0] + a[1]*chck[1])*(1.-tg_ha**2)
                        con += (-a[0]*chck[1] + a[1]*chck[0])*(2*tg_ha)
                        pos = [0, 0]
                        # next part gives an offset to input position
                        # e.g. for trailer position
                        pos[0] = position[0]*(1+tg_ha**2) + offset*(1-tg_ha**2)
                        pos[1] = position[1]*(1+tg_ha**2) + offset*(2*tg_ha)
                        con += (a[0]*pos[0] + a[1]*pos[1])
                        con += (-b+sl*rad[l]+safety_distance-eps)*(1+tg_ha**2)
                        self.define_constraint(con, -inf, 0)
            # room constraints
            # check room shape and orientation,
            # check vehicle shape and orientation
            # then decide on type of constraints to use:
            # room_limits or hyperplanes
            if self.options['room_constraints']:
                lims = room['shape'].get_canvas_limits()
                room_limits = []
                room_limits += [lims[k]+room['position'][k] for k in range(self.n_dim)]
                if ((isinstance(room['shape'], (Rectangle, Square)) and
                    room['shape'].orientation == 0.0) and
                    (isinstance(shape, Circle) or
                    (isinstance(shape, (Rectangle, Square)) and
                     shape.orientation == 0)) and
                    (isinstance(tg_ha, (int, float, long)) and tg_ha == 0.)):
                    for chck in checkpoints:
                        for k in range(self.n_dim):
                            self.define_constraint(-(chck[k]+position[k]) + room_limits[k][0] + rad[0], -inf, 0.)
                            self.define_constraint((chck[k]+position[k]) - room_limits[k][1] + rad[0], -inf, 0.)
                else:
                    hyp_room = room['shape'].get_hyperplanes(position = room['position'])
                    for l, chck in enumerate(checkpoints):
                        for hpp in hyp_room.values():
                            con = 0
                            con += (hpp['a'][0]*chck[0] + hpp['a'][1]*chck[1])*(1.-tg_ha**2)
                            con += (-hpp['a'][0]*chck[1] + hpp['a'][1]*chck[0])*(2*tg_ha)
                            pos = [0, 0]  # next part gives an offset to input position e.g. for trailer position
                            pos[0] = position[0]*(1+tg_ha**2) + offset*(1-tg_ha**2)  # = real_pos*(1+tg_ha**2)
                            pos[1] = position[1]*(1+tg_ha**2) + offset*(2*tg_ha)  # = real_pos*(1+tg_ha**2)
                            con += (hpp['a'][0]*pos[0] + hpp['a'][1]*pos[1])
                            con += (-hpp['b']+rad[l])*(1+tg_ha**2)
                            self.define_constraint(con, -inf, 0)

    def define_collision_constraints_3d(self, hyperplanes, room, positions, horizon_time):
        # orientation for 3d not yet implemented!
        t = self.define_symbol('t')
        safety_distance = self.options['safety_distance']
        safety_weight = self.options['safety_weight']
        positions = [positions] if not isinstance(
            positions[0], list) else positions
        for s, shape in enumerate(self.shapes):
            position = positions[s]
            checkpoints, rad = shape.get_checkpoints()
            # obstacle avoidance
            if shape in hyperplanes:
                for k, hyperplane in enumerate(hyperplanes[shape]):
                    a, b = hyperplane['a'], hyperplane['b']
                    safety_distance = self.options['safety_distance']
                    safety_weight = self.options['safety_weight']
                    if safety_distance > 0.:
                        # Todo: remove?
                        t = self.define_symbol('t')
                        eps = self.define_spline_variable(
                            'eps_'+str(s)+str(k))[0]
                        obj = safety_weight*definite_integral(eps, t/horizon_time, 1.)
                        self.define_objective(obj)
                        self.define_constraint(eps - safety_distance, -inf, 0.)
                        self.define_constraint(-eps, -inf, 0.)
                    else:
                        eps = 0.
                    for l, chck in enumerate(checkpoints):
                        self.define_constraint(
                            sum([a[k]*(chck[k]+position[k]) for k in range(3)])-b+rad[l]+safety_distance-eps, -inf, 0)
            # room constraints
            if self.options['room_constraints']:
                lims = room['shape'].get_canvas_limits()
                room_limits = []
                room_limits += [lims[k]+room['position'][k] for k in range(self.n_dim)]
                for chck in checkpoints:
                    for k in range(3):
                        self.define_constraint(-
                                               (chck[k]+position[k]) + room_limits[k][0], -inf, 0.)
                        self.define_constraint(
                            (chck[k]+position[k]) - room_limits[k][1], -inf, 0.)

    def get_fleet_center(self, splines, rel_pos, substitute=True):
        rel_pos = rel_pos if isinstance(rel_pos,list) else vertsplit(rel_pos)
        if substitute:
            center = self.define_substitute('fleet_center', [s+rp for s, rp in zip(splines, rel_pos)])
            return center
        else:
            return [s+rp for s, rp in zip(splines, rel_pos)]

    def set_parameters(self, current_time):
        parameters = {self: {}}
        return parameters

    # ========================================================================
    # Deploying related functions
    # ========================================================================

    def store(self, current_time, sample_time, spline_segments, segment_times, time_axis=None, **kwargs):
        if not isinstance(segment_times, list):
            segment_times = [segment_times]
        # determine how many knots must be inserted when concatenating splines
        # obtaining a certain continuity requires inserting this continuity - degree+1 knots
        if 'continuity' in kwargs:
            n_insert = kwargs['continuity'] - (self.degree-1)
        else:
            n_insert = None

         # save individual spline segments
        self.result_spline_segments = np.array(spline_segments)
        self.segment_times = segment_times

        splines = concat_splines(spline_segments, segment_times, n_insert=n_insert)

        # save concatenated splines
        self.result_splines = splines
        horizon_time = sum(segment_times)
        if time_axis is None:
            n_samp = int(round(horizon_time/sample_time, 6)) + 1
            time_axis = np.linspace(0., (n_samp-1)*sample_time, n_samp)
        self.trajectories = self.splines2signals(splines, time_axis)
        if not set(['state', 'input']).issubset(self.trajectories):
            raise ValueError(
                'Signals should contain at least state, input and pose.')
        self.trajectories['time'] = time_axis - time_axis[0] + current_time
        self.trajectories['pose'] = self._state2pose(self.trajectories['state'])
        self.trajectories['splines'] = np.c_[
            sample_splines(splines, time_axis)]
        if hasattr(self, 'rel_pos_c') and ('fleet_center' not in self.trajectories):
            self.trajectories['fleet_center'] = np.c_[sample_splines(
                [s+rp for s, rp in zip(splines, self.rel_pos_c)], time_axis)]
        knots = splines[0].basis.knots
        time_axis_kn = np.r_[knots[self.degree] + time_axis[0], [k for k in knots[
        self.degree+1:-self.degree] if k > (knots[self.degree]+time_axis[0])]]
        self.trajectories_kn = self.splines2signals(splines, time_axis_kn)
        self.trajectories_kn['time'] = time_axis_kn - \
            time_axis_kn[0] + current_time
        self.trajectories_kn['pose'] = self._state2pose(self.trajectories_kn['state'])
        self.trajectories_kn['splines'] = np.c_[
            sample_splines(splines, time_axis_kn)]
        for key in self.trajectories:
            shape = self.trajectories[key].shape
            if len(shape) == 1:
                self.trajectories[key] = self.trajectories[
                    key].reshape(1, shape[0])
        for key in self.trajectories_kn:
            shape = self.trajectories_kn[key].shape
            if len(shape) == 1:
                self.trajectories_kn[key] = self.trajectories_kn[
                    key].reshape(1, shape[0])

    def predict(self, current_time, predict_time, sample_time, state0=None, input0=None, dinput0=None, delay=0, enforce_states=False, enforce_inputs=False):
        if enforce_states and enforce_inputs:
            if all(l is not None for l in [state0, input0, dinput0]):
                # all three have a value, and are not None
                self.set_initial_conditions(state0, input=input0, dinput=dinput0)
            elif all(l is not None for l in [state0, input0]):
                # all two have a value and are not None
                self.set_initial_conditions(state0, input=input0)
            else:
                if hasattr(self, 'signals'):
                    self.set_initial_conditions(
                        self.signals['state'][:, -1], self.signals['input'][:, -1], self.signals['dinput'][:, -1])
            return
        if enforce_states:
            if state0 is not None:
                self.set_initial_conditions(state0)
            else:
                if hasattr(self, 'signals'):
                    self.set_initial_conditions(self.signals['state'][:, -1])
            return
        n_samp = int(np.round(predict_time/sample_time, 6))
        if self.options['ideal_prediction']:
            for key in self.trajectories:
                self.prediction[key] = self.trajectories[key][:, n_samp+delay]
        else:
            for key in self.trajectories:
                if key not in ['state', 'input', 'pose']:
                    self.prediction[key] = self.trajectories[key][:, n_samp+delay]
            input = self.trajectories['input'][:, delay:]
            if state0 is None:
                state0 = self.signals['state'][:, -n_samp-1]  # current state
            state = self.integrate_ode(
                state0, input, predict_time, sample_time)
            self.prediction['state'] = state[:, -1]
            self.prediction['input'] = self.trajectories['input'][:, n_samp+delay]
            self.prediction['pose'] = self._state2pose(state[:, -1])


    # ========================================================================
    # Simulation related functions
    # ========================================================================

    def overrule_state(self, state):
        state = np.array(state)
        self.signals['state'][:, -1] = state
        self.signals['pose'][:, -1] = self._state2pose(state)
        self.prediction['state'] = state
        self.prediction['pose'] = self._state2pose(state)

    def overrule_input(self, input, dinput=None):
        input = np.array(input)
        self.signals['input'][:, -1] = input
        self.prediction['input'] = input
        if dinput is not None:
            self.signals['dinput'][:, -1] = dinput
            self.prediction['dinput'] = dinput

    def simulate(self, simulation_time, sample_time):
        if self.to_simulate:
            if not hasattr(self, 'signals'):
                self.signals = {}
                for key in self.trajectories:
                    self.signals[key] = np.c_[self.trajectories[key][:, 0]]
            n_samp = int(np.round(simulation_time/sample_time, 6))
            if self.options['ideal_update']:
                for key in self.trajectories:
                    self.signals[key] = np.c_[
                        self.signals[key], self.trajectories[key][:, 1:n_samp+1]]
            else:
                for key in self.trajectories:
                    if key not in ['state', 'input', 'pose']:
                        self.signals[key] = np.c_[
                            self.signals[key], self.trajectories[key][:, 1:n_samp+1]]
                input = self.trajectories['input']
                if self.options['input_disturbance']:
                    input = self.add_disturbance(input)
                if self.options['1storder_delay']:
                    input0 = self.signals['input'][:, -1]
                    input = self.integrate_ode(
                        input0, input, simulation_time, sample_time, self._ode_1storder)
                state0 = self.signals['state'][:, -1]  # current state
                state = self.integrate_ode(
                    state0, input, simulation_time, sample_time)
                self.signals['input'] = np.c_[
                    self.signals['input'], input[:, 1:n_samp+1]]
                self.signals['state'] = np.c_[
                    self.signals['state'], state[:, 1:n_samp+1]]
                self.signals['pose'] = np.c_[
                self.signals['pose'], self._state2pose(state[:, 1:n_samp+1])]
        # store trajectories
        if not hasattr(self, 'traj_storage'):
            self.traj_storage = {}
            self.traj_storage_kn = {}
            self.pred_storage = {}
        repeat = int(simulation_time/sample_time)
        self._add_to_memory(self.traj_storage, self.trajectories, repeat)
        self._add_to_memory(self.traj_storage_kn, self.trajectories_kn, repeat)
        self._add_to_memory(self.pred_storage, self.prediction, repeat)
        # update plots
        self.update_plots()

    def _state2pose(self, state):
        if len(state.shape) <= 1:
            return self.state2pose(state)
        else:
            pose = []
            for k in range(state.shape[1]):
                pose.append(self.state2pose(state[:, k]))
            return np.c_[pose].T

    def integrate_ode(self, state0, input, integration_time, sample_time, ode=None):
        if ode is None:
            ode = self._ode
        n_samp = int(integration_time/sample_time)+1
        time_axis = np.linspace(0., (n_samp-1)*sample_time, n_samp)
        # make interpolation function which returns the input at a certain time
        time_interp = np.linspace(
            0., (input.shape[1]-1)*sample_time, input.shape[1])
        input_interp = interp1d(time_interp, input, kind='linear',
                                bounds_error=False, fill_value=input[:, -1])
        state = odeint(ode, state0, time_axis, args=(input_interp,)).T
        return state

    def _ode(self, state, time, input_interp):
        input = input_interp(time)
        return self.ode(state, input)

    def _ode_1storder(self, state, time, input_interp):
        input = input_interp(time)
        return (1./self.options['time_constant'])*(input - state)

    def add_disturbance(self, input):
        if self.options['input_disturbance'] is not None:
            fc = self.options['input_disturbance']['fc']
            stdev = self.options['input_disturbance']['stdev']
            if 'mean' in self.options['input_disturbance']:
                mean = self.options['input_disturbance']['mean']
            else:
                mean = np.zeros(stdev.shape)
            n_sign = input.shape[0]
            n_samp = input.shape[1]
            disturbance = np.zeros((n_sign, n_samp))
            filt = butter(3, fc, 'low')
            for k in range(n_sign):
                disturbance[k, :] = filtfilt(filt[0], filt[1],
                                             normal(mean[k], stdev[k], n_samp))
            return input + disturbance
        else:
            return input

    def _add_to_memory(self, memory, dictionary, repeat=1):
        for key in dictionary.keys():
            if not (key in memory):
                memory[key] = []
            memory[key].extend([dictionary[key] for k in range(repeat)])

    def draw(self, t=-1):
        surf, lines = [], []
        for shape in self.shapes:
            s, l = shape.draw(self.signals['pose'][:, t])
            surf += s
            lines += l
        return surf, lines

    # ========================================================================
    # Plot related functions
    # ========================================================================

    def init_plot(self, signal, **kwargs):
        if not hasattr(self, 'signals'):
            return None
        size = self.signals[signal].shape[0]
        if 'labels' not in kwargs:
            if size > 1:
                labels = [signal+str(k) for k in range(size)]
            else:
                labels = [signal]
        else:
            labels = kwargs['labels']
        ax_r, ax_c = size, 1
        info = []
        n_colors = len(self.colors)
        index = int([''.join(g) for _, g in groupby(self.label, str.isalpha)][-1]) % n_colors
        for k in range(ax_r):
            inf = []
            for _ in range(ax_c):
                lines = []
                lines += [{'linestyle': '-', 'color': self.colors_w[index]}]
                lines += [{'linestyle': '-', 'color': self.colors[index]}]
                if 'knots' in kwargs and kwargs['knots']:
                    lines += [{'linestyle': 'None', 'marker': 'x', 'color': self.colors[index]}]
                if 'prediction' in kwargs and kwargs['prediction']:
                    lines += [{'linestyle': 'None', 'marker': 'o', 'color': self.colors[index]}]
                dic = {'labels': ['t (s)', labels[k]], 'lines': lines}
                if 'xlim' in kwargs:
                    dic['xlim'] = kwargs['xlim']
                if 'ylim' in kwargs:
                    dic['ylim'] = kwargs['ylim']
                inf.append(dic)
            info.append(inf)
        return info

    def update_plot(self, signal, t, **kwargs):
        if not hasattr(self, 'signals'):
            return None
        size = self.signals[signal].shape[0]
        ax_r, ax_c = size, 1
        data = []
        for k in range(ax_r):
            dat = []
            for l in range(ax_c):
                lines = []
                lines += [np.vstack((self.traj_storage['time'][t], self.traj_storage[signal][t][k, :]))]
                if t == -1:
                    lines += [np.vstack((self.signals['time'], self.signals[signal][k, :]))]
                else:
                    lines += [np.vstack((self.signals['time'][:, :t+1], self.signals[signal][k, :t+1]))]
                if 'knots' in kwargs and kwargs['knots']:
                    lines += [np.vstack((self.traj_storage_kn['time'][t], self.traj_storage_kn[signal][t][k, :]))]
                if 'prediction' in kwargs and kwargs['prediction']:
                    lines += [np.vstack((self.traj_storage['time'][t][:, 0], self.pred_storage[signal][t][k]))]
                dat.append({'lines': lines})
            data.append(dat)
        return data

    # ========================================================================
    # Methods required to override
    # ========================================================================

    def init(self):
        pass

    def define_trajectory_constraints(self, splines):
        raise NotImplementedError('Please implement this method!')

    def get_initial_constraints(self, state, input=None):
        raise NotImplementedError('Please implement this method!')

    def get_terminal_constraints(self, splines):
        raise NotImplementedError('Please implement this method!')

    def check_terminal_conditions(self):
        raise NotImplementedError('Please implement this method!')

    def splines2signals(self, splines, time):
        raise NotImplementedError('Please implement this method!')

    def state2pose(self, state):
        raise NotImplementedError('Please implement this method!')

    def ode(self, state, input):
        raise NotImplementedError('Please implement this method!')
