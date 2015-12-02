from optilayer import OptiLayer, evalf
from spline import BSplineBasis
from casadi import SX, MX, SXFunction
from scipy.signal import filtfilt, butter
from scipy.interpolate import splev, interp1d
from numpy.random import normal
import numpy as np

import time as tttime


class Vehicle(OptiLayer):

    def __init__(self, n_y, degree, shape, options, **kwargs):
        self.index = 0
        OptiLayer.__init__(self, 'vehicle'+str(self.index))
        self.shape = shape
        self._signals = {}
        self.signal_length = {}
        self._init_memory()

        # create spline basis
        self.degree = degree
        if 'knots' in kwargs:
            self.knots = kwargs['knots']
        elif 'knot_intervals' in kwargs:
            self.knot_intervals = kwargs['knot_intervals']
            self.knots = np.r_[np.zeros(self.degree),
                               np.linspace(0., 1., self.knot_intervals+1),
                               np.ones(self.degree)]
        else:
            self.knot_intervals = 10
            self.knots = np.r_[np.zeros(self.degree),
                               np.linspace(0., 1., self.knot_intervals+1),
                               np.ones(self.degree)]
        self.basis = BSplineBasis(self.knots, self.degree)

        # create spline variable
        self.n_y = n_y
        self.splines = self.define_spline_variable('y', self.n_y)

        # create corresponding signal
        self.order = self.degree if not (
            'order' in kwargs) else kwargs['order']
        self._y = SX.sym('y', n_y, self.order+1)
        self.define_signal('y', self._y)

        # set options
        self.set_default_options()
        self.set_options(options)

    # ========================================================================
    # Vehicle options
    # ========================================================================

    def set_default_options(self):
        self.options = {'model_mismatch': False, 'safety_distance': 0.,
                        'horizon_time': 10., 'sample_time': 0.01,
                        'ideal_update': False}
        self.options['boundary_smoothness'] = {'initial': self.order,
                                               'internal': self.degree-2,
                                               'terminal': self.degree}

    def set_options(self, options):
        self.options.update(options)

    def set_input_disturbance(self, fc, stdev, mean=None):
        # fc = cut-off frequency of disturbance
        # stdev, mean = parameters for Gaussian noise
        filt = butter(3, fc, 'low')
        if mean is None:
            mean = np.zeros(stdev.shape)
        self._inpute_dist = {'filter': filt, 'mean': mean, 'stdev': stdev}

    def add_input_disturbance(self, input):
        if not hasattr(self, '_inpute_dist'):
            return input
        n_sign = input.shape[0]
        n_samp = input.shape[2]
        disturbance = np.zeros((n_sign, 1, n_samp))
        filt = self._inpute_dist['filter']
        mean = self._inpute_dist['mean']
        stdev = self._inpute_dist['stdev']
        for k in range(n_sign):
            if stdev[k] > 0:
                disturbance[k, :] = filtfilt(filt[0], filt[1],
                                             normal(mean[k], stdev[k], n_samp))
            else:
                disturbance[k, :] = np.zeros(n_samp)
        return input + disturbance

    # ========================================================================
    # Signal manipulation
    # ========================================================================

    # define signals as symbolic expression
    def define_signal(self, name, expr):
        if isinstance(expr, list):
            self.signal_length[name] = len(expr)
        else:
            self.signal_length[name] = expr.shape[0]
        expr = [expr] if not isinstance(expr, list) else expr
        self._signals[name] = SXFunction([self._y], expr)

    def define_position(self, expr):
        self.define_signal('position', expr)

    def define_input(self, expr):
        self.define_signal('input', expr)

    def define_state(self, expr):
        self.define_signal('state', expr)

    # evaluate symbols from symbolic or numeric y trajectories
    def get_signal(self, name, y=None):
        if y is None:
            result = evalf(self._signals[name], self._y)
            result = result[0] if len(result) == 1 else result
        elif isinstance(y, (MX, SX)):
            result = evalf(self._signals[name], y)
            result = result[0] if len(result) == 1 else result
        else:
            if len(y.shape) == 3:
                result = []
                for t in range(y.shape[2]):
                    ret = evalf(self._signals[name], y[:, :, t])
                    result.append(np.vstack([r.toArray() for r in ret]))
                result = np.dstack(result)
            else:
                result = [r.toArray() for r in evalf(self._signals[name], y)]
                result = np.vstack(result)
        return result

    def get_position(self, y=None):
        if not ('position' in self._signals):
            raise ValueError('Position not defined! Use define_position().')
        return self.get_signal('position', y)

    def get_input(self, y=None):
        if not ('input' in self._signals):
            raise ValueError('Input not defined! Use define_input().')
        return self.get_signal('input', y)

    def get_state(self, y=None):
        if not ('state' in self._signals):
            raise ValueError('State not defined! Use define_state().')
        return self.get_signal('state', y)

    def get_signals(self, y_coeffs, T, time):
        signals = {}
        signals['time'] = time
        y = self.get_splines(y_coeffs, T, time)
        for name in self._signals.keys():
            signals[name] = self.get_signal(name, y)
        return signals

    # compute spline trajectories from coefficients
    def get_splines(self, y_coeffs, T, time):
        y = np.zeros((self.n_y, self.order+1, time.size))
        for k in range(self.n_y):
            for d in range(self.order+1):
                y[k, d, :] = (1./T**d)*splev(time/T, (self.knots,
                                                      y_coeffs[:, k].ravel(),
                                                      self.degree), d)
        return y

    # ========================================================================
    # Initial and terminal conditions
    # ========================================================================

    def set_initial_condition(self, y0):
        self.y0 = y0
        for name in self._signals.keys():
            signal = self.get_signal(name, y0)
            self.prediction[name] = signal
            self.path[name] = np.zeros(signal.shape + (1,))
            self.path[name][:, :, 0] = self.get_signal(name, y0)
        self.path['time'] = np.array([0.])
        # self.plant.set_state(self.get_state(y0))

    def set_terminal_condition(self, yT):
        self.yT = yT

    # ========================================================================
    # Update and simulation related functions
    # ========================================================================

    def update(self, y_coeffs, current_time, update_time, **kwargs):
        horizon_time = self.options['horizon_time']
        sample_time = self.options['sample_time']
        self._update_trajectory(y_coeffs, current_time, update_time,
                                horizon_time, sample_time, **kwargs)
        self._predict(update_time, sample_time)
        self._follow_trajectory(update_time, sample_time)

    def _update_trajectory(self, y_coeffs, current_time, update_time,
                           horizon_time, sample_time, **kwargs):
        # rel_current_time : current time wrt to spline basis horizon
        rel_current_time = 0.
        if 'rel_current_time' in kwargs:
            rel_current_time = kwargs['rel_current_time']
        n_samp = int(round((horizon_time-rel_current_time)/sample_time, 3)) + 1
        time_axis = np.linspace(current_time,
                                current_time + (n_samp-1)*sample_time, n_samp)
        if 'time_axis_knots' in kwargs:
            time_axis_knots = kwargs['time_axis_knots']
        else:
            time_axis_knots = ((np.r_[self.knots] - rel_current_time) *
                               horizon_time + current_time)
        time_spline_ev = time_axis - current_time + rel_current_time
        time_spline_ev_knots = (time_axis_knots -
                                current_time + rel_current_time)

        self.trajectory = self.get_signals(y_coeffs, horizon_time,
                                           time_spline_ev)
        self.trajectory['time'] = time_axis
        self.trajectory_kn = self.get_signals(y_coeffs, horizon_time,
                                              time_spline_ev_knots)
        self.trajectory_kn['time'] = time_axis_knots
        self._add_to_memory(self.trajectories, self.trajectory,
                            int(update_time/sample_time))
        self._add_to_memory(self.trajectories_kn, self.trajectory_kn,
                            int(update_time/sample_time))

    def _predict(self, predict_time, sample_time):
        if self.options['ideal_update']:
            n_samp = int(predict_time/sample_time)
            y_pred = self.trajectory['y'][:, :, n_samp]
        else:
            input = self.trajectory['input']
            y0 = self.path['y'][:, :, -1]
            y = self.integrate_model(y0, input, sample_time, predict_time)
            y_pred = y[:, :, -1]
            n_samp = int(predict_time/sample_time)
        for key in self._signals.keys():
            self.prediction[key] = self.get_signal(key, y_pred)
        self._add_to_memory(self.predictions, self.prediction,
                            int(predict_time/sample_time))

    def _follow_trajectory(self, follow_time, sample_time):
        n_samp = int(follow_time/sample_time)
        if self.options['ideal_update']:
            y = self.trajectory['y'][:, :, 1:n_samp+1]
        else:
            y0 = self.path['y'][:, :, -1]
            input = self.add_input_disturbance(self.trajectory['input'])
            y = self.integrate_model(y0, input, sample_time, follow_time)
            y = y[:, :, 1:]
        self.path['time'] = np.append(self.path['time'],
                                      (self.path['time'][-1] +
                                       np.linspace(sample_time, follow_time,
                                                   n_samp)), axis=1)
        for key in self._signals.keys():
            self.path[key] = np.dstack(
                (self.path[key], self.get_signal(key, y)))

    def _get_input_sample(self, time, input, sample_time):
        n_samp = input.shape[1]
        time_axis = np.linspace(0, (n_samp-1)*sample_time, n_samp)
        u_interp = interp1d(time_axis, input, kind='linear')
        return u_interp(time)

    def _init_memory(self):
        self.trajectories = {}
        self.trajectories_kn = {}
        self.predictions = {}
        self.trajectory = {}
        self.trajectory_kn = {}
        self.prediction = {}
        self.path = {}

    def _add_to_memory(self, memory, dictionary, repeat=1):
        for key in dictionary.keys():
            if not (key in memory):
                memory[key] = []
            memory[key].extend([dictionary[key] for k in range(repeat)])

    # ========================================================================
    # Methods encouraged to override (very basic implementation)
    # ========================================================================

    def init_variables(self):
        variables = {'y': np.zeros((len(self.basis), self.n_y))}
        for k in range(self.n_y):
            variables['y'][:, k] = np.linspace(
                self.y0[k, 0], self.yT[k, 0], len(self.basis))
        return variables

    def get_parameters(self, time):
        return {}

    def get_checkpoints(self, y=None):
        if y is None:
            return self.shape.get_checkpoints(self.splines)
        else:
            return self.shape.get_checkpoints(y)

    def draw(self, k=-1):
        return self.path['position'][:, k]

    # ========================================================================
    # Methods required to override (no general implementation possible)
    # ========================================================================

    def set_initial_pose(self, pose):
        raise NotImplementedError('Please implement this method!')

    def set_terminal_pose(self, pose):
        raise NotImplementedError('Please implement this method!')

    def integrate_model(self, y0, input, sample_time, integration_time):
        raise NotImplementedError('Please implement this method!')
