from optilayer import OptiChild, evalf
from spline import BSplineBasis
from casadi import SX, MX, SXFunction, vertcat, horzcat
from scipy.signal import filtfilt, butter
from scipy.interpolate import splev, interp1d
from scipy.integrate import odeint
from numpy.random import normal
import numpy as np
import re


class Vehicle(OptiChild):

    def __init__(self, n_y, n_der, degree, order, shape, options, **kwargs):
<<<<<<< HEAD
        # n_y: number of spline variables
        # n_der: required number of derivatives of y to express signals
        # degree: degree of spline
        # order: order of ode describing the system
=======
        #n_y: number of spline variables
        #n_der: required number of derivatives of y to save/plot/express signals
        #degree: degree of spline
        #order: order of ode describing the system
>>>>>>> develop

        OptiChild.__init__(self, 'vehicle')
        self.shape = shape
        self._signals = {}
        self._signals_expr = {}
        self._signals_num = {}
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
        self.order = order
        # create corresponding signal
        if n_der > degree:
            raise ValueError('n_der should be smaller or ' +
                             'equal to degree of spline.')
        self.n_der = n_der
        self._y = SX.sym('y', n_y, self.n_der+1)
        self.define_signal('y', self._y)

        # set options
        self.set_default_options()
        self.set_options(options)

        # default y0 & yT
        self.y0 = np.zeros((self.n_y, self.n_der+1))
        self.yT = np.zeros((self.n_y, self.n_der+1))

    # ========================================================================
    # Vehicle options
    # ========================================================================

    def set_default_options(self):
        self.options = {'safety_distance': 0., 'safety_weight': 10.,
                        'sample_time': 0.01,
                        'ideal_update': False, '1storder_delay': False,
                        'time_constant': 0.1}
<<<<<<< HEAD
        self.options['boundary_smoothness'] = {'initial': self.order,
=======
        self.options['boundary_smoothness'] = {'initial': self.degree,
>>>>>>> develop
                                               'internal': self.order,
                                               'terminal': self.degree}
        # *internal smooth up to order: states are continuous
        # *initial smoothness <= order: otherwise resolved after 1 update
        #       because internal smoothness is smaller
        # *terminal as much as possible for stability reasons.

    def set_options(self, options):
        if 'boundary_smoothness' in options:
            self.options['boundary_smoothness'].update(options['boundary_smoothness'])
        for key in options:
            if key not in ['boundary_smoothness']:
                self.options[key] = options[key]
        if self.options['1storder_delay']:
            self.integrate_plant = self._integrate_plant
        else:
            self.integrate_plant = self.integrate_model

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
        self._signals[name] = SXFunction(name, [self._y], expr)
        self._signals_num[name] = self._generate_function(expr, self._y)
        self._signals_expr[name] = expr

    def define_position(self, expr):
        self.define_signal('position', expr)
        self.n_pos = self.signal_length['position']
        self._position = SX.sym('position', self.n_pos)
        return self._position

    def define_input(self, expr):
        self.define_signal('input', expr)
        self.n_in = self.signal_length['input']
        self._input = SX.sym('input', self.n_in)
        return self._input

    def define_state(self, expr):
        self.define_signal('state', expr)
        self.n_st = self.signal_length['state']
        self._state = SX.sym('state', self.n_st)
        return self._state

    def define_dstate(self, expr):
        if len(expr) != self.n_st:
            raise ValueError('Size of dstate does not agree with size of state!')
        dstate = vertcat(expr)
        self._signals_expr['dstate'] = dstate
        self.model_update = self._generate_function(dstate,
                                                    [self._state, self._input])

    def define_y(self, expr):
        y = []
        for e in expr:
            if len(e) != self.n_y:
                raise ValueError('Size of y does not agree with n_y!')
            y.append(vertcat(e))
        y = horzcat(y)
        if y.shape[1] != self.order+1:
            raise ValueError('You should provide the same number ' +
                             'of derivatives as order')
        self._signals_expr['y'] = y
        self._get_y = self._generate_function(y, [self._state, self._input])

    # evaluate symbols from symbolic or numeric y trajectories
    def get_signal(self, name, y=None):
        if y is None:
            result = evalf(self._signals[name], self._y)
            result = result[0] if len(result) == 1 else result
        elif isinstance(y, (MX, SX)):
            result = evalf(self._signals[name], y)
            result = result[0] if len(result) == 1 else result
        else:  # numerical evaluation
            if len(y.shape) == 3:
                result = []
                for t in range(y.shape[2]):
                    ret = np.vstack(self._signals_num[name](y[:, :, t]))
                    result.append(ret)
                result = np.dstack(result)
            else:
                result = np.vstack(self._signals_num[name](y))
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
        y = np.zeros((self.n_y, self.n_der+1, time.size))
        for k in range(self.n_y):
            for d in range(self.n_der+1):
                y[k, d, :] = (1./T**d)*splev(time/T, (self.knots,
                                                      y_coeffs[:, k].ravel(),
                                                      self.degree), d)
        return y

    # inverse getter: get y from state and input
    def get_y(self, state, input):
        if len(state.shape) == 3:
            n_samp = state.shape[2]
            y = np.zeros((self.n_y, self.n_der+1, n_samp))
            for k in range(n_samp):
                # fill in first part of y e.g. pos and vel
                _y = np.array(self._get_y(state[:, :, k].ravel(),
                                          input[:, :, k].ravel()))
                y[:, :_y.shape[1], k] = _y
            sample_time = self.options['sample_time']
            for d in range(_y.shape[1], self.n_der+1):
                # fill in second part e.g. acceleration
                for k in range(self.n_y):
                    y[k, d, :] = np.gradient(y[k, d-1, :], sample_time)
            return y
        else:
            y = np.zeros((self.n_y, self.n_der+1))
            _y = np.array(self._get_y(state.ravel(), input.ravel()))
            y[:, :_y.shape[1]] = _y
            return y

    # convert symbolic casadi expression to numerical function evaluation
    # this is probably the most stupid way to do it, but it is the most
    # efficient way that works. Idea is: check your casadi expression and
    # build up a string from which a lambda expression is build. Because casadi
    # uses in its __str__ function some non-python operation names,
    # _check_expression should translate them. This is however manually
    # implemented and probably not all cases are covered...
    def _generate_function(self, expression, arguments):
        arg, body = self._get_function_expression(expression, arguments)
        exec('fun = lambda %s: %s' % (arg, body))
        return fun

    def _get_function_expression(self, expression, arguments, translator=None):
        if not isinstance(expression, list):
            expression = [expression]
        if not isinstance(arguments, list):
            arguments = [arguments]
        if translator is None:
            translator = self._translate_expression
        expr_str, names = [], []
        for arg in arguments:
            if arg.size() > 1:
                names.append(arg[0].getName()[:-2])
            else:
                names.append(arg.getName())
        for expr in expression:
            string = expr.__str__()
            for i, arg in enumerate(arguments):
                for k in range(arg.size1()):
                    if arg.size2() > 1:
                        for l in range(arg.size2()):
                            j = arg.size1()*l + k
                            string = string.replace('%s_%d' % (names[i], j),
                                                    '%s[%d,%d]' % (names[i], k, l))
                    elif arg.size() > 1:
                        string = string.replace('%s_%d' % (names[i], k),
                                                '%s[%d]' % (names[i], k))
                    else:
                        string = string.replace('%s' % (names[i]),
                                                '%s[%d]' % (names[i], 0))
            string = translator(string)
            expr_str.append(string)
        body = ','.join(expr_str)
        arg = ','.join(names)
        if body[0] != '[':
            return arg, '['+body+']'
        else:
            return arg, body

    def _translate_expression(self, expression):
        dictionary = {'sqrt(': 'np.sqrt(', 'cos(': 'np.cos(',
                      'sin(': 'np.sin(', 'atan2(': 'np.arctan2(',
                      'atan(': 'np.arctan(', 'tan(': 'np.tan(',
                      'pow(': 'np.power(', 'log(': 'np.log(',
                      'log10(': 'np.log10('}
        for str1, str2 in dictionary.items():
            if expression.find(str1) >= 0:
                expression = expression.replace(str1, str2)
        # remove '\n'
        expression = expression.translate(None, '\n')
        # sq() means power of 2
        while expression.find('sq(') >= 0:
            splt = expression.split('sq(', 1)
            a = splt[1]
            br_in_between = len(a.split(')')[0].split('(')) - 1
            splt2 = a.split(')', br_in_between+1)
            expression = (splt[0]+'np.power('+')'.join(splt2[:br_in_between+1])
                          + ','+str(2)+')' + splt2[-1])
        # find @i
        cnt = 1
        while expression.find('@'+str(cnt)) >= 0:
            value = expression.split('@'+str(cnt)+'=')[1].split(',')[0]
            expression = re.sub(r'@%d\b' % cnt, value, expression)
            expression = ','.join(expression.split(',')[1:])
            cnt += 1
        return expression

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
        self.init()

    def set_terminal_condition(self, yT):
        self.yT = yT
        self.init()

    def init(self):
        init_y = np.zeros((len(self.basis), self.n_y))
        for k in range(self.n_y):
<<<<<<< HEAD
            init_y[:, k] = np.r_[self.y0[k, 0]*np.ones(self.degree),
                                 np.linspace(self.y0[k, 0], self.yT[k, 0],
                                             len(self.basis) - 2*self.degree),
                                 self.yT[k, 0]*np.ones(self.degree)]
            # init_y[:, k] = np.linspace(self.y0[k, 0], self.yT[k, 0],
            #                            len(self.basis))
=======
            # init_y[:, k] = np.r_[self.y0[k, 0]*np.ones(self.degree),
            #                      np.linspace(self.y0[k, 0], self.yT[k, 0],
            #                                  len(self.basis) - 2*self.degree),
            #                      self.yT[k, 0]*np.ones(self.degree)]
            init_y[:, k] = np.linspace(self.y0[k, 0], self.yT[k, 0],
                                       len(self.basis))
>>>>>>> develop
        self.set_value('y', init_y)

    # ========================================================================
    # Update and simulation related functions
    # ========================================================================

    def update(self, y_coeffs, current_time, update_time, horizon_time,
               **kwargs):

        sample_time = self.options['sample_time']
        # check if arrived almost at the end of movement, make sure to only
        # update until remaining time (horizon_time)
        if horizon_time < update_time:
            update_time = horizon_time
        # note: horizon_time cannot become < sample_time since stop_criterium
        # is activated when remaining time (= horizon_time) < update_time and
        # update_time > sample_time
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
            # self.path = which path did the robot already cover at this time
            # -1 gets e.g. current pos/vel/acc = state + inputs
            y0 = self.path['y'][:, :, -1]
            # get y based on current y (y0) & input over a certain predict_time
            y = self.integrate_model(y0, input, sample_time, predict_time)
            # which elements are selected here?
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
            y = self.integrate_plant(y0, input, sample_time, follow_time)
            y = y[:, :, 1:]
        self.path['time'] = np.append(self.path['time'],
                                      (self.path['time'][-1] +
                                       np.linspace(sample_time, follow_time,
                                                   n_samp)), axis=1)
        for key in self._signals.keys():
            # append current info to path: position, input, velocity,...
            self.path[key] = np.dstack(
                (self.path[key], self.get_signal(key, y)))

    def integrate_model(self, y0, input, sample_time, integration_time):
        # make interpolation function which returns the input at a certain time
        u_interp = self._get_input_interpolator(input, sample_time)
        n_samp = int(integration_time/sample_time)+1
        time_axis = np.linspace(0., (n_samp-1)*sample_time, n_samp)
        y = np.zeros((self.n_y, self.n_der+1, n_samp))
        state0 = self.get_state(y0).ravel()
        state = np.zeros((self.n_st, 1, n_samp))
        # solves d_input/dt = f(input, time)
        # gives state for next points in time, specified by time_axis
        state[:, 0, :] = odeint(self._update_ode_model, state0, time_axis,
                                args=(u_interp,)).T
        y = self.get_y(state, input[:, :, :n_samp])
        return y

    def _update_ode_model(self, state, time, u_interp):
        u = u_interp(time)
        return self.model_update(state, u)

    def _integrate_plant(self, y0, input, sample_time, integration_time):
        # make interpolation function which returns the input at a certain time
        u_interp = self._get_input_interpolator(input, sample_time)
        n_samp = int(integration_time/sample_time)+1
        # time_axis is [0, integration time]
        time_axis = np.linspace(0., (n_samp-1)*sample_time, n_samp)
        y = np.zeros((self.n_y, self.n_der+1, n_samp))
        state0_a = self.get_state(y0).ravel()
        state0_b = self.get_input(y0).ravel()
        # current state
        state0 = np.r_[state0_a, state0_b]
        # state + inputs
        state = np.zeros((self.n_st+self.n_in, 1, n_samp))
        # u_interp is passed as an extra argument to _update_ode_plant()
        # that function gives the derivative of the state and input, based
        # on the current state and u_interp. These derivatives form the input
        # of odeint. odeint returns the future states.
        state[:, 0, :] = odeint(self._update_ode_plant, state0, time_axis,
                                args=(u_interp,)).T
        # get the pos, vel,... given the current state and the inputs
        y = self.get_y(state[:self.n_st, :, :], state[-self.n_in:, :, :])
        return y

    def _update_ode_plant(self, state, time, u_interp):
        # note: time can be a vector, then the multiple corresponding states
        # are returned
        u = u_interp(time)
        # put input into plant model, returns derivatives of state and input
        return self.plant_update(state, u)

    def _get_input_interpolator(self, input, sample_time):
        # this function generates a function to which interpolates between
        # different inputs
        n_samp = input.shape[2]
        time_axis = np.linspace(0, (n_samp-1)*sample_time, n_samp)
        # turn off error message, fill with last input value when extrapolating
        return interp1d(time_axis, input[:, 0, :], kind='linear',
                        bounds_error=False, fill_value=input[:, 0, -1])

    def plant_update(self, state, input):
        tau = self.options['time_constant']
        state_a = state[:self.n_st]
        state_b = state[-self.n_in:]
        dstate_a = self.model_update(state=state_a, input=state_b)
        dstate_b = (1./tau)*(-state_b + input)
        return np.r_[dstate_a, dstate_b]

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
