from vehicle import *

class Holonomic(Vehicle):
    def __init__(self, options = {}, bounds = {}, shape = Circle(0.1), **kwargs):
        self.vmin   = bounds['vmin'] if 'vmin' in bounds else -0.5
        self.vmax   = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amin   = bounds['amin'] if 'amin' in bounds else -1.
        self.amax   = bounds['amax'] if 'amax' in bounds else 1.
        self.shape  = shape
        self.tau    = 0.1
        self.safety_distance = self.shape.radius*0.2

        degree  = kwargs['degree'] if 'degree' in kwargs else 3
        index   = kwargs['index'] if 'index' in kwargs else 0
        self.n_y    = 2
        self.n_dy   = (degree-1)*self.n_y
        if degree == 3:
            self.n_st   = 8
            self.n_in   = 4
        else:
            self.n_st   = 6
            self.n_in   = 2
        Vehicle.__init__(self, index, shape, degree, options)

    def getDisturbance(self, sample_time):
        stdev = 0.1*self.vmax*np.ones(self.n_in) if self.input_dist else np.zeros(self.n_in)
        return {'fc': 2.*sample_time*3., 'stdev': stdev}

    def setInitialPosition(self, position):
        y   = np.vstack(position)
        dy  = np.zeros((self.n_dy,1))
        self.setInitialConditions(y, dy)

    def setTerminalPosition(self, position):
        y   = np.vstack(position)
        dy  = np.zeros((self.n_dy,1))
        self.setTerminalConditions(y, dy)

    def getState(self, y, dy):
        return np.vstack((y, dy, dy[0:2,:]))

    def getInput(self, y, dy):
        return np.copy(dy)

    def getPosition(self, y):
        return y

    def getDy(self, y_coeffs, knots, horizon_time, time_axis):
        dy = []
        dy.extend([(1./horizon_time)*splev(time_axis, (knots, y_coeffs[:,k], self.degree), 1) for k in range(self.n_y)])
        if self.degree == 3:
            dy.extend([(1./(horizon_time**2))*splev(time_axis, (knots, y_coeffs[:,k], self.degree), 2) for k in range(self.n_y)])
        return np.vstack(dy)

    def updateModel(self, state, input, Ts):
        # Integrator, discretized via Tustin
        if self.degree == 3:
            a = np.array([[1., 0., 0., 0.5*Ts], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.]])
            b = np.array([[0.5*Ts, 0.], [1., 0.], [0., 1.], [1., 0.]])
        elif self.degree == 2:
            a = np.array([[1., 0., 0.5*Ts], [0., 0., 0.], [0., 0., 0.]])
            b = np.vstack([0.5*Ts, 1., 1.])

        A = np.kron(a, np.identity(self.n_y))
        B = np.kron(b, np.identity(self.n_y))

        state   = A.dot(state) + B.dot(input)
        y       = np.vstack(state[0:2,:])
        dy      = np.vstack(state[2:6,:]) if (self.degree == 3) else np.vstack(state[2:4,:])
        return y, dy, state

    def updateReal(self, state, input, Ts):
        # Integrator + 1st order system, discretized via Tustin
        if self.degree == 3:
            a = np.array([[1., (2*self.tau*Ts)/(2.*self.tau+Ts), 0., Ts**2/(4.*self.tau+2.)], [0., (2.*self.tau-Ts)/(2.*self.tau+Ts), 0., Ts/(2.*self.tau+Ts)], [0., 0., 0., 0.], [0., 0., 0., 0.]])
            b = np.array([[Ts**2/(4.*self.tau+2.*Ts), 0.], [Ts/(2.*self.tau+Ts), 0.], [0., 1.], [1., 0.]])
        elif self.degree == 2:
            a = np.array([[1., (2*self.tau*Ts)/(2.*self.tau+Ts), Ts**2/(4.*self.tau+2.)], [0., (2.*self.tau-Ts)/(2.*self.tau+Ts), Ts/(2.*self.tau+Ts)], [0., 0.,  0.]])
            b = np.vstack([Ts**2/(4.*self.tau+2.*Ts), Ts/(2.*self.tau+Ts),1.])

        A = np.kron(a, np.identity(self.n_y))
        B = np.kron(b, np.identity(self.n_y))

        state   = A.dot(state) + B.dot(input)
        y       = np.vstack(state[0:2,:])
        dy      = np.vstack(state[2:6,:]) if (self.degree == 3) else np.vstack(state[2:4,:])
        return y, dy, state

    def getConstraints(self, y, initial, terminal, boundary_smoothness, T, t = 0.):
        self.boundary_smoothness = boundary_smoothness
        dy = []
        for d in range(1,self.degree+1):
            dy.extend([y_l.derivative(d) for y_l in y])

        y0  = initial['y']
        dy0 = initial['dy']
        if 'y' in terminal:
            yT = terminal['y']
        dyT = terminal['dy']

        # Plant constraints
        spline_con = {}
        spline_con['vmin0'] = -dy[0] + T*self.vmin
        spline_con['vmin1'] = -dy[1] + T*self.vmin
        spline_con['vmax0'] = dy[0] - T*self.vmax
        spline_con['vmax1'] = dy[1] - T*self.vmax

        spline_con['amin0'] = -dy[2] + (T**2)*self.amin
        spline_con['amin1'] = -dy[3] + (T**2)*self.amin
        spline_con['amax0'] = dy[2] - (T**2)*self.amax
        spline_con['amax1'] = dy[3] - (T**2)*self.amax

        # Spline relaxation
        spline_entries = [entry(name, expr = spline.coeffs) for name,spline in spline_con.items()]

        # Initial constraints
        initial_entries = []
        initial_entries.append(entry('y0_0',   expr = evalspline(y[0], t/T) - y0[0]))
        initial_entries.append(entry('y0_1',   expr = evalspline(y[1], t/T) - y0[1]))
        for d in range(1,max(boundary_smoothness['initial'],boundary_smoothness['internal'])+1):
            for k in range(self.n_y):
                ind     = (d-1)*self.n_y+k
                dy0_    = dy0[ind] if (ind < dy0.size()) else 0.
                initial_entries.append(entry('dy0_'+str(ind),   expr = evalspline(dy[ind], t/T) - (T**d)*dy0_))

        # Terminal constraints
        terminal_entries = []
        if 'y' in terminal:
            terminal_entries.append(entry('yT_0',   expr = y[0](1.) - yT[0]))
            terminal_entries.append(entry('yT_1',   expr = y[1](1.) - yT[1]))
        for d in range(1,boundary_smoothness['terminal']+1):
            for k in range(self.n_y):
                ind     = (d-1)*self.n_y+k
                dyT_    = dyT[ind] if (ind < dyT.size()) else 0.
                terminal_entries.append(entry('dyT_'+str(ind),   expr = dy[ind](1.) - (T**d)*dyT_))

        constraints = struct_MX(spline_entries + initial_entries + terminal_entries)
        self.lbg    = constraints(-inf)
        self.ubg    = constraints(0)
        for ent in initial_entries:
            self.lbg[ent.name] = 0
        for ent in terminal_entries:
            self.lbg[ent.name] = 0

        return constraints, self.lbg, self.ubg, 0.

    def draw(self, k = -1):
        return np.vstack(self.path['y'][:,k]) + self.shape.draw()

    def getPlotInfo(self, plot_type = None):
        info = {}
        info['y']       = {'names': ['x (m)', 'y (m)'], 'indices': range(self.n_y)}
        if self.degree == 3:
            info['dy']      = {'names': ['v_x (m/s)', 'v_y (m/s)', 'a_x (m/s^2)', 'a_y (m/s^2)'], 'indices': range(self.n_dy)}
            info['input']   = {'names': ['v_x (m/s)', 'v_y (m/s)', 'a_x (m/s^2)', 'a_y (m/s^2)'], 'indices': range(2)}
            info['state']   = {'names': ['x (m)', 'y (m)', 'v_x (m/s)', 'v_y (m/s)', 'a_x (m/s^2)', 'a_y (m/s^2)', 'u_x (m/s)', 'u_y (m/s)'], 'indices': range(self.n_st)}
        if self.degree == 2:
            info['dy']      = {'names': ['v_x (m/s)', 'v_y (m/s)'], 'indices': range(self.n_dy)}
            info['input']   = {'names': ['v_x (m/s)', 'v_y (m/s)'], 'indices': range(2)}
            info['state']   = {'names': ['x (m)', 'y (m)', 'v_x (m/s)', 'v_y (m/s)', 'u_x (m/s)', 'u_y (m/s)'], 'indices': range(self.n_st)}
        if plot_type == None:
            return info
        else:
            return info[plot_type]
