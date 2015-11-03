from vehicle import *

class Quadrotor(Vehicle):
    def __init__(self, options = {}, bounds = {}, shape = Quad(0.2), **kwargs):
        self.u1min   = bounds['u1min'] if 'u1min' in bounds else 1.
        self.u1max   = bounds['u1max'] if 'u1max' in bounds else 15.
        self.u2min   = bounds['u2min'] if 'u2min' in bounds else -8.
        self.u2max   = bounds['u2max'] if 'u2max' in bounds else 8.
        self.shape   = shape
        self.g       = 9.81
        self.safety_distance = self.shape.radius*0.2

        degree  = kwargs['degree'] if 'degree' in kwargs else 4
        index   = kwargs['index'] if 'index' in kwargs else 0

        self.n_y    = 2
        self.n_dy   = 6
        self.n_st   = 7
        self.n_in   = 4

        Vehicle.__init__(self, index, shape, degree, options)

    def getDisturbance(self, sample_time):
        stdev = np.r_[u1min, 0.1*u2max, 0., 0.] if self.input_dist else np.zeros(self.n_in)
        return {'fc': 2.*sample_time*3., 'stdev': stdev}

    def setInitialPosition(self, position):
        y   = np.vstack(position)
        dy  = np.zeros((self.n_dy,1))
        self.setInitialConditions(y, dy)

    def setTerminalPosition(self, position):
        y   = np.vstack(position)
        dy  = np.zeros((self.n_dy,1))
        self.setTerminalConditions(y,dy)

    def getState(self, y, dy):
        theta   = np.arctan2(dy[2,:], dy[3,:]+self.g)
        u1      = np.sqrt(dy[2,:]**2 + (dy[3,:] + self.g)**2)
        u2      = (dy[4,:]*(dy[3,:] + self.g) - dy[2,:]*dy[5,:])/(dy[2,:]**2 + (dy[3,:] + self.g)**2)
        return np.vstack((y, dy[0:2], theta, u1, u2))

    def getInput(self, y, dy):
        u1      = np.sqrt(dy[2,:]**2 + (dy[3,:] + self.g)**2)
        u2      = (dy[4,:]*(dy[3,:] + self.g) - dy[2,:]*dy[5,:])/(dy[2,:]**2 + (dy[3,:] + self.g)**2)
        return np.vstack((u1, u2, dy[4,:], dy[5,:]))

    def getPosition(self, y):
        return y

    def getDy(self, y_coeffs, knots, horizon_time, time_axis):
        dy = []
        dy.extend([(1./horizon_time)*splev(time_axis, (knots, y_coeffs[:,k], self.degree), 1) for k in range(self.n_y)])
        dy.extend([(1./horizon_time**2)*splev(time_axis, (knots, y_coeffs[:,k], self.degree), 2) for k in range(self.n_y)])
        dy.extend([(1./horizon_time**3)*splev(time_axis, (knots, y_coeffs[:,k], self.degree), 3) for k in range(self.n_y)])
        return np.vstack(dy)

    def updateModel(self, state, input, Ts):
        x_old   = state[0,:]
        z_old   = state[1,:]
        dx_old  = state[2,:]
        dz_old  = state[3,:]
        th_old  = state[4,:]
        u1_old  = state[5,:]
        u2_old  = state[6,:]

        u1  = input[0,:]
        u2  = input[1,:]
        th  = th_old+0.5*Ts*(u2+u2_old)
        dx  = dx_old+0.5*Ts*(u1*np.sin(th)+u1_old*np.sin(th_old))
        dz  = dz_old+0.5*Ts*(u1*np.cos(th)-self.g+u1_old*np.cos(th_old)-self.g)
        x   = x_old+0.5*Ts*(dx+dx_old)
        z   = z_old+0.5*Ts*(dz+dz_old)

        state  = np.vstack([x,z,dx,dz,th,u1,u2])
        y   = np.vstack([x,z])
        dy  = np.vstack([dx,dz,u1*np.sin(th),u1*np.cos(th)-self.g,input[2,:],input[3,:]])
        return y, dy, state

    def updateReal(self, state, input, Ts):
        # no model-plant mismatch implemented yet...
        return updateModel(self, state, input, Ts)

    def getConstraints(self, y, initial, terminal, boundary_smoothness, T, t = 0.):
        self.boundary_smoothness = boundary_smoothness
        dy = []
        for d in range(1,self.degree+1):
            dy.extend([y_l.derivative(d) for y_l in y])

        g_tf = self.g*(T**2)

        y0  = initial['y']
        dy0 = initial['dy']
        if 'y' in terminal:
            yT = terminal['y']
        dyT = terminal['dy']

        # Plant constraints
        spline_con = {}
        ddy     = [dy[2],dy[3]]
        dddy    = [dy[4],dy[5]]
        spline_con['u1min'] = -(ddy[0]**2 + (ddy[1]+g_tf)**2) + (T**4)*self.u1min**2
        spline_con['u1max'] = (ddy[0]**2 + (ddy[1]+g_tf)**2) - (T**4)*self.u1max**2
        spline_con['u2min'] = -(dddy[0]*(ddy[1]+g_tf) - ddy[0]*dddy[1]) + (ddy[0]**2 + (ddy[1] + g_tf)**2)*(T*self.u2min)
        spline_con['u2max'] = (dddy[0]*(ddy[1]+g_tf) - ddy[0]*dddy[1]) - (ddy[0]**2 + (ddy[1] + g_tf)**2)*(T*self.u2max)

        # Spline relaxation
        spline_entries = [entry(name, expr = spline.coeffs) for name,spline in spline_con.items()]

        # Initial constraints
        initial_entries = []
        initial_entries.append(entry('y0_0',    expr = evalspline(y[0], t/T) - y0[0]))
        initial_entries.append(entry('y0_1',    expr = evalspline(y[1], t/T) - y0[1]))
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
        return np.vstack(self.path['y'][:,k]) + self.shape.draw(-self.path['state'][4,k])

    def getPlotInfo(self, plot_type = None):
        info = {}
        info['y']       = {'names': ['x (m)', 'z (m)'], 'indices': range(self.n_y)}
        info['dy']      = {'names': ['v_x (m/s)', 'v_z (m/s)', 'a_x (m/s^2)', 'a_z (m/s^2)', 'j_x (m/s^3)', 'j_z (m/s^3)'], 'indices': range(self.n_dy)}
        info['input']   = {'names': ['Thrust force (N/kg)', 'Pitch rate (rad/s)', 'j_x (m/s^3)', 'j_z (m/s^3)'], 'indices': range(2)}
        info['state']   = {'names': ['x (m)', 'z (m)', 'v_x (m/s)', 'v_z (m/s)', 'theta(rad)'], 'indices': [0,1,4]}
        if plot_type == None:
            return info
        else:
            return info[plot_type]
