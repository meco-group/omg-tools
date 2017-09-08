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
from ..basics.shape_HolonomicF import shape_HolonomicF
from ..basics.spline_extra import sample_splines
from casadi import inf
import numpy as np

class HolonomicF(Vehicle):

    def __init__(self, mass_i, mass_j, mass_k, Ki, Kj, Kk, Ci, Cj, Ck, mass_0, Lfree, tuning_weights, shapes=shape_HolonomicF(0.1), options=None, bounds=None):
        bounds = bounds or {}
        #n_spl = 10
        #x11,y11,x12,y12,x13,y13,x10,y10,ax10_approx,ay10_approx
        Vehicle.__init__(
            self, n_spl=10, degree=3, shapes=shapes, options=options)
        self.vmin = bounds['vmin'] if 'vmin' in bounds else -0.5
        self.vmax = bounds['vmax'] if 'vmax' in bounds else 0.5
        self.amin = bounds['amin'] if 'amin' in bounds else -0.5
        self.amax = bounds['amax'] if 'amax' in bounds else 0.5

        #Vehicle Dynamic Parameters
        self.mi = mass_i
        self.mj = mass_j
        self.mk = mass_k
        self.Ki = Ki
        self.Kj = Kj
        self.Kk = Kk
        self.Ci = Ci
        self.Cj = Cj
        self.Ck = Ck
        self.m0 = mass_0
        self.Lfree = Lfree

        #Weights
        self.tuning_weights = tuning_weights

        #Payload transport problem bounds
        self.Fmin = bounds['Fmin'] if 'Fmin' in bounds else -300.0
        self.Fmax = bounds['Fmax'] if 'Fmax' in bounds else 300.0
        #self.Lmin = bounds['Lmin'] if 'Lmin' in bounds else self.Lfree*0.5
        #self.Lmax = bounds['Lmax'] if 'Lmax' in bounds else self.Lfree*2
        #self.Lmin = bounds['Lmin'] if 'Lmin' in bounds else 0.1
        #self.Lmax = bounds['Lmax'] if 'Lmax' in bounds else 0.5
        self.Lmin = bounds['Lmin'] if 'Lmin' in bounds else 0.2
        self.Lmax = bounds['Lmax'] if 'Lmax' in bounds else 0.5
        self.ThetaMin = bounds['ThetaMin'] if 'ThetaMin' in bounds else 30.*(np.pi/180.)
        self.ThetaMax = bounds['ThetaMax'] if 'ThetaMax' in bounds else 60.*(np.pi/180.)
        self.BackDispLim = bounds['BackDispLim'] if 'BackDispLim' in bounds else 0.1
        ############################################################################

    def set_default_options(self):
        Vehicle.set_default_options(self)
        self.options.update({'syslimit': 'norm_inf'})

    def init(self):
        # time horizon
        self.T = self.define_symbol('T')

    ######################################################################################################
    def assign_payload_accn_splines(self,splines):
        #Change basis of ax10_approx,ay10_approx splines to one representing the whole equation
        # m0ax0 = sigma(Ki*(xi-x0) + Ci*(vxi-vx0)
        x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx= splines
        vx11, vy11 = x11.derivative(), y11.derivative()
        vx12, vy12 = x12.derivative(), y12.derivative()
        vx13, vy13 = x13.derivative(), y13.derivative()
        vx10, vy10 = x10.derivative(), y10.derivative()

        #Required Parameters
        K1 = self.Ki
        K2 = self.Kj
        K3 = self.Kk
        C1 = self.Ci
        C2 = self.Cj
        C3 = self.Ck
        m0 = self.m0
        T = self.T

        #expr = (-(T**2)*(K1*(x11-x10) + K2*(x12-x10) + K3*(x13-x10)) - T*(C1*(vx11-vx10) + C2*(vx12-vx10) + C3*(vx13-vx10)))*(1/m0)

        #expr = (-(T**2)*(K1*(y11-y10) + K2*(y12-y10) + K3*(y13-y10)) - T*(C1*(vy11-vy10) + C2*(vy12-vy10) + C3*(vy13-vy10)))*(1/m0)

        expr = m0*ax10_approx - (T**2)*(K1*(x11-x10) + K2*(x12-x10) + K3*(x13-x10)) - T*(C1*(vx11-vx10) + C2*(vx12-vx10) + C3*(vx13-vx10))
    #####################################################################################################

    def define_trajectory_constraints(self, splines):
        x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx= splines

        vx11, vy11 = x11.derivative(), y11.derivative()
        vx12, vy12 = x12.derivative(), y12.derivative()
        vx13, vy13 = x13.derivative(), y13.derivative()
        vx10, vy10 = x10.derivative(), y10.derivative()

        ax11, ay11 = x11.derivative(2), y11.derivative(2)
        ax12, ay12 = x12.derivative(2), y12.derivative(2)
        ax13, ay13 = x13.derivative(2), y13.derivative(2)
        ax10, ay10 = x10.derivative(2), y10.derivative(2)

        #Pull out variables for ease of use
        m1 = self.mi
        K1 = self.Ki
        K2 = self.Kj
        K3 = self.Kk
        C1 = self.Ci
        C2 = self.Cj
        C3 = self.Ck
        m0 = self.m0
        T = self.T
        F_veh_max = self.Fmax
        F_veh_min = self.Fmin
        v_max = self.vmax
        v_min = self.vmin
        a_max = self.amax
        a_min = self.amin
        ThetaMin = self.ThetaMin
        ThetaMax = self.ThetaMax
        BackDispLim = self.BackDispLim
        Lmin = self.Lmin
        Lmax = self.Lmax
        ####################################

        #Local Vehicle Constraints
        # Vehicle Dynamic Constraints ##############
        expr = m1*ax11 + (T**2)*(K1*(x11-x10)) + T*C1*(vx11-vx10)
        self.define_constraint(expr - (T**2)*F_veh_min, 0., np.inf)
        self.define_constraint(expr - (T**2)*F_veh_max, -1.*np.inf, 0.)

        expr = m1*ay11 + (T**2)*(K1*(y11-y10)) + T*C1*(vy11-vy10)
        self.define_constraint(expr - (T**2)*F_veh_min, 0., np.inf)
        self.define_constraint(expr - (T**2)*F_veh_max, -1.*np.inf, 0.)
        ###########################################

        # Payload Dynamic Constraints #############
        eps = 0.1
        expr = m0*ax10_approx - (T**2)*(K1*(x11-x10) + K2*(x12-x10) + K3*(x13-x10)) - T*(C1*(vx11-vx10) + C2*(vx12-vx10) + C3*(vx13-vx10))
        self.define_constraint(expr, -eps, eps)

        expr = m0*ay10_approx - (T**2)*(K1*(y11-y10) + K2*(y12-y10) + K3*(y13-y10)) - T*(C1*(vy11-vy10) + C2*(vy12-vy10) + C3*(vy13-vy10))
        self.define_constraint(expr, -eps, eps)
        ###########################################

        # Velocity Constraints ####################
        self.define_constraint(vx11 - T*v_min, 0, np.inf)
        self.define_constraint(vx11 - T*v_max, -1.*np.inf, 0.)
        self.define_constraint(vy11 - T*v_min, 0, np.inf)
        self.define_constraint(vy11 - T*v_max, -1.*np.inf, 0.)
        ###########################################

        # Acceleration Constraints ####################
        self.define_constraint(ax11 - (T**2)*a_min, 0, np.inf)
        self.define_constraint(ax11 - (T**2)*a_max, -1.*np.inf, 0.)
        self.define_constraint(ay11 - (T**2)*a_min, 0, np.inf)
        self.define_constraint(ay11 - (T**2)*a_max, -1.*np.inf, 0.)
        ###########################################

        #Spring Length##############
        expr = (x11-x10)*(x11-x10) +(y11-y10)*(y11-y10)
        self.define_constraint(expr, Lmin*Lmin, Lmax*Lmax)
        ###########################################

        #Angle Constraints##############
        if self.tuning_weights[3]==1:
            #Maintain at more than 90 degrees apart
            expr = ((x12-x10)*(x11-x10)) + ((y12-y10)*(y11-y10))
            self.define_constraint(expr, -inf, 0.)

            expr = ((x13-x10)*(x11-x10)) + ((y13-y10)*(y11-y10))
            self.define_constraint(expr, -inf, 0.)

######################################################################################################
    def get_initial_constraints(self, splines):
        #defineParameter defines an MX variable
        #Note that on each spline passed into the output, the initial condition constraint is put up
        state0 = self.define_parameter('state0', 16)
        input0 = self.define_parameter('input0', 6)

        #Make sense of the splines again########
        x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx= splines

        vx11, vy11 = x11.derivative(), y11.derivative()
        vx12, vy12 = x12.derivative(), y12.derivative()
        vx13, vy13 = x13.derivative(), y13.derivative()
        vx10, vy10 = x10.derivative(), y10.derivative()

        ax11, ay11 = x11.derivative(2), y11.derivative(2)
        ax12, ay12 = x12.derivative(2), y12.derivative(2)
        ax13, ay13 = x13.derivative(2), y13.derivative(2)
        ax10, ay10 = x10.derivative(2), y10.derivative(2)

        #Parameters
        m1 = self.mi
        m2 = self.mj
        m3 = self.mk
        K1 = self.Ki
        K2 = self.Kj
        K3 = self.Kk
        T = self.T

        #Initial Inputs
        Fx11= m1*ax11*(1./(np.power(T,2))) + (K1*(x11-x10))
        Fy11= m1*ay11*(1./(np.power(T,2))) + (K1*(y11-y10))

        Fx12= m2*ax12*(1./(np.power(T,2))) + (K2*(x12-x10))
        Fy12= m2*ay12*(1./(np.power(T,2))) + (K2*(y12-y10))

        Fx13= m3*ax13*(1./(np.power(T,2))) + (K3*(x13-x10))
        Fy13= m3*ay13*(1./(np.power(T,2))) + (K3*(y13-y10))

        return [(x10, state0[0],'hard'), (vx10, self.T*state0[1],'soft'),
                (x11, state0[2],'hard'), (vx11, self.T*state0[3],'soft'),
                (y10, state0[8],'hard'), (vy10, self.T*state0[9],'soft'),
                (y11, state0[10],'hard'), (vy11, self.T*state0[11],'soft')]
######################################################################################################
    def get_terminal_constraints(self, splines):
        position = self.define_parameter('poseT', 4)
        #Final Conditions are purely on position - [x11,y11],[x10,y10]

        x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx = splines

        term_con = [(x11, position[0]), (y11, position[1]), (x10, position[2]), (y10, position[3])]
        #term_con = [(x10, position[2]), (y10, position[3])]
        term_con_der = []
        #All higher derivatives are set to zero for all the splines
        for d in range(1, self.degree+1):
            term_con_der.extend([(x11.derivative(d), 0.), (y11.derivative(d), 0.)])
            term_con_der.extend([(x12.derivative(d), 0.), (y12.derivative(d), 0.)])
            term_con_der.extend([(x13.derivative(d), 0.), (y13.derivative(d), 0.)])
            term_con_der.extend([(x10.derivative(d), 0.), (y10.derivative(d), 0.)])

        term_con_der.extend([(ax10_approx, 0.), (ay10_approx, 0.)]) #Approximate derivatives enforced to zero as well
        return [term_con, term_con_der]
######################################################################################################
    def set_initial_conditions(self, position, input=None):
        if input is None:
            input = np.zeros(6)
        self.prediction['state'] = position
        self.prediction['input'] = input

    def set_terminal_conditions(self, position):
        self.poseT = position

    def get_init_spline_value(self):
        init_value = np.zeros((len(self.basis), self.n_spl))
        pos0 = self.prediction['state']
        posT = self.poseT
        enum = 0
        enum_spl = 0

        for k in [2,10]:
            #x11,y11
            init_value[:, enum_spl] = np.linspace(pos0[k], posT[enum], len(self.basis))
            enum+=1
            enum_spl+=1

        for k in [4,12,6,14]:
            #x12,y12,
            init_value[:, enum_spl] = np.linspace(pos0[k], 0., len(self.basis))
            enum_spl+=1

        for k in [0,8]:
            #x10,y10
            init_value[:, enum_spl] = np.linspace(pos0[k], posT[enum], len(self.basis))
            enum+=1
            enum_spl+=1
        return init_value

    def check_terminal_conditions(self):
        tol = self.options['stop_tol']
        x0_term = self.signals['state'][0, -1]
        y0_term = self.signals['state'][8, -1]
        term_vec = [x0_term,y0_term]
        var_x0 = term_vec[0]- self.poseT[2]
        var_y0 = term_vec[1]- self.poseT[3]
        velocities = self.signals['velocities'][:, -1]
        #if (np.linalg.norm([var_x0,var_y0]) > tol or
        #        np.linalg.norm(self.signals['input'][:, -1])) > tol:
        if np.sqrt((var_x0*var_x0) + (var_y0*var_y0))>=tol:
            if np.sqrt((velocities[0]*velocities[0]) + (velocities[1]*velocities[1]))>=0.000001:
                return False
            else:
                return True
        else:
            return True

    def set_parameters(self, current_time):
        parameters = Vehicle.set_parameters(self, current_time)
        parameters[self]['state0'] = self.prediction['state']
        parameters[self]['input0'] = self.prediction['input']
        parameters[self]['poseT'] = self.poseT
        return parameters

    def define_collision_constraints(self, hyperplanes, environment, splines):
        x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx = splines
        self.define_collision_constraints_2d(hyperplanes, environment, [x11, y11])
        self.define_collision_constraints_2d(hyperplanes, environment, [x10, y10])
        #self.define_collision_constraints_2d(hyperplanes, environment, [x12, y12])
        #self.define_collision_constraints_2d(hyperplanes, environment, [x13, y13])
        #The method of putting hyperplanes on the convex hull of the setup does seem to work for a few cases.
        #But if its too drastic, like right in the middle, we need consensus on the hyperplanes as well.
        use_planes = 1
        enum = 0
        if use_planes == 1:
            for obstacle in environment.obstacles:
                obstacle_pos = obstacle.initial['position']
                a = self.define_spline_variable('a_'+str(enum)+self.label, 1, basis=x11.basis)
                b = self.define_spline_variable('b_'+str(enum)+self.label, 1, basis=x11.basis)
                c = self.define_spline_variable('c_'+str(enum)+self.label, 1, basis=x11.basis)
                self.define_constraint(a[0]*x11 + b[0]*y11 +c[0] , 0., inf)
                self.define_constraint(a[0]*x12 + b[0]*y12 +c[0] , 0., inf)
                self.define_constraint(a[0]*x13 + b[0]*y13 +c[0] , 0., inf)
                self.define_constraint(a[0]*x10 + b[0]*y10 +c[0] , 0., inf)
                self.define_constraint(a[0]*obstacle_pos[0] + b[0]*obstacle_pos[1] +c[0] , -inf, -0.1)
                self.define_constraint(a[0]*a[0] + b[0]*b[0], 0., 1.)
                enum+=1

    def splines2signals(self, splines, time):
        signals = {}
        x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx = splines

        vx11, vy11 = x11.derivative(), y11.derivative()
        vx12, vy12 = x12.derivative(), y12.derivative()
        vx13, vy13 = x13.derivative(), y13.derivative()
        vx10, vy10 = x10.derivative(), y10.derivative()

        ax11, ay11 = x11.derivative(2), y11.derivative(2)
        ax12, ay12 = x12.derivative(2), y12.derivative(2)
        ax13, ay13 = x13.derivative(2), y13.derivative(2)
        ax10, ay10 = x10.derivative(2), y10.derivative(2)

        #Parameters
        m1 = self.mi
        m2 = self.mj
        m3 = self.mk
        K1 = self.Ki
        K2 = self.Kj
        K3 = self.Kk
        C1 = self.Ci
        C2 = self.Cj
        C3 = self.Ck
        m0 = self.m0

        accnx_residue = ax10 - ax10_approx*0.0025
        accny_residue = ay10 - ay10_approx*0.0025

        #Inputs
        Fx11= m1*ax11+ (K1*(x11-x10)) + C1*(vx11-vx10)
        Fy11= m1*ay11 + (K1*(y11-y10)) + C1*(vy11-vy10)

        Fx12= m2*ax12+ (K2*(x12-x10)) + C2*(vx12-vx10)
        Fy12= m2*ay12 + (K2*(y12-y10)) + C2*(vy12-vy10)

        Fx13= m3*ax13 + (K3*(x13-x10)) + C3*(vx13-vx10)
        Fy13= m3*ay13+ (K3*(y13-y10)) + C3*(vy13-vy10)

        input = np.c_[sample_splines([Fx11, Fx12, Fx13, Fy11, Fy12, Fy13], time)]

        velocity_i = np.c_[sample_splines([vx11,vy11], time)]
        acceleration_i = np.c_[sample_splines([ax11,ay11], time)]

        x1_sampled = sample_splines(x11,time)
        y1_sampled = sample_splines(y11,time)
        x0_sampled = sample_splines(x10,time)
        y0_sampled = sample_splines(y10,time)

        length = np.sqrt(np.power(x1_sampled-x0_sampled,2) + np.power(y1_sampled-y0_sampled,2))
        x_pos_new = x1_sampled + self.Lfree*(x1_sampled - x0_sampled)/length
        y_pos_new = y1_sampled + self.Lfree*(y1_sampled - y0_sampled)/length

        #signals['local_positions'] = np.c_[[x_pos_new,y_pos_new]]
        signals['local_positions'] = np.c_[[x1_sampled,y1_sampled]]
        signals['state'] = np.c_[sample_splines([x10, vx10, x11, vx11, x12, vx12, x13, vx13, y10, vy10, y11, vy11, y12, vy12, y13, vy13], time)]
        signals['input'] = input
        signals['velocities'] = velocity_i
        signals['accelerations'] = acceleration_i
        signals['v_tot'] = np.sqrt(velocity_i[0, :]**2 + velocity_i[1, :]**2)
        signals['acceleration_residue'] = np.c_[sample_splines([accnx_residue,accny_residue], time)]
        signals['vehicle_j_trajs'] = np.c_[sample_splines([x12, y12], time)]
        signals['vehicle_k_trajs'] = np.c_[sample_splines([x13, y13], time)]
        signals['payload_pos'] = np.c_[sample_splines([x10, y10], time)]
        return signals

    def state2pose(self, state):
        return np.r_[state, 0.]

    def ode(self, state, input):
        #Parameters
        m1 = self.mi
        m2 = self.mj
        m3 = self.mk
        K1 = self.Ki
        K2 = self.Kj
        K3 = self.Kk
        C1 = self.Ci
        C2 = self.Cj
        C3 = self.Ck
        m0 = self.m0

        #SIMULATION WITH ACTUAL DYNAMICS (State - [x0,vx0,x1,vx1,x2,vx2,x3,vx3,y0,vy0,v1,vy1,y2,vy2,y3,vy3]', Input - [Fx1,Fx2,Fx3,Fy1,Fy2,Fy3]')
        #This should work irrspecive of the order as Ks and Cs get adjusted accordingly....
        state_space = 0
        if state_space == 1:
            A = np.array([[0.,                     1.,         0.,        0.,       0.,        0.,       0.,        0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [-(K1+K2+K3)/m0,   -(C1+C2+C3)/m0,      K1/m0,     C1/m0,    K2/m0,     C2/m0,    K3/m0,     C3/m0, 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0.,                     0.,         0.,        1.,       0.,        0.,       0.,        0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [(K1/m1),             C1/m1,   -(K1/m1),  -(C1/m1),       0.,        0.,       0.,        0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0.,                     0.,         0.,        0.,       0.,        1.,       0.,        0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [(K2/m2),             C2/m2,         0.,        0.,   -K2/m2,  -(C2/m2),       0.,        0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0.,                     0.,         0.,        0.,       0.,        0.,       0.,        1., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [(K3/m3),             C3/m3,         0.,        0.,       0.,        0.,   -K3/m3,  -(C3/m3), 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0.,                     1.,         0.,        0.,       0.,        0.,       0.,        0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., -(K1+K2+K3)/m0,   -(C1+C2+C3)/m0,      K1/m0,     C1/m0,    K2/m0,     C2/m0,    K3/m0,     C3/m0],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0.,                     0.,         0.,        1.,       0.,        0.,       0.,        0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., (K1/m1),             C1/m1,   -(K1/m1),  -(C1/m1),       0.,        0.,       0.,        0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0.,                     0.,         0.,        0.,       0.,        1.,       0.,        0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., (K2/m2),             C2/m2,         0.,        0.,   -K2/m2,  -(C2/m2),       0.,        0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0.,                     0.,         0.,        0.,       0.,        0.,       0.,        1.],
                                [0., 0., 0., 0., 0., 0., 0., 0., (K3/m3),             C3/m3,         0.,        0.,       0.,        0.,   -K3/m3,  -(C3/m3)]])

            B = np.array([[  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [1/m1,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,  1/m2,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0., 1/m3,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0., 1/m1,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0., 1/m2,   0.],
                                [  0.,    0.,   0.,   0.,   0.,   0.],
                                [  0.,    0.,   0.,   0.,   0., 1/m3]])
            return np.dot(A,state) + np.dot(B,input)
        else:
            x0 = state[0];
            vx0 = state[1];
            x1 = state[2];
            vx1 = state[3];
            x2 = state[4];
            vx2 = state[5];
            x3 = state[6];
            vx3 = state[7];

            y0 = state[8];
            vy0 = state[9];
            y1 = state[10];
            vy1 = state[11];
            y2 = state[12];
            vy2 = state[13];
            y3 = state[14];
            vy3 = state[15];

            Fx1 = input[0];
            Fx2 = input[1];
            Fx3 = input[2];
            Fy1 = input[3];
            Fy2 = input[4];
            Fy3 = input[5];

            A = np.array([vx0,
                                (K1*(x1-x0) + K2*(x2-x0) + K3*(x3-x0) + C1*(vx1-vx0) + C2*(vx2-vx0) + C3*(vx3-vx0))/m0,
                                vx1,
                                (Fx1 - K1*(x1-x0) - C1*(vx1-vx0))/m1,
                                vx2,
                                (Fx2 - K2*(x2-x0) - C2*(vx2-vx0))/m2,
                                vx3,
                                (Fx3 - K3*(x3-x0) - C3*(vx3-vx0))/m3,
                                vy0,
                               (K1*(y1-y0) + K2*(y2-y0) + K3*(y3-y0) + C1*(vy1-vy0) + C2*(vy2-vy0) + C3*(vy3-vy0))/m0,
                               vy1,
                               (Fy1 - K1*(y1-y0) - C1*(vy1-vy0))/m1,
                               vy2,
                               (Fy2 - K2*(y2-y0) - C2*(vy2-vy0))/m2,
                               vy3,
                               (Fy3 - K3*(y3-y0) - C3*(vy3-vy0))/m3])
            return A


#Functions that should actually be in Vehicle.py, but since they are exclusive, they are here
#############################################################################
    def get_spline_copies(self, splines):
        x11, y11, x12, y12, x13, y13, x10, y10, ax10_approx, ay10_approx = splines
        splines_neighbors_j = [x12,y12]
        splines_neighbors_k = [x13,y13]
        home_splines = [x11,y11]
        substitutes_j = self.define_substitute('neighbor_splines_of_'+str(self)+'_j_', splines_neighbors_j)
        substitutes_k = self.define_substitute('neighbor_splines_of_'+str(self)+'_k_', splines_neighbors_k)
        home_spline_x = self.define_substitute('home_spline_of_'+str(self)+'_x_', home_splines[0])
        home_spline_y = self.define_substitute('home_spline_of_'+str(self)+'_y_', home_splines[1])
        return substitutes_j,substitutes_k,home_spline_x,home_spline_y
###############################################################################

def draw(self, t=-1):
        ret = []
        for shape in self.shapes:
            ret += shape.draw(self.signals['pose'][:, t])
        return ret
########################################################
def draw_HolonomicF(self, t=-1):
    ret = []
    for shape in self.shapes:
        ret += shape.draw_HolonomicF(self.signals['pose'][:, t])
        ret += shape.draw_HolonomicF_payload(self.signals['pose'][:, t])
        #Draw Springs - As Lines
        pose = self.signals['pose'][:, t]
        veh_x = pose[2]
        veh_y = pose[10]
        payload_x = pose[0]
        payload_y = pose[8]
        length = np.sqrt(np.power(veh_x-payload_x,2) + np.power(veh_y-payload_y,2))
        x_pos_new = veh_x + self.Lfree*(veh_x - payload_x)/length
        y_pos_new = veh_y + self.Lfree*(veh_y - payload_y)/length
        x_spring = np.linspace(pose[0],pose[2],20)
        y_spring = np.linspace(pose[8],pose[10],20)
        #x_spring = np.linspace(pose[0],x_pos_new,20)
        #y_spring = np.linspace(pose[8],y_pos_new,20)
        for i in range(len(x_spring[:-1])):
            temp = np.zeros(2);
            temp[0] = x_spring[i]
            temp[1] = y_spring[i]
            ret += [np.c_[[x_spring[i],y_spring[i]],[x_spring[i+1],y_spring[i+1]]]]
    return ret
########################################################

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
    #for k in range(ax_r): ###CHANGING FOR HOLONOMICF ALONE - Will have (Fx11,Fyii,vxii,vyii) per vehicle
    if signal == 'lengths' or signal == 'angles' or signal == 'velocities_x'  or signal == 'velocities_y':
        range_s = 3
    else:
        range_s = 2
    for k in range(range_s):
        inf = []
        for _ in range(ax_c):
            lines = []
            lines.append(
                {'linestyle': '-', 'color': self.colors_w[index]})
            lines.append(
                {'linestyle': '-', 'color': self.colors[index]})
            if 'knots' in kwargs and kwargs['knots']:
                lines.append(
                    {'linestyle': 'None', 'marker': 'x', 'color': self.colors[index]})
            if 'prediction' in kwargs and kwargs['prediction']:
                lines.append(
                    {'linestyle': 'None', 'marker': 'o', 'color': self.colors[index]})
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

   # for k in range(ax_r):###CHANGING FOR HOLONOMICF ALONE
    if signal == 'input':
        to_plot = [0,3];
    elif signal == 'lengths' or signal == 'angles' or signal == 'velocities_x'  or signal == 'velocities_y':
        to_plot = [0,1,2];
    else:
        to_plot = [0,1];

    if signal == 'lengths' or signal == 'angles' or signal == 'velocities_x'  or signal == 'velocities_y':
        range_s = 3
    else:
        range_s = 2

    #for k in range(range_s):
    for k in range(ax_r):
        dat = []
        for l in range(ax_c):
            lines = []
            lines.append(
                [self.traj_storage['time'][t], self.traj_storage[signal][t][to_plot[k], :]])
            if t == -1:
                lines.append(
                    [self.signals['time'], self.signals[signal][to_plot[k], :]])
            else:
                lines.append(
                    [self.signals['time'][:, :t+1], self.signals[signal][to_plot[k], :t+1]])
            if 'knots' in kwargs and kwargs['knots']:
                lines.append(
                    [self.traj_storage_kn['time'][t], self.traj_storage_kn[signal][t][to_plot[k], :]])
            if 'prediction' in kwargs and kwargs['prediction']:
                lines.append(
                    [self.signals['time'][:, t], self.pred_storage[signal][t][to_plot[k]]])
            dat.append(lines)
        data.append(dat)
    return data



