import numpy as np
from shape import *
from vehicle import *

class Environment:
    def __init__(self, room, obstacles = []):
        self.room       = room
        self.obstacles  = obstacles
        self.No         = len(self.obstacles)

    def addObstacle(self, obstacle):
        if isinstance(obstacle, list):
            for obst in obstacle:
                self.addObstacle(obst)
        else:
            self.obstacles.append(obstacle)
            self.No += 1

    def transformSplines(self, variables, transformation):
        for l in range(self.No):
            variables['obstacles',l,'ca'] = transformation(variables['obstacles',l,'ca'], self.knots_hp, self.degree_hp)
            variables['obstacles',l,'cb'] = transformation(variables['obstacles',l,'cb'], self.knots_hp, self.degree_hp)
            if self.safety:
                variables['obstacles',l,'ceps'] = transformation(variables['obstacles',l,'ceps'], self.knots_hp, self.degree_hp)
        return variables

    def initVariables(self, variables):
        return variables

    def getVariableStruct(self, knot_intervals = 10, safety = False):
        self.degree_hp  = 1
        self.knots_hp   = np.r_[np.zeros(self.degree_hp), np.linspace(0., 1., knot_intervals+1), np.ones(self.degree_hp)]
        self.Basis_hp   = BSplineBasis(self.knots_hp, self.degree_hp)
        obstacle_struct = struct([entry('ca', shape=(len(self.Basis_hp),2)), entry('cb', shape=len(self.Basis_hp))])
        self.safety     = safety
        if self.safety:
            obstacle_struct = struct(obstacle_struct.entries + [entry('ceps', shape=len(self.Basis_hp))])
        return struct([entry('obstacles', struct=obstacle_struct, repeat=self.No)])

    def getParameterStruct(self):
        obstacle_struct = struct([entry('x',shape=2), entry('v',shape=2), entry('a',shape=2)])
        return struct([entry('obstacles', struct=obstacle_struct, repeat=self.No)])

    def setParameters(self, parameters):
        for l in range(self.No):
            parameters['obstacles',l,'x'] = self.obstacles[l].path['position'][:,-1]
            parameters['obstacles',l,'v'] = self.obstacles[l].path['velocity'][:,-1]
            parameters['obstacles',l,'a'] = self.obstacles[l].path['acceleration'][:,-1]
        return parameters

    def getConstraints(self, vehicle, vehicle_trajectory, environment_variables, environment_parameters, T, t):
        spline_con = {}
        # Anti-collision constraints
        chck_veh, rad_veh = vehicle.getCheckPoints(vehicle_trajectory)
        Basis_quad  = BSplineBasis([0, 0, 0, 1, 1, 1], 2)
        obj = 0.
        for l in range(self.No):
            a_hp    = [ BSpline(self.Basis_hp, environment_variables['obstacles',l,'ca',:,k]) for k in range(2)]
            b_hp    =   BSpline(self.Basis_hp, environment_variables['obstacles',l,'cb'])
            eps     =   (BSpline(self.Basis_hp, environment_variables['obstacles',l,'ceps']) - vehicle.safety_distance) if self.safety else 0.

            v0      = [environment_parameters['obstacles',l,'v',k] - mul(t,environment_parameters['obstacles',l,'a',k]) for k in range(2)]
            x0      = [environment_parameters['obstacles',l,'x',k] - mul(t,v0[k]) - 0.5*mul(t**2,environment_parameters['obstacles',l,'a',k]) for k in range(2)]
            a0      = [environment_parameters['obstacles',l,'a',k] for k in range(2)]
            x_obs   = [BSpline(Basis_quad, vertcat([x0[k], 0.5*mul(v0[k], T) + x0[k], x0[k] + mul(v0[k], T) + 0.5*mul(a0[k], T**2)])) for k in range(2)]

            chck_obs, rad_obs = self.obstacles[l].getCheckPoints(x_obs)
            for j in range(len(chck_veh)):
                spline_con['veh_hp_'+str(l)+'_'+str(j)] = rad_veh + a_hp[0]*chck_veh[j][0] + a_hp[1]*chck_veh[j][1] - b_hp - eps
            for j in range(len(chck_obs)):
                spline_con['obs_hp_'+str(l)+'_'+str(j)] = rad_obs - a_hp[0]*chck_obs[j][0] - a_hp[1]*chck_obs[j][1] + b_hp - eps
            spline_con['a_hp_'+str(l)]  = a_hp[0]*a_hp[0] + a_hp[1]*a_hp[1] - 1.
            if self.safety:
                spline_con['epsmin_'+str(l)] = -eps-vehicle.safety_distance
                spline_con['epsmax_'+str(l)] = eps
                obj += (100./self.No)*definite_integral(eps,t/T,1.)
        # Spline relaxation
        constraints = struct_MX([entry(name, expr = spline.coeffs) for name,spline in spline_con.items()])
        self.lbg    = constraints(-inf)
        self.ubg    = constraints(0)


        return constraints, self.lbg, self.ubg, obj

    def getLbgUbg(self):
        return self.lbg, self.ubg

    def draw(self, k = -1):
        draw = {}
        draw['obstacles'] = []
        for obstacle in self.obstacles:
            draw['obstacles'].append(obstacle.draw(k))
        return draw

    def update(self, update_time, sample_time):
        for obstacle in self.obstacles:
            obstacle.update(update_time, sample_time)

    def getCanvasLimits(self):
        limits = self.room['shape'].getCanvasLimits()
        limits['xlim'] += self.room['position'][0]
        limits['ylim'] += self.room['position'][1]
        return limits

class Obstacle:

    def __init__(self, initial, shape, trajectory = {}):
        self.shape  = shape

        self.path = {}
        self.path['position']       = np.vstack(initial['position'])
        self.path['velocity']       = np.vstack(initial['velocity'])        if 'velocity'       in initial else np.zeros(((2,1)))
        self.path['acceleration']   = np.vstack(initial['acceleration'])    if 'acceleration'   in initial else np.zeros(((2,1)))
        self.path['time']           = np.array([0.])

        if 'position' in trajectory:
            self.pos_traj   = trajectory['position']
            self.pos_index  = 0
        if 'velocity' in trajectory:
            self.vel_traj   = trajectory['velocity']
            self.vel_index  = 0
        if 'acceleration' in trajectory:
            self.acc_traj   = trajectory['acceleration']
            self.acc_index  = 0

    def draw(self, k = -1):
        return np.vstack(self.path['position'][:,k]) + self.shape.draw()

    def getCheckPoints(self, position):
        return self.shape.getCheckPoints(position)

    def reset(self):
        self.time           = 0.
        self.pos_index      = 0
        self.vel_index      = 0
        self.acc_index      = 0

    def update(self, update_time, sample_time):
        n_samp = int(update_time/sample_time)
        for k in range(n_samp):
            dpos    = np.zeros((2,1))
            dvel    = np.zeros((2,1))
            dacc    = np.zeros((2,1))
            t1_vel  = sample_time
            t2_vel  = 0.
            t1_acc  = sample_time
            t2_acc  = 0.
            time    = self.path['time'][-1] + sample_time
            if hasattr(self,'pos_traj') and not (self.pos_index >= self.pos_traj.shape[0]):
                if np.round(time - self.pos_traj[self.pos_index,0],3) >= 0.:
                    dpos    = np.vstack(self.pos_traj[self.pos_index,1:3])
                    self.pos_index += 1
            if hasattr(self, 'vel_traj') and not (self.vel_index >= self.vel_traj.shape[0]):
                if np.round(time - self.vel_traj[self.vel_index,0],3) >= 0.:
                    t1_vel  = self.vel_traj[self.vel_index,0] - time + update_time
                    t2_vel  = time - self.vel_traj[self.vel_index,0]
                    dvel    = np.vstack(self.vel_traj[self.vel_index,1:3])
                    self.vel_index += 1
            if hasattr(self, 'acc_traj') and not (self.acc_index >= self.acc_traj.shape[0]):
                if np.round(time - self.acc_traj[self.acc_index,0],3) >= 0.:
                    t1_acc  = self.acc_traj[self.acc_index,0] - time + update_time
                    t2_acc  = time - self.acc_traj[self.acc_index,0]
                    dacc    = np.vstack(self.acc_traj[self.acc_index,1:3])
                    self.acc_index += 1
            position       = np.vstack(self.path['position'][:,-1]) + dpos + t1_vel*np.vstack(self.path['velocity'][:,-1]) + t2_vel*(np.vstack(self.path['velocity'][:,-1])+dvel) + 0.5*(t1_acc**2)*np.vstack(self.path['acceleration'][:,-1]) + 0.5*(t2_acc**2)*(np.vstack(self.path['acceleration'][:,-1])+dacc)
            velocity       = np.vstack(self.path['velocity'][:,-1]) + dvel + t1_acc*np.vstack(self.path['acceleration'][:,-1]) + t2_acc*(np.vstack(self.path['acceleration'][:,-1])+dacc)
            acceleration   = np.vstack(self.path['acceleration'][:,-1]) + dacc
            self.path['time']           = np.append(self.path['time'], np.array([time]), axis = 1)
            self.path['position']       = np.append(self.path['position'], np.vstack(position), axis = 1)
            self.path['velocity']       = np.append(self.path['velocity'], np.vstack(velocity), axis = 1)
            self.path['acceleration']   = np.append(self.path['acceleration'], np.vstack(acceleration), axis = 1)
