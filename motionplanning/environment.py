from optilayer import OptiLayer
from spline import BSplineBasis, BSpline
from spline_extra import definite_integral
from casadi import inf
import numpy as np


class Environment(OptiLayer):

    def __init__(self, room, obstacles=[], options={}):
        OptiLayer.__init__(self, 'environment')

        self.set_default_options()
        self.set_options(options)

        # create room and define dimension of the space
        self.room, self.n_dim = room, room['shape'].n_dim
        if not ('position' in room):
            self.room['position'] = [0. for k in range(self.n_dim)]

        # objective for environment
        self._objective = 0.

        # add obstacles
        self.obstacles, self.No = [], 0
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

    # ========================================================================
    # Environment options
    # ========================================================================

    def set_default_options(self):
        self.options = {}

    def set_options(self, options):
        self.options.update(options)

    # ========================================================================
    # Add obstacles
    # ========================================================================

    def add_obstacle(self, obstacle):
        if isinstance(obstacle, list):
            for obst in obstacle:
                self.add_obstacle(obst)
        else:
            obstacle.index = self.No
            self.obstacles.append(obstacle)
            self.No += 1

    def add_vehicle(self, vehicle):
        if isinstance(vehicle, list):
            for veh in vehicle:
                self.add_vehicle(veh)
        else:
            if not hasattr(self, 'basis'):
                # create spline basis: same knot sequence as vehicle
                # (but other degree)
                veh = vehicle
                self.degree = 1
                self.knots = np.r_[
                    np.zeros(self.degree), veh.knots[veh.degree:-veh.degree],
                    np.ones(self.degree)]
                self.knot_intervals = len(veh.knots[veh.degree:-veh.degree])-1
                self.basis = BSplineBasis(self.knots, self.degree)
                # define time information
                self.options['horizon_time'] = veh.options['horizon_time']
                self.options['sample_time'] = veh.options['sample_time']

            self._add_vehicleconstraints(vehicle)

    def _add_vehicleconstraints(self, vehicle):
        safety_distance = vehicle.options['safety_distance']
        safety_weight = vehicle.options['safety_weight']

        chck_veh, rad_veh = vehicle.get_checkpoints()
        for obstacle in self.obstacles:
            chck_obs = self.define_spline_parameter(
                'chck_'+str(obstacle.index), self.n_dim,
                obstacle.shape.n_chck, basis=obstacle.basis)
            if obstacle.shape.n_chck == 1:
                chck_obs = [chck_obs]
            rad_obs = self.define_parameter('rad_'+str(obstacle.index))

            a_hp = self.define_spline_variable(
                'a_' + str(vehicle.index)+str(obstacle.index), self.n_dim)
            b_hp = self.define_spline_variable(
                'b_' + str(vehicle.index)+str(obstacle.index))[0]
            if safety_distance > 0.:
                t, T = self.define_symbol('t'), self.define_symbol('T')
                eps = self.define_spline_variable(
                    'eps_'+str(vehicle.index)+str(obstacle.index))[0]
                self._objective += (safety_weight *
                                    definite_integral(eps, t/T, 1.))
                self.define_constraint(eps - safety_distance, -inf, 0.)
                self.define_constraint(-eps, -inf, 0.)
            else:
                eps = 0.

            for chck in chck_veh:
                self.define_constraint(sum(
                    [a_hp[k]*chck[k] for k in range(self.n_dim)]) -
                    b_hp + rad_veh + 0.5*(safety_distance-eps), -inf, 0.)
            for chck in chck_obs:
                self.define_constraint(-sum(
                    [a_hp[k]*chck[k] for k in range(self.n_dim)]) +
                    b_hp + rad_obs + 0.5*(safety_distance-eps), -inf, 0.)
            self.define_constraint(sum(
                [a_hp[k]*a_hp[k] for k in range(self.n_dim)])-1., -inf, 0.)
        self.define_objective(self._objective)

        # simple room constraint
        lim = self.get_canvas_limits()
        for chck in chck_veh:
            for k in range(self.n_dim):
                self.define_constraint(-chck[k] + lim[k][0], -inf, 0.)
                self.define_constraint(chck[k] - lim[k][1], -inf, 0.)

    # ========================================================================
    # Update environment
    # ========================================================================

    def update(self, current_time, update_time):
        for obstacle in self.obstacles:
            obstacle.update(update_time, self.options['sample_time'])

    # ========================================================================
    # Other required methods
    # ========================================================================

    def init_variables(self):
        return {}

    def get_parameters(self, current_time):
        parameters = {}
        for obstacle in self.obstacles:
            # current time relative to begin of horizon + horizon time
            knot_time = (int(self.options['horizon_time']*1000.) /
                         self.knot_intervals)/1000.
            t = np.round(current_time, 6) % knot_time
            T = self.options['horizon_time']
            x_obs = obstacle.get_positionspline(t, T)
            # checkpoints
            chck_obs, rad_obs = obstacle.get_checkpoints(x_obs)
            for l, chck in enumerate(chck_obs):
                if len(chck_obs) == 1:
                    name = 'chck_'+str(obstacle.index)
                else:
                    name = 'chck_'+str(obstacle.index)+str(l)
                parameters[name] = np.hstack([np.c_[chck[k].coeffs]
                                             for k in range(self.n_dim)])
            parameters['rad_'+str(obstacle.index)] = rad_obs
        return parameters

    def draw(self, t=-1):
        draw = []
        for obstacle in self.obstacles:
            draw.append(obstacle.draw(t))
        return draw

    def get_canvas_limits(self):
        limits = self.room['shape'].get_canvas_limits()
        return [limits[k]+self.room['position'][k] for k in range(self.n_dim)]


class Obstacle:

    def __init__(self, initial, shape, trajectory={}):
        self.shape = shape
        self.n_dim = shape.n_dim
        self.basis = BSplineBasis([0, 0, 0, 1, 1, 1], 2)

        self.path = {}
        self.path['time'] = np.array([0.])
        self.path['position'] = np.c_[initial['position']]
        self.path['velocity'] = np.zeros((self.n_dim, 1))
        self.path['acceleration'] = np.zeros((self.n_dim, 1))

        if 'velocity' in initial:
            self.path['velocity'] = np.c_[initial['velocity']]
        if 'acceleration' in initial:
            self.path['acceleration'] = np.c_[initial['acceleration']]

        if 'position' in trajectory:
            self.pos_traj, self.pos_index = trajectory['position'], 0
        if 'velocity' in trajectory:
            self.vel_traj, self.vel_index = trajectory['velocity'], 0
        if 'acceleration' in trajectory:
            self.acc_traj, self.acc_index = trajectory['acceleration'], 0

    def draw(self, t=-1):
        return np.c_[self.path['position'][:, t]] + self.shape.draw()

    def get_positionspline(self, current_time, horizon_time):
        # current observed pos, vel, acc of obstacle
        x = self.path['position'][:, -1]
        v = self.path['velocity'][:, -1]
        a = self.path['acceleration'][:, -1]
        # current time relative to begin of horizon + horizon time
        t = current_time
        T = horizon_time
        # pos, vel, acc at time zero of time horizon
        v0 = v - t*a
        x0 = x - t*v0 - 0.5*(t**2)*a
        a0 = a
        # pos spline over time horizon
        return [BSpline(self.basis, [x0[k], 0.5*v0[k]*T + x0[k], x0[k] +
                                     v0[k]*T + 0.5*a0[k]*(T**2)])
                for k in range(self.n_dim)]

    def get_checkpoints(self, position):
        return self.shape.get_checkpoints(position)

    def reset(self):
        self.time = 0.
        self.pos_index = 0
        self.vel_index = 0
        self.acc_index = 0

    def update(self, update_time, sample_time):
        n_samp = int(update_time/sample_time)
        for k in range(n_samp):
            dpos, dvel, dacc = np.zeros(2), np.zeros(2), np.zeros(2)
            t1_vel, t2_vel = sample_time, 0.
            t1_acc, t2_acc = sample_time, 0.
            time = self.path['time'][-1] + sample_time
            if (hasattr(self, 'pos_traj') and not
                    (self.pos_index >= self.pos_traj.shape[0])):
                if np.round(time - self.pos_traj[self.pos_index, 0], 3) >= 0.:
                    dpos = self.pos_traj[self.pos_index, 1:3]
                    self.pos_index += 1
            if (hasattr(self, 'vel_traj') and not
                    (self.vel_index >= self.vel_traj.shape[0])):
                if np.round(time - self.vel_traj[self.vel_index, 0], 3) >= 0.:
                    t1_vel = self.vel_traj[
                        self.vel_index, 0] - time + update_time
                    t2_vel = time - self.vel_traj[self.vel_index, 0]
                    dvel = self.vel_traj[self.vel_index, 1:3]
                    self.vel_index += 1
            if (hasattr(self, 'acc_traj') and not
                    (self.acc_index >= self.acc_traj.shape[0])):
                if np.round(time - self.acc_traj[self.acc_index, 0], 3) >= 0.:
                    t1_acc = self.acc_traj[
                        self.acc_index, 0] - time + update_time
                    t2_acc = time - self.acc_traj[self.acc_index, 0]
                    dacc = self.acc_traj[self.acc_index, 1:3]
                    self.acc_index += 1

            pos0 = self.path['position'][:, -1]
            vel0 = self.path['velocity'][:, -1]
            acc0 = self.path['acceleration'][:, -1]

            position = (pos0 + dpos + t1_vel*vel0 + t2_vel*(vel0 + dvel) +
                        0.5*(t1_acc**2)*acc0 + 0.5*(t2_acc**2)*(acc0 + dacc))
            velocity = (vel0 + dvel + t1_acc*acc0 + t2_acc*(acc0 + dacc))
            acceleration = acc0 + dacc

            self.path['time'] = np.r_[self.path['time'], time]
            self.path['position'] = np.c_[self.path['position'], position]
            self.path['velocity'] = np.c_[self.path['velocity'], velocity]
            self.path['acceleration'] = np.c_[self.path['acceleration'],
                                              acceleration]
