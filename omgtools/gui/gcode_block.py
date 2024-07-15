import numpy as np

import warnings

class GCodeBlock(object):

    def __init__(self, command, number, prev_block=None, **kwargs):

        self.default_F = 444  # default feedrate [mm/min]
        self.default_S = 30000  # default rotation velocity [rev/min]

        if prev_block is not None:
            self.X0 = prev_block.X1
            self.Y0 = prev_block.Y1
            self.Z0 = prev_block.Z1
        else:
            if 'start_pos' in kwargs:
                self.X0 = kwargs['start_pos'][0]
                self.Y0 = kwargs['start_pos'][1]
                self.Z0 = kwargs['start_pos'][2]
            else:
                self.X0 = 0.
                self.Y0 = 0.
                self.Z0 = 0.

        self.X1 = command['X'] if 'X' in command else self.X0
        self.Y1 = command['Y'] if 'Y' in command else self.Y0
        self.Z1 = command['Z'] if 'Z' in command else self.Z0

        self.start = [self.X0, self.Y0, self.Z0]  # start position of block
        self.end = [self.X1, self.Y1, self.Z1]  # end position of block

        if 'F' in command:
            self.F = command['F']
        else:
            self.F = self.default_F
        if 'S' in command:
            self.S = command['S']
        else:
            self.S = self.default_S

class G00(GCodeBlock):
    def __init__(self, command, number, prev_block, **kwargs):
        GCodeBlock.__init__(self, command, number, prev_block, **kwargs)
        self.type = 'G00'

    def get_coordinates(self):
        start = [self.X0, self.Y0, self.Z0]
        end = [self.X1, self.Y1, self.Z1]
        return [start, end]

class G01(GCodeBlock):
    def __init__(self, command, number, prev_block):
        GCodeBlock.__init__(self, command, number, prev_block)
        self.type = 'G01'

    def get_coordinates(self):
        start = [self.X0, self.Y0, self.Z0]
        end = [self.X1, self.Y1, self.Z1]
        N = 4
        coords = []
        x_values = np.linspace(start[0], end[0], N)
        y_values = np.linspace(start[1], end[1], N)
        z_values = np.linspace(start[2], end[2], N)
        for x, y, z in zip(x_values, y_values, z_values):
            coords.append([x, y, z])
        return coords

    def get_velocity_profile(self, t0, v_max, a_max):
        # compute velocity profile, linear
        start = [self.X0, self.Y0, self.Z0]
        end = [self.X1, self.Y1, self.Z1]
        start = [coord/1000 for coord in start]
        end = [coord/1000 for coord in end]
        dist_to_max = v_max**2/(2*a_max)
        p = [[], []]
        v = [[], []]
        a = [[], []]
        t = []
        a_s = [[np.sign(end[0] - start[0])], 
               [np.sign(end[1] - start[1])]]

        dist_to_cover = [np.abs(start[0] - end[0]),
                         np.abs(start[1] - end[1])]
        bd = 0 if dist_to_cover[0] > dist_to_cover[1] else 1
        dist_ratio = dist_to_cover[1-bd]/dist_to_cover[bd]
        a_s[bd][0] = a_s[bd][0]*a_max
        a_s[1-bd][0] = a_s[1-bd][0]*a_max*dist_ratio

        N = 10

        curr_pos = [start[0], start[1]]
        ax = a_s[0][0]
        ay = a_s[1][0]

        if 2*dist_to_max > dist_to_cover[bd]:
            print('Warning: unable to reach max velocity')
            # accelerate
            t1 = np.sqrt(dist_to_cover[bd]/a_max)
            t.append(0.)
            v[0].append(0.)
            v[1].append(0.)
            p[0].append(curr_pos[0])
            p[1].append(curr_pos[1])
            for i in range(N):
                t_local = (i+1)*t1/N
                t.append(t_local)
                a[0].append(ax)
                a[1].append(ay)
                v[0].append(ax*t_local)
                v[1].append(ay*t_local)
                p[0].append(curr_pos[0] + ax*t_local**2/2)
                p[1].append(curr_pos[1] + ay*t_local**2/2)
            a[0].append(0)
            a[1].append(0)

            curr_pos = [p[0][-1], p[1][-1]]
            curr_vel = [v[0][-1], v[1][-1]]

            # brake
            ax = -ax
            ay = -ay
            t.append(t1)
            v[0].append(curr_vel[0])
            v[1].append(curr_vel[1])
            p[0].append(curr_pos[0])
            p[1].append(curr_pos[1])
            for i in range(N):
                t_local = (i+1)*t1/N
                t.append(t1 + t_local)
                a[0].append(ax)
                a[1].append(ay)
                v[0].append(curr_vel[0] + ax*t_local)
                v[1].append(curr_vel[1] + ay*t_local)
                p[0].append(curr_pos[0] + curr_vel[0]*t_local + ax*t_local**2/2)
                p[1].append(curr_pos[1] + curr_vel[1]*t_local + ay*t_local**2/2)
            a[0].append(0)
            a[1].append(0)

        else:
            print('Warning: able to reach max velocity')
            # accelerate
            t1 = v_max/a_max
            t.append(0.)
            v[0].append(0.)
            v[1].append(0.)
            p[0].append(curr_pos[0])
            p[1].append(curr_pos[1])
            for i in range(N):
                t_local = (i+1)*t1/N
                t.append(t_local)
                a[0].append(ax)
                a[1].append(ay)
                v[0].append(ax*t_local)
                v[1].append(ay*t_local)
                p[1].append(curr_pos[1] + ay*t_local**2/2)
                p[0].append(curr_pos[0] + ax*t_local**2/2)
            a[0].append(0)
            a[1].append(0)

            curr_pos = [p[0][-1], p[1][-1]]
            curr_vel = [v[0][-1], v[1][-1]]

            # coast
            ax_coast = 0
            ay_coast = 0
            t.append(t1)
            a[0].append(ax_coast)
            a[1].append(ay_coast)
            v[0].append(curr_vel[0])
            v[1].append(curr_vel[1])
            p[0].append(curr_pos[0])
            p[1].append(curr_pos[1])

            t2 = (dist_to_cover[bd]-2*dist_to_max)/v_max
            t.append(t1 + t2)
            a[0].append(0)
            a[1].append(0)
            v[0].append(curr_vel[0])
            v[1].append(curr_vel[1])
            p[0].append(curr_pos[0] + curr_vel[0]*t2)
            p[1].append(curr_pos[1] + curr_vel[1]*t2)
            
            curr_pos = [p[0][-1], p[1][-1]]

            # brake       
            ax = -ax
            ay = -ay
            t.append(t1 + t2)
            v[0].append(curr_vel[0])
            v[1].append(curr_vel[1])
            p[0].append(curr_pos[0])
            p[1].append(curr_pos[1])
            for i in range(N):
                t_local = (i+1)*t1/N
                t.append(t1 + t2 + t_local)
                a[0].append(ax)
                a[1].append(ay)
                v[0].append(curr_vel[0] + ax*t_local)
                v[1].append(curr_vel[1] + ay*t_local)
                p[0].append(curr_pos[0] + curr_vel[0]*t_local + ax*t_local**2/2)
                p[1].append(curr_pos[1] + curr_vel[1]*t_local + ay*t_local**2/2)
            a[0].append(0)
            a[1].append(0)

        t = [t0 + t_local for t_local in t]
        return p, v, a, t

class G02(GCodeBlock):
    def __init__(self, command, number, prev_block):
        GCodeBlock.__init__(self, command, number, prev_block)
        self.type = 'G02'

        self.plane = 'XY'  # in which plane is the arc formed

        self.center = [0, 0, 0]  # initialize
        if 'I' in command:
            self.I = command['I']
            self.center[0] = self.X0+self.I
        if 'J' in command:
            self.J = command['J']
            self.center[1] = self.Y0+self.J
        if 'K' in command:
            self.K = command['K']
            self.center[2] = self.Z0+self.K
        self.radius = distance_between(self.center, [self.X0, self.Y0,self.Z0])

        # Todo: add check if IJ, IK, or JK present, if not throw error

    def get_coordinates(self):

        start = [self.X0, self.Y0, self.Z0]
        end = [self.X1, self.Y1, self.Z1]

        value = np.arctan2(end[1],end[0]) - np.arctan2(start[1],start[0])
        arc_angle = value
        # value = (2*self.radius**2 -distance_between(start,end)**2)/(2*self.radius**2)
        # if np.abs(value) > 1:
        #     # may be due to floating point operations, but arccos only works in [-1,1]
        #     if np.abs(value) - 1 <= 1e-2:
        #         # manual rounding
        #         value = 1*np.sign(value)
        #     else:
        #         raise ValueError('Arccos was significantly outside range [-1,1], something is wrong')
        # arc_angle = np.arccos(value)

        coords = []

        if self.plane == 'YZ':
            # arc in YZ-plane

            # compute angle of both vectors with horizontal axes
            angle1 = np.arctan2(self.Z0-self.center[2], self.Y0-self.center[1])
            angle2 = np.arctan2(self.Z1-self.center[2], self.Y1-self.center[1])

            if angle1 < angle2:
                # clockwise so angle2 must be < angle1
                # probably angle2 is smaller, but arctan2 returned a negative angle
                angle1 += 2*np.pi
            arc_angle = angle1 - angle2
            self.arc_angle = arc_angle

            # if angle1 < angle2 and angle2 != np.pi:
            #     arc_angle += np.pi

            start_angle = angle1
            end_angle = start_angle - arc_angle  # clockwise
            angles = np.linspace(start_angle,end_angle,20)
            self.start_angle = start_angle
            self.end_angle = end_angle
            for s in angles:
                coords.append([self.X0, self.center[1]+self.radius*np.cos(s),self.center[2]+self.radius*np.sin(s)])
        elif self.plane == 'XZ':
            # arc in XZ-plane

            # compute angle of both vectors with horizontal axes
            angle1 = angle1
            angle2 = np.arctan2(self.Z1-self.center[2], self.X1-self.center[0])

            if angle1 < angle2:
                # clockwise so angle2 must be < angle1
                # probably angle2 is smaller, but arctan2 returned a negative angle
                angle1 += 2*np.pi
            arc_angle = angle1 - angle2
            self.arc_angle = arc_angle

            # if angle1 < angle2 and angle2 != np.pi:
            #     arc_angle += np.pi

            start_angle = np.arctan2(self.Z0-self.center[2],self.X0-self.center[0])
            end_angle = start_angle - arc_angle  # clockwise
            angles = np.linspace(start_angle,end_angle,20)
            self.start_angle = start_angle
            self.end_angle = end_angle

            for s in angles:
                coords.append([self.center[0]+self.radius*np.cos(s), self.Y0, self.center[2]+self.radius*np.sin(s)])
        elif self.plane == 'XY':
            # arc in XY-plane

            # compute angle of both vectors with horizontal axes
            angle1 = np.arctan2(self.Y0-self.center[1], self.X0-self.center[0])
            angle2 = np.arctan2(self.Y1-self.center[1], self.X1-self.center[0])

            if angle1 < angle2:
                # clockwise so angle2 must be < angle1
                # probably angle2 is smaller, but arctan2 returned a negative angle
                angle1 += 2*np.pi
            arc_angle = angle1 - angle2
            self.arc_angle = arc_angle

            # if angle1 < angle2 and angle2 != np.pi:
            #     arc_angle += np.pi

            start_angle = angle1
            end_angle = start_angle - arc_angle  # clockwise
            angles = np.linspace(start_angle,end_angle,20)
            self.start_angle = start_angle
            self.end_angle = end_angle

            for s in angles:
                coords.append([self.center[0]+self.radius*np.cos(s),self.center[1]+self.radius*np.sin(s), self.Z0])
        else:
            raise ValueError('Trying to plot an arc in 3D space, but only arcs in 2D subspace are supported')

        return coords
    
    def get_velocity_profile(self, t0, v_max, a_max):
        return G03.get_velocity_profile(self, t0, v_max, a_max, sign=-1)


class G03(GCodeBlock):
    def __init__(self, command, number, prev_block):
        GCodeBlock.__init__(self, command, number, prev_block)
        self.type = 'G03'

        self.plane = 'XY'  # in which plane is the arc formed

        self.center = [0, 0, 0]  # initialize
        if 'I' in command:
            self.I = command['I']
            self.center[0] = self.X0+self.I
        if 'J' in command:
            self.J = command['J']
            self.center[1] = self.Y0+self.J
        if 'K' in command:
            self.K = command['K']
            self.center[2] = self.Z0+self.K

        # Todo: add check if IJ, IK, or JK present, if not throw error

    def get_coordinates(self):

        start = [self.X0, self.Y0, self.Z0]
        end = [self.X1, self.Y1, self.Z1]

        value = np.arctan2(end[1],end[0]) - np.arctan2(start[1],start[0])
        arc_angle = value
        # value = (2*self.radius**2 -distance_between(start,end)**2)/(2*self.radius**2)
        # if np.abs(value) > 1:
        #     # may be due to floating point operations, but arccos only works in [-1,1]
        #     if np.abs(value) - 1 <= 1e-2:
        #         # manual rounding
        #         value = 1*np.sign(value)
        #     else:
        #         raise ValueError('Arccos was significantly outside range [-1,1], something is wrong')
        # arc_angle = np.arccos(value)

        coords = []

        if self.plane == 'YZ':
            # arc in YZ-plane

            self.radius = distance_between(self.center[1:], [self.Y0,self.Z0])

            # compute angle of both vectors with horizontal axes
            angle1 = np.arctan2(self.Z0-self.center[2], self.Y0-self.center[1])
            angle2 = np.arctan2(self.Z1-self.center[2], self.Y1-self.center[1])

            if angle1 > angle2:
                # counter-clockwise so angle2 must be > angle1
                # probably angle2 is bigger, but arctan2 returned a negative angle
                angle2 += 2*np.pi
            arc_angle = angle2 - angle1
            self.arc_angle = arc_angle

            # if angle1 > angle2 and angle1 != np.pi:
            #     arc_angle -= np.pi

            start_angle = angle1
            end_angle = start_angle + arc_angle  # counter-clockwise
            angles = np.linspace(start_angle,end_angle,20)
            self.start_angle = start_angle
            self.end_angle = end_angle

            for s in angles:
                coords.append([self.X0, self.center[1]+self.radius*np.cos(s),self.center[2]+self.radius*np.sin(s)])
        elif self.plane == 'XZ':
            # arc in XZ-plane

            self.radius = distance_between([self.center[0],self.center[2]], [self.X0,self.Z0])

            # compute angle of both vectors with horizontal axes
            angle1 = np.arctan2(self.Z0-self.center[2], self.X0-self.center[0])
            angle2 = np.arctan2(self.Z1-self.center[2], self.X1-self.center[0])

            if angle1 > angle2:
                # counter-clockwise so angle2 must be > angle1
                # probably angle2 is bigger, but arctan2 returned a negative angle
                angle2 += 2*np.pi
            arc_angle = angle2 - angle1
            self.arc_angle = arc_angle

            # if angle1 > angle2 and angle1 != np.pi:
            #     arc_angle -= np.pi

            start_angle = angle1
            end_angle = start_angle + arc_angle  # counter-clockwise
            angles = np.linspace(start_angle,end_angle,20)
            self.start_angle = start_angle
            self.end_angle = end_angle

            for s in angles:
                coords.append([self.center[0]+self.radius*np.cos(s), self.Y0, self.center[2]+self.radius*np.sin(s)])
        elif self.plane == 'XY':
            # arc in XY-plane

            self.radius = distance_between(self.center[:2], [self.X0,self.Y0])

            # compute angle of both vectors with horizontal axes
            angle1 = np.arctan2(self.Y0-self.center[1], self.X0-self.center[0])
            angle2 = np.arctan2(self.Y1-self.center[1], self.X1-self.center[0])

            if angle1 > angle2:
                # counter-clockwise so angle2 must be > angle1
                # probably angle2 is bigger, but arctan2 returned a negative angle
                angle2 += 2*np.pi
            arc_angle = angle2 - angle1
            self.arc_angle = arc_angle

            # if angle1 > angle2 and angle1 != np.pi:
            #     arc_angle -= np.pi

            start_angle = angle1
            end_angle = start_angle + arc_angle  # counter-clockwise
            angles = np.linspace(start_angle,end_angle,20)
            self.start_angle = start_angle
            self.end_angle = end_angle

            for s in angles:
                coords.append([self.center[0]+self.radius*np.cos(s),self.center[1]+self.radius*np.sin(s), self.Z0])
        else:
            raise ValueError('Trying to plot an arc in 3D space, but only arcs in 2D subspace are supported')

        return coords
    
    def get_velocity_profile(self, t0, v_max, a_max, sign=1):
        # compute velocity profile, linear
        radius = self.radius/1000
        center = [coord/1000 for coord in self.center]

        omega_v = v_max/radius
        omega_a = np.sqrt(a_max/radius)
        omega_max = omega_v if omega_v < omega_a else omega_a
        alpha_max = a_max/radius
        
        print('start_angle: ', self.start_angle)
        print('end_angle: ', self.end_angle)
        print('arc_angle: ', self.arc_angle)

        N = 10

        p = [[], []]
        v = [[], []]
        a = [[], []]
        t = []

        omega = []
        alpha = []
        theta = []

        # for i in range(N+1):
        #     t_local = i*T/N
        #     t.append(t0 + t_local)
        #     p[0].append(center[0]+radius*np.cos(angles[i]))
        #     p[1].append(center[1]+radius*np.sin(angles[i]))
        #     v[0].append(-radius*np.sin(angles[i])*omega_max)
        #     v[1].append(radius*np.cos(angles[i])*omega_max)
        #     a[0].append(-radius*np.cos(angles[i])*omega_max**2)
        #     a[1].append(-radius*np.sin(angles[i])*omega_max**2)

        t_acc = omega_max/alpha_max
        theta_acc = 0.5*alpha_max*t_acc**2
        theta_coast = self.arc_angle - 2*theta_acc
        t_coast = theta_coast/omega_max

        t.append(0)
        omega.append(0.)
        theta.append(self.start_angle)
        for i in range(N):
            dt_local = t_acc/N
            t_new = t[-1] + dt_local
            alpha_new = sign*alpha_max
            omega_new = omega[-1] + alpha_new*dt_local
            theta_new = theta[-1] + omega[-1]*dt_local + 0.5*alpha_new*dt_local**2
            t.append(t_new)
            alpha.append(alpha_new)
            omega.append(omega_new)
            theta.append(theta_new)
            # continue on this approach by checking if the acceleration is too high
            # if it is, scale the acceleration down
            # if it is not, continue
        alpha.append(0)        
 
        t.append(t_acc)
        omega.append(sign*omega_max)
        theta.append(self.start_angle + sign*theta_acc)
        for i in range(N):
            dt_local = t_coast/N
            t_new = t[-1] + dt_local
            alpha_new = 0.
            omega_new = omega[-1] + alpha_new*dt_local
            theta_new = theta[-1] + omega[-1]*dt_local + 0.5*alpha_new*dt_local**2
            t.append(t_new)
            alpha.append(alpha_new)
            omega.append(omega_new)
            theta.append(theta_new)
        alpha.append(0)     

        t.append(t_acc + t_coast)
        omega.append(sign*omega_max)
        theta.append(self.start_angle + sign*theta_acc + sign*theta_coast)
        for i in range(N):
            dt_local = t_acc/N
            t_new = t[-1] + dt_local
            alpha_new = -sign*alpha_max
            omega_new = omega[-1] + alpha_new*dt_local
            theta_new = theta[-1] + omega[-1]*dt_local + 0.5*alpha_new*dt_local**2
            t.append(t_new)
            alpha.append(alpha_new)
            omega.append(omega_new)
            theta.append(theta_new)
        alpha.append(0)

        positions = [[center[0] + radius*np.cos(thetas) for thetas in theta], 
                     [center[1] + radius*np.sin(thetas) for thetas in theta]]
        velocities = [[-radius*np.sin(theta)*omega for theta, omega in zip(theta, omega)],
                      [radius*np.cos(theta)*omega for theta, omega in zip(theta, omega)]]
        accelerations = [[-radius*np.cos(theta)*omega**2 - radius*np.sin(theta)*alpha for theta, omega, alpha in zip(theta, omega, alpha)],
                         [-radius*np.sin(theta)*omega**2 + radius*np.cos(theta)*alpha for theta, omega, alpha in zip(theta, omega, alpha)]]
        
        # tt = np.zeros_like(t)
        # dt = t[1] - t[0]
        # for i in range(len(t)-1):
        #     ax, ay = accelerations[0][i], accelerations[1][i]
        #     if abs(ax) > a_max or abs(ay) > a_max:
        #         axy = max(abs(ax), abs(ay))
        #         a_scaling = a_max/axy
        #     else:
        #         a_scaling = 1
        #     t_new = dt/a_scaling
        #     accelerations[0][i] *= a_scaling
        #     accelerations[1][i] *= a_scaling
           
        #     tt[i+1] = tt[i] + t_new
        # print(tt)

        t = [t0 + t_local for t_local in t]
        
        p[0] = positions[0]
        p[1] = positions[1]
        v[0] = velocities[0]
        v[1] = velocities[1]
        a[0] = accelerations[0]
        a[1] = accelerations[1]

        return p, v, a, t, alpha, omega, theta


def distance_between(point1, point2):
    # compute distance between points in nD space
    dist = 0
    for n in range (len(point1)):
        dist += (point2[n]-point1[n])**2
    return np.sqrt(dist)

def generate_gcodeblock(command, number, prev_block):
    # convert string to a dictionary representation
    command = command.split()  # split at white spaces
    command_dict = {}
    for c in command:
        if c in ['G00','G01','G02','G03']:
            command_dict['type'] = c
        elif '(' in c:
            # neglect this command, since it is a comment
            pass
        elif 'X' in c:
            command_dict['X'] = float(c[1:])
        elif 'Y' in c:
            command_dict['Y'] = float(c[1:])
        elif 'Z' in c:
            command_dict['Z'] = float(c[1:])
        elif 'I' in c:
            command_dict['I'] = float(c[1:])
        elif 'J' in c:
            command_dict['J'] = float(c[1:])
        elif 'K' in c:
            command_dict['K'] = float(c[1:])

    command = command_dict
    if 'type' in command:
        if command['type'] == 'G00':
            # sometimes also used as alternative for G01
            if prev_block is None:
                # if no prev_block, this is the first block,
                # use the X0,Y0,Z0 of this block as start position of the tool
                start_x = command['X'] if 'X' in command else 0.
                start_y = command['Y'] if 'Y' in command else 0.
                start_z = command['Z'] if 'Z' in command else 0.
                start = [start_x, start_y, start_z]
                block = G00(command, number, prev_block, start_pos=start)
            else:
                block = G00(command, number, prev_block)
        elif command['type'] == 'G01':
            block = G01(command, number, prev_block)
        elif command['type'] == 'G02':
            block = G02(command, number, prev_block)
        elif command['type'] == 'G03':
            block = G03(command, number, prev_block)
        else:
            warnings.warn('G-code given which was not yet implemented: ' + command['type'] + ', ignoring  this line.')
            return None
    else:
        # this line was not a G-code block
        return None
    #     raise RuntimeError('Invalid G-code block given: ', command)
    return block