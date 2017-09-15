import numpy as np

import warnings

class GCodeBlock(object):

    def __init__(self, command, number, prev_block=None):

        self.default_F = 444  # default feedrate [mm/min]
        self.default_S = 30000  # default rotation velocity [rev/min]

        if prev_block is not None:
            self.X0 = prev_block.X1
            self.Y0 = prev_block.Y1
            self.Z0 = prev_block.Z1
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
    def __init__(self, command, number, prev_block):
        GCodeBlock.__init__(self, command, number, prev_block)
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
        return [start, end]

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

            # if angle1 < angle2 and angle2 != np.pi:
            #     arc_angle += np.pi

            start_angle = angle1
            end_angle = start_angle - arc_angle  # clockwise
            angles = np.linspace(start_angle,end_angle,20)
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

            # if angle1 < angle2 and angle2 != np.pi:
            #     arc_angle += np.pi

            start_angle = np.arctan2(self.Z0-self.center[2],self.X0-self.center[0])
            end_angle = start_angle - arc_angle  # clockwise
            angles = np.linspace(start_angle,end_angle,20)

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

            # if angle1 < angle2 and angle2 != np.pi:
            #     arc_angle += np.pi

            start_angle = angle1
            end_angle = start_angle - arc_angle  # clockwise
            angles = np.linspace(start_angle,end_angle,20)

            for s in angles:
                coords.append([self.center[0]+self.radius*np.cos(s),self.center[1]+self.radius*np.sin(s), self.Z0])
        else:
            raise ValueError('Trying to plot an arc in 3D space, but only arcs in 2D subspace are supported')

        return coords

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

            if angle1 > angle2:
                # counter-clockwise so angle2 must be > angle1
                # probably angle2 is bigger, but arctan2 returned a negative angle
                angle2 += 2*np.pi
            arc_angle = angle2 - angle1

            # if angle1 > angle2 and angle1 != np.pi:
            #     arc_angle -= np.pi

            start_angle = angle1
            end_angle = start_angle + arc_angle  # counter-clockwise
            angles = np.linspace(start_angle,end_angle,20)
            for s in angles:
                coords.append([self.X0, self.center[1]+self.radius*np.cos(s),self.center[2]+self.radius*np.sin(s)])
        elif self.plane == 'XZ':
            # arc in XZ-plane

            # compute angle of both vectors with horizontal axes
            angle1 = np.arctan2(self.Z0-self.center[2], self.X0-self.center[0])
            angle2 = np.arctan2(self.Z1-self.center[2], self.X1-self.center[0])

            if angle1 > angle2:
                # counter-clockwise so angle2 must be > angle1
                # probably angle2 is bigger, but arctan2 returned a negative angle
                angle2 += 2*np.pi
            arc_angle = angle2 - angle1

            # if angle1 > angle2 and angle1 != np.pi:
            #     arc_angle -= np.pi

            start_angle = angle1
            end_angle = start_angle + arc_angle  # counter-clockwise
            angles = np.linspace(start_angle,end_angle,20)

            for s in angles:
                coords.append([self.center[0]+self.radius*np.cos(s), self.Y0, self.center[2]+self.radius*np.sin(s)])
        elif self.plane == 'XY':
            # arc in XY-plane

            # compute angle of both vectors with horizontal axes
            angle1 = np.arctan2(self.Y0-self.center[1], self.X0-self.center[0])
            angle2 = np.arctan2(self.Y1-self.center[1], self.X1-self.center[0])

            if angle1 > angle2:
                # counter-clockwise so angle2 must be > angle1
                # probably angle2 is bigger, but arctan2 returned a negative angle
                angle2 += 2*np.pi
            arc_angle = angle2 - angle1

            # if angle1 > angle2 and angle1 != np.pi:
            #     arc_angle -= np.pi

            start_angle = angle1
            end_angle = start_angle + arc_angle  # counter-clockwise
            angles = np.linspace(start_angle,end_angle,20)

            for s in angles:
                coords.append([self.center[0]+self.radius*np.cos(s),self.center[1]+self.radius*np.sin(s), self.Z0])
        else:
            raise ValueError('Trying to plot an arc in 3D space, but only arcs in 2D subspace are supported')

        return coords

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