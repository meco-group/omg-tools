try:
  import tkinter as tk
  import tkinter.filedialog as tkfiledialog
except ImportError:
  import Tkinter as tk
  import tkFileDialog as tkfiledialog
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from .gcode_block import generate_gcodeblock

try:
  input = raw_input
except NameError:
  pass

class GCodeReader(object):
    def __init__(self):
        self.root = tk.Tk();
        self.root.withdraw()

        self.GCodes = ['G00', 'G01', 'G02', 'G03']  # list of supported GCode commands

        self.commands = []
        self.blocks = []

    def reset(self):
        self.commands = []
        self.blocks = []

    def load_file(self, file=None):

        if file is None:
            file = tkfiledialog.askopenfilename(filetypes=[('GCode files', '.nc'), ('all files','.*')])
            self.filepath = file.rsplit('/',1)[0]  # save file path without the file name
            if file:
                try:
                    data = open(file, 'rb')
                except Exception as details:
                    tkmessagebox.showerror(('Error'),details)
                    return
        else:
            try:
                data = open(file, 'rb')
            except Exception as details:
                print(details)
                return
        self.file = data

    def convert(self):
        # shift midpoint and scale up or down a certain GCode file

        # ask the user if he wants to change the GCode file
        answer = 'no'
        # while (not answer in ['yes', 'no']):
        #     answer = input('Do you want to shift or scale the loaded GCode? (yes/no): ')

        if answer == 'yes':
            # shift and scale the GCode file
            lines = self.file.read().splitlines()

            # get user input for offset and scaling factor
            offset = []
            scaling = []
            while (not ((type(offset) == list) and (len(offset) == 2) and (all(isinstance(o, float) for o in offset)))
                    or not (isinstance(scaling, float))):
                offset = input('What is the offset?  E.g. a,b to move midpoint from x,y to x-a, y-b: ').split(',')
                offset = [float(offset[0]), float(offset[1])]
                scaling = float(input('What is the scaling factor? E.g. 2.1: '))

            # apply offset
            offset_lines = []
            for line in lines:
                if any(type in line for type in ['G00', 'G01', 'G02', 'G03']):
                    # only look at lines containing a GCode command
                    split_line = line.split()
                    for idx, s in enumerate(split_line):
                        if 'X' in s:
                            x_val = float(s[1:])
                            x_val_n = x_val-offset[0]
                            split_line[idx] = 'X' + str(x_val_n)
                        if 'Y' in s:
                            y_val = float(s[1:])
                            y_val_n = y_val-offset[1]
                            split_line[idx] = 'Y' + str(y_val_n)
                    offset_lines.append(' '.join(split_line))

            # apply scaling
            new_lines = []
            for line in offset_lines:
                if any(type in line for type in ['G00', 'G01', 'G02', 'G03']):
                    # only look at lines containing a GCode command
                    split_line = line.split()
                    for idx, s in enumerate(split_line):
                        if 'X' in s:
                            x_val = float(s[1:])
                            x_val_n = x_val*scaling
                            split_line[idx] = 'X' + str(x_val_n)
                        if 'Y' in s:
                            y_val = float(s[1:])
                            y_val_n = y_val*scaling
                            split_line[idx] = 'Y' + str(y_val_n)
                        if 'I' in s:
                            i_val = float(s[1:])
                            i_val_n = i_val*scaling
                            split_line[idx] = 'I' + str(i_val_n)
                        if 'J' in s:
                            j_val = float(s[1:])
                            j_val_n = j_val*scaling
                            split_line[idx] = 'J' + str(j_val_n)
                    new_lines.append(' '.join(split_line))

            # get file name and remove extension '.nc'
            old_name = self.file.name.split('/')[-1][:-3]
            file = open(old_name + '_shift_scale.nc', 'w')
            for line in new_lines:
                file.write(line+'\n')
            file.close()
            file = open(old_name+'_shift_scale.nc', 'rb')
            self.file = file
        # else: # do nothing

    def read(self, file=None):
        self.subfiles = []
        if file is None:
            file_str = self.file.readlines()
        else:
            file_str = file.readlines()
        for line in file_str:
            line = line.strip().decode("utf-8")
            # extract commands
            if line[0] == '(':
                # this is a commented line
                pass
            elif line[0] == 'G' or (line[0] == 'N' and (any([code in line for code in self.GCodes]))):
                self.commands.append(line)
            # extract subfiles: sometimes the self.file .nc file contains references to extra .nc files
            elif (line[0] == 'N' and ' L ' in line):
                # this is a line specifying a subfile
                info = line.split()
                for i in info:
                    if '.nc' in i:
                        filename = i
                        self.subfiles.append(filename)
            # else: skip line, this is not a G-code block

        # loop over all subfiles
        for f in self.subfiles:
            f = self.filepath +'/subfiles/'+ f
            file_data = open(f, 'rb')
            file_str = file_data.readlines()
            for line in file_str:
                line = line.strip().decode("utf-8")
                # extract commands
                if line[0] == 'G' or (line[0] == 'N' and 'G' in line):
                    self.commands.append(line)

    def create_blocks(self):
        # represents all GCode commands as corresponding GCode block-object
        self.cnt = 0  # reset counter
        if not self.blocks:
            # first block, so add empty block
            prev_block = None
        else:
            prev_block = self.blocks[-1]
        for command in self.commands:
            self.cnt += 1
            new_block = generate_gcodeblock(command, self.cnt, prev_block)
            if (new_block is not None and not
               (new_block.type in ['G00', 'G01'] and
                new_block.X0 == new_block.X1 and
                new_block.Y0 == new_block.Y1 and
                new_block.Z0 == new_block.Z1)):
                # there was a new block and there was a movement compared
                # to the last block

                # new previous block = last one added
                self.blocks.append(new_block)
                prev_block = new_block
            elif new_block is not None:
                prev_block = new_block
            # else:  # new block was not valid, don't add it

    def get_gcode(self):
        return self.blocks

    def get_block_division(self, GCode):
        # divide the provided GCode in different collections of blocks:
        # Z-axis movements and XY-plane movements
        # this function can handle two possible situations:
        #   1) machining several layers, with a z-movement at the end of each layer
        #   2) retracting the tool at some points to e.g. machine a sequence of circles
        # for both cases the complete GCode is split in different parts
        # for 1)
        #   1. machining
        #   2. move z-axis deeper
        # for 2)
        #   1. machining
        #   2. z-axis retraction (may be done fast)
        #   3. movement without machining (may be done fast)
        #   4. z-axis approach (must be done slowly)

        GCode_blocks = []  # contains the different GCode parts in a list of lists
        blocks = []  # local variable to save the different GCode parts temporarily
        for block in GCode:
            if block.type not in ['G00', 'G01'] or (block.Z0 == block.Z1):
                # arc segment ('G02', 'G03'), or no movement in z-direction
                blocks.append(block)
            else:
                # this is a z-retract/engage block
                # save the previous blocks
                if blocks:
                    GCode_blocks.append(blocks)
                # clear blocks variable
                blocks = []
                # add z-block separately
                GCode_blocks.append([block])
        GCode_blocks.append(blocks)
        return GCode_blocks

    def get_connections(self):
        # returns a list of points in which the different GCode trajectories are connected
        connections = []
        for block in self.blocks:
            connections.append([block.X0,block.Y0, block.Z0])
        connections.append([self.blocks[-1].X0, self.blocks[-1].Y0, self.blocks[-1].Z0])  # add last point
        connections.append([self.blocks[0].X0, self.blocks[0].Y0, self.blocks[0].Z0])  # close contour
        self.connections = np.array(connections)

    def plot_gcode(self):
        # plot the workpiece that is represented by the GCode

        self.get_connections()

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(self.connections[:,0], self.connections[:,1], self.connections[:,2], color='red', marker='x', label='GCode')
        coordinates = []
        t0 = 0
        p = [[], []]
        v = [[], []]
        a = [[], []]
        t = []
        for block in self.blocks:
            coords = block.get_coordinates()
            print(block.type)
            if block.type == 'G01':
                pb, vb, ab, tb = block.get_velocity_profile(t0, 0.1, .5)
            elif block.type == 'G02':
                pb, vb, ab, tb = block.get_velocity_profile(t0, 0.1, .5)
            elif block.type == 'G03':
                pb, vb, ab, tb = block.get_velocity_profile(t0, 0.1, .5)
            p[0].append(pb[0])
            p[1].append(pb[1])
            v[0].append(vb[0])
            v[1].append(vb[1])
            a[0].append(ab[0])
            a[1].append(ab[1])
            t.append(tb)
            t0 = tb[-1]

            for c in coords:
                coordinates.append(c)
        self.coords = np.array(coordinates)
        ax.plot(self.coords[:,0], self.coords[:,1], self.coords[:,2], color='blue', label='GCode')
        # plt.show(block=True)

        _, (ax2_1, ax2_2) = plt.subplots(2, 1)
        ax2_1.plot(self.coords[:,0])
        ax2_2.plot(self.coords[:,1])

        _, (ax2_1, ax2_2) = plt.subplots(2, 1)
        ax2_1.plot(self.coords[2:,0] - self.coords[:-2,0])
        ax2_2.plot(self.coords[2:,1] - self.coords[:-2,1])

        _, (ax2_1, ax2_2) = plt.subplots(2, 1)
        ax2_1.plot(self.coords[2:,0] - 2*self.coords[1:-1,0] + self.coords[:-2,0])
        ax2_2.plot(self.coords[2:,1] - 2*self.coords[1:-1,1] + self.coords[:-2,1])

        _, (ax3_1, ax3_2) = plt.subplots(2, 1)
        pxx = []
        pyy = []
        ttt = []
        for tt, px, py in zip(t, p[0], p[1]):
            ax3_1.plot(tt, px)
            ax3_2.plot(tt, py)
            pxx.extend([1000*ppx for ppx in px])
            pyy.extend([1000*ppy for ppy in py])
            ttt.extend(tt)

        _, (ax4_1, ax4_2, ax4_3) = plt.subplots(3, 1)
        vxx = []
        vyy = []
        ttt = []
        for tt, vx, vy in zip(t, v[0], v[1]):
            ax4_1.plot(tt, vx)
            ax4_2.plot(tt, vy)
            ax4_3.plot(tt, [np.sqrt(vxx**2 + vyy**2) for vxx, vyy in zip(vx, vy)])
            vxx.extend(vx)
            vyy.extend(vy)
            ttt.extend(tt)
            
        _, (ax5_1, ax5_2, ax5_3) = plt.subplots(3, 1)
        axx = []
        ayy = []
        ttt = []
        for tt, ax, ay in zip(t, a[0], a[1]):
            ax5_1.step(tt, ax, where='post')
            ax5_2.step(tt, ay, where='post')
            ax5_3.step(tt, [np.sqrt(axx**2 + ayy**2) for axx, ayy in zip(ax, ay)], where='post')
            axx.extend(ax)
            ayy.extend(ay)
            ttt.extend(tt)

        _, ax5_1 = plt.subplots(1, 1)
        ax5_1.plot(ttt, axx)
        a_v = [(vxx1-vxx2)/(tx1-tx2) for vxx1, vxx2, tx1, tx2 in zip(vxx[1:], vxx[:-1], ttt[1:], ttt[:-1])]
        ax5_1.plot(ttt[1:], a_v)

        _, (ax1_1) = plt.subplots(1, 1)
        ax1_1.plot(self.coords[:,0], self.coords[:,1], color='blue')
        ax1_1.plot(pxx, pyy, color='red')
        ax1_1.set_xlabel('X [mm]')
        ax1_1.set_ylabel('Y [mm]')
        ax1_1.set_aspect('equal')

        plt.show(block=True)


    def run(self):
        self.load_file()
        self.convert()
        self.read()
        self.create_blocks()
        self.plot_gcode()
        GCode = self.get_gcode()
        return GCode