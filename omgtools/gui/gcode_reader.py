import Tkinter as tk
import tkFileDialog as tkfiledialog
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from gcode_block import generate_gcodeblock

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
                except Exception, details:
                    tkmessagebox.showerror(('Error'),details)
                    return
        else:
            try:
                data = open(file, 'rb')
            except Exception, details:
                print details
                return
        self.file = data

    def convert(self):
        # shift midpoint and scale up or down a certain GCode file

        # ask the user if he wants to change the GCode file
        answer = ''
        while (not answer in ['yes', 'no']):
            answer = raw_input('Do you want to shift or scale the loaded GCode? (yes/no): ')

        if answer == 'yes':
            # shift and scale the GCode file
            lines = self.file.read().splitlines()

            # get user input for offset and scaling factor
            offset = []
            scaling = []
            while (not ((type(offset) == list) and (len(offset) == 2) and (all(isinstance(o, float) for o in offset)))
                    or not (isinstance(scaling, float))):
                offset = raw_input('What is the offset?  E.g. a,b to move midpoint from x,y to x-a, y-b: ').split(',')
                offset = [float(offset[0]), float(offset[1])]
                scaling = float(raw_input('What is the scaling factor? E.g. 2.1: '))

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
                # extract commands
                if line[0] == 'G' or (line[0] == 'N' and 'G' in line):
                    self.commands.append(line)

    def create_blocks(self):
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
        ax = fig.gca(projection='3d')
        ax.scatter(self.connections[:,0], self.connections[:,1], self.connections[:,2], color='red', marker='x', label='GCode')
        coordinates = []
        for block in self.blocks:
            coords = block.get_coordinates()
            for c in coords:
                coordinates.append(c)
        self.coords = np.array(coordinates)
        ax.plot(self.coords[:,0], self.coords[:,1], self.coords[:,2], color='blue', label='GCode')
        plt.show(block=False)

    def run(self):
        self.load_file()
        self.convert()
        self.read()
        self.create_blocks()
        self.plot_gcode()
        GCode = self.get_gcode()
        return GCode