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

    def read(self):
        self.subfiles = []
        file_str = self.file.readlines()
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
        for file in self.subfiles:
            file = self.filepath +'/subfiles/'+ file
            file_data = open(file, 'rb')
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
        self.read()
        self.create_blocks()
        self.plot_gcode()
        GCode = self.get_gcode()
        return GCode