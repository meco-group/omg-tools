#!/usr/bin/python

from ..environment import Environment
from ..basics.shape import Circle, Rectangle

import numpy as np
from xml.etree.ElementTree import ElementTree
import re
import sys, os
from matplotlib import pyplot as plt

class SVGReader(object):
    def __init__(self):
        # Load xml-tree
        self.ns = 'http://www.w3.org/2000/svg' # XML namespace
        self.et = ElementTree()
        self.svgpath = None

    def init(self, data):
        self.data = data
        self.tree = self.et.parse(data)
        if (self.tree.get('width')[-2:] == 'mm'):  # units millimeter
            viewbox = self.tree.get('viewBox').split(' ')
            xmin, ymin, xmax, ymax = viewbox
            self.width = float(xmax) - float(xmin)
            self.height = float(ymax) - float(ymin)
            self.meter_to_pixel = self.width/float(self.tree.get('width')[:-2])
        else:
            self.width = float(self.tree.get('width'))  # get width from svg 
            self.height = float(self.tree.get('height'))  # get height from svg
        self.obstacles = []

    def convert_path_to_points(self):
        
        # Find svg-paths, describing the shapes
        try:
            self.svgpath = self.tree.findall("{%s}path" %self.ns)  # Search for the word path in the outer branch of the SVG-file.
            if not self.svgpath:
                self.svgpath = self.tree.find("{%s}g" %self.ns).findall("{%s}path" %self.ns)  # If not yet found, search for the word path in the next branch
            if not self.svgpath:
                self.svgpath = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}path" %self.ns)  # If not yet found, search for the word path in the next branch
        except:  # error occured, e.g. no <g/> found, this is possible when you only have basic shapes
            print 'No shapes found which are described by a path, probably you only have basic shapes'
            return
        if not self.svgpath:  # no path found, this is possible when you only have basic shapes
            print 'No shapes found which are described by a path, probably you only have basic shapes'
            return

        self.n_paths = len(self.svgpath)  # number of paths which build up the figure

        #Initialize output file
        counter = 0
        # Loop over paths
        while counter < self.n_paths:
            lines = re.findall('[MCmc][\s.,0-9-]+', self.svgpath[counter].get('d')) # look for all MCmc with a number behind it, line runs until a space or minus sign is found
            points = []
            for line in lines:
                if line:
                    # line [0] contains Mx,y, the startpoint
                    test1=line[1:].replace(","," ")  # replace comma by: space 
                    test2 = test1.replace("-"," -")  # replace minus sign by: space minus 
                    test3 = test2.replace("c"," c ")  # replace c by: space c space
                    newpoints = np.array(map(eval, test3.strip().split(' ')))  # splits the line at each space, to create separate points
                    if line[0] == 'c':  # lower case c means relative coordinates, upper case C is absolute coordinates
                        newpoints[0:6:2] = newpoints[0:6:2] + points[-2]  # relative to absolute coordinates for x
                        newpoints[1:6:2] = newpoints[1:6:2] + points[-1]  # relative to absolute coordinates for y
                    # for the first line (Mx,y) there is no 'c', so the starting point (x,y) is added to points in the first iteration
                    points.extend(newpoints)  # add newpoints to points
            counter += 1
            # Save points to file
            f = open("environment.txt", "a")
            f.write( "path_"+ str(counter) + "="+ str(np.array(points)) + "\n" )
            f.close() 

    def convert_basic_shapes(self):
        
        # Code for basic shapes <circle> and <rect>

        # Find svg-paths, describing the shapes
        self.rectangles = self.tree.findall("{%s}rect" %self.ns)  # Search for the word rect in the outer branch of the SVG-file.
        if not self.rectangles:
            self.rectangles = self.tree.find("{%s}g" %self.ns).findall("{%s}rect" %self.ns)  # If not yet found, search for the word rect in the next branch
        elif not self.rectangles:
            self.rectangles = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}rect" %self.ns)  # If not yet found, search for the word rect in the next branch
        self.n_rect = len(self.rectangles)  # number of paths which build up the figure
        if self.n_rect == 0:
            print 'No rectangles found'

        # Find svg-paths, describing the shapes
        self.circles = self.tree.findall("{%s}circle" %self.ns)  # Search for the word circ in the outer branch of the SVG-file.
        if not self.circles:
            self.circles = self.tree.find("{%s}g" %self.ns).findall("{%s}circle" %self.ns)  # If not yet found, search for the word circ in the next branch
        elif not self.circles:
            self.circles = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}circle" %self.ns)  # If not yet found, search for the word circ in the next branch
        self.n_circ = len(self.circles)  # number of paths which build up the figure
        if self.n_circ == 0:
            print 'No circles found'

        for rectangle in self.rectangles:
            obstacle = {}
            obstacle['shape'] = 'rectangle'
            pos = [float(rectangle.get('x')), float(rectangle.get('y'))]  # Note: [x,y] is the top left corner
            # axis are placed in the top left corner and point to the right(x) and downward(y)
            obstacle['pos'] = [pos[0]+float(rectangle.get('width'))*0.5, pos[1]+float(rectangle.get('height'))*0.5]
            obstacle['width'] = float(rectangle.get('width'))
            obstacle['height'] = float(rectangle.get('height'))
            obstacle['velocity'] = [0, 0]
            obstacle['bounce'] = False
            self.obstacles.append(obstacle)

        for circle in self.circles:
            obstacle = {}
            obstacle['shape'] = 'circle'
            obstacle['pos'] = [float(circle.get('cx')), float(circle.get('cy'))]  # Note: [x,y] is the top left corner
            obstacle['radius'] = float(circle.get('r'))
            obstacle['velocity'] = [0, 0]
            obstacle['bounce'] = False
            self.obstacles.append(obstacle)

        # Code for Bezier expression
        # how represent a circle in bezier?
        # for rectangle check on straight lines --> is control point on line between start and end?

    # def save_environment(self):
    #     environment = {}
    #     environment['position'] = [0,0]
    #     environment['position'][0] = self.position[0]/self.meter_to_pixel
    #     environment['position'][1] = self.position[1]/self.meter_to_pixel
    #     environment['width'] = self.width/self.meter_to_pixel
    #     environment['height'] = self.height/self.meter_to_pixel
    #     env_to_save = [environment, self.obstacles]
    #     with open('environment_svg.pickle', 'wb') as handle:
    #         pickle.dump(env_to_save, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def reconstruct(self, file):
        points = []
        with open(file, "r") as f:
            for line in f:
                for word in line.split(' '):
                    if (word != ', ' and word != ',  ' and word != '' and word != ' '):
                        points.append(word)
        f.close() 
        newpoints = []
        for point in points:
            if point[-1:] == '\n':
                point = point[:-1]
            if point[-1:] == ']':
                point = point[:-1]
            if point[0] != 'p':
                newpoints.append(point)
        x = []
        y = []
        for i in range(0,len(newpoints),2):
            x.append(newpoints[i])
            y.append(newpoints[i+1])
        
        plt.plot(x,y)
        plt.show()

    def build_environment(self):

        self.convert_basic_shapes()  # gives self.rects and self.circs 
        self.convert_path_to_points()  # completes self.rects and self.circs with shapes defined by path
        # if you found some paths, call function to transform them to an obstacle
        # e.g. rectangle or circle and place add it to self.obstacles

        # now self.obstacles is filled

# if __name__ == '__main__':

#     args = sys.argv
#     args[0] = args[0].replace("/",os.sep)
#     data = args[1]

#     reader = SVGReader()
#     reader.init(data)
#     # reader.reconstruct(data)
#     # reader.convert_path_to_points(data)
#     reader.build_environment()