import numpy as np
from xml.etree.ElementTree import ElementTree
import re
from matplotlib import pyplot as plt

class SVGReader(object):
    def __init__(self):
        # load XML-tree
        self.ns = 'http://www.w3.org/2000/svg' # XML namespace
        self.et = ElementTree()
        self.svgpath = None

    def init(self, data):
        self.data = data
        self.tree = self.et.parse(data)
        if (self.tree.get('width')[-2:] in ['mm', 'px']):  # units millimeter or pixels
            viewbox = self.tree.get('viewBox').split(' ')
            xmin, ymin, xmax, ymax = viewbox
            if (self.tree.get('width')[-2:] is 'mm'):
                self.width_px = float(xmax) - float(xmin)
                self.height_px = float(ymax) - float(ymin)
                self.meter_to_pixel = self.width_px/float(self.tree.get('width')[:-2])
            else: # [px]
                self.width_px = float(xmax) - float(xmin)
                self.height_px = float(ymax) - float(ymin)
        else:
            # if no unit mentioned, it is px
            self.width_px = float(self.tree.get('width'))  # get width from svg
            self.height_px = float(self.tree.get('height'))  # get height from svg

        self.position = [0, 0]  # default, [px]
        self.obstacles = []

    def convert_path_to_points(self):

        # find svg-paths, describing the shapes e.g. using Bezier curves
        # for rectangle check on straight lines --> is control point on line
        # between start and end?
        try:
            # search for the word path in the outer branch of the SVG-file
            self.svgpath = self.tree.findall("{%s}path" %self.ns)
            if not self.svgpath:
                # if not yet found, search for the word path in the next branch
                self.svgpath = self.tree.find("{%s}g" %self.ns).findall("{%s}path" %self.ns)
            if not self.svgpath:
                # if not yet found, search for the word path in the next branch
                self.svgpath = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}path" %self.ns)
        except:  # error occured, e.g. no <g/> found, this is possible when you only have basic shapes
            print 'No shapes found which are described by a path, probably you only have basic shapes'
            return
        if not self.svgpath:  # no path found, this is possible when you only have basic shapes
            print 'No shapes found which are described by a path, probably you only have basic shapes'
            return

        self.n_paths = len(self.svgpath)  # number of paths which build up the figure

        # initialize output file
        counter = 0
        # loop over paths
        while counter < self.n_paths:
            # look for all MCmc with a number behind it, line runs until a space or minus sign is found
            lines = re.findall('[MCmc][\s.,0-9-]+', self.svgpath[counter].get('d'))
            points = []
            for line in lines:
                if line:
                    # line [0] contains Mx,y, the startpoint
                    test1=line[1:].replace(","," ")  # replace comma by: space
                    test2 = test1.replace("-"," -")  # replace minus sign by: space minus
                    test3 = test2.replace("c"," c ")  # replace c by: space c space
                    # splits the line at each space, to create separate points
                    newpoints = np.array(map(eval, test3.strip().split(' ')))
                    if line[0] == 'c':  # lower case c means relative coordinates, upper case C is absolute coordinates
                        newpoints[0:6:2] = newpoints[0:6:2] + points[-2]  # relative to absolute coordinates for x
                        newpoints[1:6:2] = newpoints[1:6:2] + points[-1]  # relative to absolute coordinates for y
                    # for the first line (Mx,y) there is no 'c', so the starting point (x,y)
                    # is added to points in the first iteration
                    points.extend(newpoints)  # add newpoints to points
            counter += 1
            # save points to file
            f = open("environment.txt", "a")
            f.write( "path_"+ str(counter) + "="+ str(np.array(points)) + "\n" )
            f.close()

    def convert_basic_shapes(self):
        # code for basic shapes <circle> and <rect>

        # Todo: these shapes can also have a transform, find it and
        # add this transform to self.transform

        # find svg-paths, describing the rectangles
        try:
            # search for the word rect in the outer branch of the SVG-file
            self.rectangles = self.tree.findall("{%s}rect" %self.ns)
            if not self.rectangles:
                # if not yet found, search for the word rect in the next branch
                self.rectangles = self.tree.find("{%s}g" %self.ns).findall("{%s}rect" %self.ns)
            if not self.rectangles:
                # if not yet found, search for the word rect in the next branch
                self.rectangles = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}rect" %self.ns)
            self.n_rect = len(self.rectangles)  # number of paths which build up the figure
            if self.n_rect == 0:
                print 'No rectangles found'
        except:
            print 'No shapes found which are described by a rect'

        # find svg-paths, describing the circles
        try:
            # search for the word circ in the outer branch of the SVG-file
            self.circles = self.tree.findall("{%s}circle" %self.ns)
            if not self.circles:
                # if not yet found, search for the word circ in the next branch
                self.circles = self.tree.find("{%s}g" %self.ns).findall("{%s}circle" %self.ns)
            if not self.circles:
                # if not yet found, search for the word circ in the next branch
                self.circles = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}circle" %self.ns)
            self.n_circ = len(self.circles)  # number of paths which build up the figure
            if self.n_circ == 0:
                print 'No circles found'
        except:
            print 'No shapes found which are described by a circle'

        for rectangle in self.rectangles:
            obstacle = {}
            obstacle['shape'] = 'rectangle'
            pos = [float(rectangle.get('x')), float(rectangle.get('y'))]  # Note: [x,y] is the top left corner
            # axis are placed in the top left corner and point to the right(x) and downward(y)
            obstacle['pos'] = [pos[0]+float(rectangle.get('width'))*0.5, pos[1]+float(rectangle.get('height'))*0.5]
            obstacle['pos'] += self.transform  # apply transform
            obstacle['width'] = float(rectangle.get('width'))
            obstacle['height'] = float(rectangle.get('height'))
            obstacle['velocity'] = [0, 0]
            obstacle['bounce'] = False
            self.obstacles.append(obstacle)

        for circle in self.circles:
            obstacle = {}
            obstacle['shape'] = 'circle'
            obstacle['pos'] = [float(circle.get('cx')), float(circle.get('cy'))]  # Note: [x,y] is the top left corner
            obstacle['pos'] += self.transform  # apply transform
            obstacle['radius'] = float(circle.get('r'))
            obstacle['velocity'] = [0, 0]
            obstacle['bounce'] = False
            self.obstacles.append(obstacle)

    def convert_lines(self):
        # code for basic shapes <line> and <polyline>

        # find svg-polylines
        # example: <polyline fill="none" stroke="#333333" stroke-width="10" points="25.41,40.983 25.41,258.197 414.754,258.197 "/>
        try:
            # search for the word polyline in the outer branch of the SVG-file
            self.polylines = self.tree.findall("{%s}polyline" %self.ns)
            if not self.polylines:
                # if not yet found, search for the word polyline in the next branch
                self.polylines = self.tree.find("{%s}g" %self.ns).findall("{%s}polyline" %self.ns)
            if not self.polylines:
                # if not yet found, search for the word polyline in the next branch
                self.polylines = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}polyline" %self.ns)
            self.n_polylines = len(self.polylines)  # number of paths which build up the figure
            if self.n_polylines == 0:
                print 'No polylines found'
        except:
            print 'No shapes found which are described by a polyline'

        # find svg-lines
        # example: <line fill="none" stroke="#333333" stroke-width="10" x1="25.41" y1="174.59" x2="69.672" y2="174.59"/>
        try:
            # search for the word line in the outer branch of the SVG-file
            self.lines = self.tree.findall("{%s}line" %self.ns)
            if not self.lines:
                # if not yet found, search for the word line in the next branch
                self.lines = self.tree.find("{%s}g" %self.ns).findall("{%s}line" %self.ns)
            if not self.lines:
                # if not yet found, search for the word line in the next branch
                self.lines = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).findall("{%s}line" %self.ns)
            self.n_lines = len(self.lines)  # number of paths which build up the figure
            if self.n_lines == 0:
                print 'No lines found'
        except:
            print 'No shapes found which are described by a line'

        for polyline in self.polylines:
            try:
                stroke_width = float(polyline.get('stroke-width'))  # stroke-width given as basic element
            except:  # stroke-width wrapped in style element
                style = polyline.get('style').split(';')
                for element in style:
                    if 'stroke-width' in element:
                        stroke_width = float(element.split(':')[1])
            vertices = polyline.get('points').split(' ')
            vertices[:] = (v for v in vertices if v != '')  # remove all empty strings
            vertices = np.array(map(eval, vertices))  # gives array of arrays [[x,y],[],...]
            vertices += self.transform

            # make rectangle of each vertex couple
            for l in range(len(vertices)-1):
                obstacle = {}
                obstacle['shape'] = 'rectangle'
                obstacle['velocity'] = [0, 0]
                obstacle['bounce'] = False

                # Note: to avoid explicitly checking if the line goes from
                # left to right / right to left
                # bottom to top / top to bottom
                # we use w and h separate from obstacle width and height
                line = np.array(vertices[l+1]) - np.array(vertices[l])
                if line[0] == 0:  # vertical line
                    obstacle['width'] =  stroke_width
                    obstacle['height'] = abs(line[1])
                    h = line[1]
                    w = cmp(h,0)*stroke_width  # give stroke_width same sign as h
                elif line[1] == 0:  # horizontal line
                    obstacle['width'] = abs(line[0])
                    obstacle['height'] = stroke_width
                    w = line[0]
                    h = cmp(w,0)*stroke_width  # give stroke_width same sign as w
                else:
                    raise RuntimeError('Diagonal lines are not yet supported')
                obstacle['pos'] = [vertices[l][0] + w*0.5, vertices[l][1] + h*0.5]
                self.obstacles.append(obstacle)

        for line in self.lines:
            obstacle = {}
            obstacle['shape'] = 'rectangle'
            obstacle['velocity'] = [0, 0]
            obstacle['bounce'] = False

            try:
                stroke_width = float(line.get('stroke-width'))  # stroke-width given as basic element
            except:  # stroke-width wrapped in style element
                style = line.get('style').split(';')
                for element in style:
                    if 'stroke-width' in element:
                        stroke_width = float(element.split(':')[1])
            x1, y1 = float(line.get('x1')), float(line.get('y1'))
            x2, y2 = float(line.get('x2')), float(line.get('y2'))
            # add transform
            x1 += self.transform[0]
            y1 += self.transform[1]
            x2 += self.transform[0]
            y2 += self.transform[1]
            if x1 == x2:  # vertical line
                obstacle['width'] =  stroke_width
                obstacle['height'] = abs(y2-y1)
                h = y2-y1  # signed value
                w = cmp(h,0)*stroke_width
            elif y1 == y2:  # horizontal line
                obstacle['width'] = abs(x2-x1)
                obstacle['height'] = stroke_width
                w = x2-x1  # signed value
                h = cmp(w,0)*stroke_width
            else:
                raise RuntimeError('Diagonal lines are not yet supported')

            # don't use width and height since then you have to check if x1 > x2 etc,
            # to decide if the line goes from left to right or the other way around
            obstacle['pos'] = [x1 + w*0.5, y1 + h*0.5]
            self.obstacles.append(obstacle)

    def compute_transform(self):
        # Note: only works for translation for the moment
        # check if figure is transformed, e.g. a translation
        try:
            trans1 = self.tree.find("{%s}g" %self.ns).get('transform')
            trans1 = trans1.split('translate')
            trans1.remove('')
            self.transform1 = np.array(map(eval, trans1)[0])
        except:
            print 'No transform1 found'
        try:
            trans2 = self.tree.find("{%s}g" %self.ns).find("{%s}g" %self.ns).get('transform')
            trans2 = trans2.split('translate')
            trans2.remove('')
            self.transform2 = np.array(map(eval, trans2)[0])
        except:
            print 'No transform2 found'
        if hasattr(self, 'transform1') and hasattr(self, 'transform2'):
            self.transform = self.transform1 + self.transform2  # coordinate frame transformation
        elif hasattr(self, 'transform1') :
            self.transform = self.transform1
        elif hasattr(self, 'transform2') :
            self.transform = self.transform2
        else:
            self.transform = [0, 0]  # no transforms found

    def reconstruct(self, file):
        # help function, re-draws the loaded figure, allowing to check if it has the desired shapes
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

        # Todo: write code here to check which elements are in the svg:
        # path, rectangle, circ, line, polyline,...
        # and call the appropriate functions, instead of calling them all

        self.compute_transform()  # assigns values to self.transform

        self.convert_basic_shapes()  # looks for rect and circle shapes
        self.convert_path_to_points()  # looks for shapes defined by a Bezier path
        self.convert_lines()  # looks for shapes defined by polyline and line
        # if you found some paths, they are transformed to an obstacle and
        # added to self.obstacles

    def get_gcode_description (self):
        # Note: for now this function only works for lines and paths. With capital M and lower-case c.
        # The paths are supposed to represent circle segments, if they don't,
        # a circle approximation of the curve is used.

        children = self.tree.getchildren()  # the xml-tree
        self.commands = []  # holds the GCode commands

        for idx, child in enumerate(children):
            if 'line' in child.tag:
                # child is a line
                # example: <line fill="none" stroke="#333333" stroke-width="10" x1="25.41" y1="174.59" x2="69.672" y2="174.59"/>
                x1, y1 = float(child.get('x1')), float(child.get('y1'))  # start
                x2, y2 = float(child.get('x2')), float(child.get('y2'))  # end
                # add transform
                x1 += self.transform[0]
                y1 += self.transform[1]
                x2 += self.transform[0]
                y2 += self.transform[1]

                y1 = -y1  # flip axis: in svg top left corner is [0,0], y-axis points downwards
                y2 = -y2  # make y-axis point upwards

                # make line GCode segment
                if not self.commands:
                    # this is the first command, so make it a G00
                    self.commands.append('G00 X'+str(x1)+' Y'+str(y1))
                self.commands.append('G01 X'+str(x2)+' Y'+str(y2))  # only add endpoint, startpoint comes from previous command

            elif 'path' in child.tag:
                path = child.get('d')
                # d='Mx,y c x1 y1 x2 y2 x y'
                # Mx,y = the startpoint or endpoint of the curve
                # c starts a curve, lower case means relative coordinates i.e. relative to Mx, y
                # x1 y1 is the control point that is closest to Mx, y
                # x2 y2 is the second control point
                # x y is the endpoint of the curve
                # d= can contain multiple c-commands = curves
                if path[0] == 'M':
                    path = path[1:].replace(","," ")  # replace comma by: space
                    path = path.replace("-"," -")  # replace minus sign by: space minus
                    path = path.replace("c"," c ")  # replace c by: space c space
                    path = path.split(' c ')

                    filtered_path = []
                    for curve in path:
                        curve = curve.split(' ')
                        # remove all empty strings
                        curve = [e for e in curve if e!= '']
                        curve = [e for e in curve if e!= ' ']
                        filtered_path.append(curve)  # save filtered path

                    # the first border point of the curve (later decide if this is start or end)
                    curve_point1 = filtered_path[0]
                    curve_point1[0] = float(curve_point1[0])
                    curve_point1[1] = -float(curve_point1[1])  # minus: let y-axis point up
                    filtered_path.pop(0)  # remove first point from path

                    if filtered_path:
                        # there are curves in the path,
                        # loop over all curves
                        circle_points = []
                        for curve in filtered_path:
                            if not circle_points:
                                # first circle point, move starting from Mx, y = curve_point1
                                # note: minus sign for y-direction
                                circle_points.append([float(curve[-2])+curve_point1[0], -float(curve[-1])+curve_point1[1]])
                            else:
                                # this was not the first circle point, move relative from previous point
                                # note: minus sign for y-direction
                                circle_points.append([float(curve[-2])+circle_points[-1][0], -float(curve[-1])+circle_points[-1][1]])

                        # add endpoint of curve
                        circle_points.insert(0,curve_point1)

                        # For now this function supposes that each curve consists of minimum three points.
                        # If not, you need to approximate the circle shape in another way

                        # if len(circle_points) == 2:
                        #     # compute circle center when only one curve is present
                        #     # do this by solving a system with 3 equations in unknowns: xc, yc, r
                        #     # curve_point1 and curve_point2 have to lie on the circle:
                        #     # (x1-xc)**2+(y1-yc)**2-r**2=0
                        #     # (x2-xc)**2+(y2-yc)**2-r**2=0
                        #     # the normal to the connection between (x1,y1) and control point has to pass through xc, yc:
                        #     # a*xc+b*yc+c=0

                        #     control_point1 = np.array(curve_point1) + np.array([float(curve[0]), -float(curve[1])])
                        #     curve_point2 = circle_points[-1]
                        #     control_point2 = np.array(curve_point2) - np.array([float(curve[2]), -float(curve[3])])

                        #     # (y-y1) = (y2-y1)/(x2-x1)(x-x1) --> ax+by+c=0

                        #     x1 = curve_point1[0]
                        #     y1 = curve_point1[1]
                        #     x2 = curve_point2[0]
                        #     y2 = curve_point2[1]
                        #     a = (x2-x1)
                        #     b = (y2-y1)
                        #     c = x1*(x2-x1)-y1*(y2-y1)

                        #     cx = (b*x1**2 + b*y1**2 - b*x2**2 - b*y2**2 + 2*c*y1 - 2*c*y2)/(2*(-a*y1 + a*y2 + b*x1 - b*x2))
                        #     cy = (-a*x1**2 - a*y1**2 + a*x2**2 + a*y2**2 - 2*c*x1 + 2*c*x2)/(2*(-a*y1 + a*y2 + b*x1 - b*x2))

                        #     r = np.sqrt((x1-cx)**2+(y1-cy)**2)

                        #     # # do this by finding the intersection of the normals on the connection
                        #     # # between the end points of the curve and their respective control point
                        #     # curve = filtered_path[0]

                        #     # control_point1 = np.array(curve_point1) + np.array([float(curve[0]), -float(curve[1])])
                        #     # curve_point2 = circle_points[-1]
                        #     # control_point2 = np.array(curve_point2) - np.array([float(curve[2]), -float(curve[3])])

                        #     # p1 = curve_point1
                        #     # p2 = control_point1
                        #     # p3 = curve_point2
                        #     # p4 = control_point2

                        #     # if (p1[0] == p2[0] and p3[0] == p4[0]):
                        #     #     # vertical lines between end and control points
                        #     #     cx = (p1[0] + p3[0])*0.5
                        #     #     cy = p1[1]  # = p3[1]
                        #     # elif (p1[1] == p2[1] and p3[1] == p4[1]):
                        #     #     # horizontal lines between and and control points
                        #     #     cx = p1[0]  # p3[0]
                        #     #     cy = (p1[1] + p3[1])*0.5
                        #     # elif (p1[0] == p2[0] and p3[1] == p4[1]):
                        #     #     cx = p3[0]
                        #     #     cy = p1[1]
                        #     # elif (p1[1] == p2[1] and p3[0] == p4[0]):
                        #     #     cx = p1[0]
                        #     #     cy = p3[1]
                        #     # elif p2[0] == p1[0]:
                        #     #     # vertical line1
                        #     #     cy = p1[1]

                        #     #     rico2 = (p4[1]-p3[1])/(p4[0]-p3[0])
                        #     #     normal2 = -1/rico2
                        #     #     cx = (cy-p3[1])/normal2 + p3[0]
                        #     # elif p2[1] == p1[1]:
                        #     #     # horizontal line1
                        #     #     cx = p1[0]

                        #     #     rico2 = (p4[1]-p3[1])/(p4[0]-p3[0])
                        #     #     normal2 = -1/rico2
                        #     #     cy = p3[1] + normal2*(cx-p3[0])
                        #     # elif p3[0] == p4[0]:
                        #     #     # vertical line1
                        #     #     cy = p3[1]

                        #     #     rico2 = (p2[1]-p1[1])/(p2[0]-p1[0])
                        #     #     normal2 = -1/rico2
                        #     #     cx = (cy-p1[1])/normal2 + p1[0]
                        #     # elif p3[1] == p4[1]:
                        #     #     # horizontal line1
                        #     #     cx = p3[0]

                        #     #     rico2 = (p2[1]-p1[1])/(p2[0]-p1[0])
                        #     #     normal2 = -1/rico2
                        #     #     cy = p1[1] + normal2*(cx-p1[0])
                        #     # else:
                        #     #     rico1 = (p2[1]-p1[1])/(p2[0]-p1[0])
                        #     #     normal1 = -1/rico1
                        #     #     #y = mid1[1] + normal1*(x-mid1[0]) [1]
                        #     #     rico2 = (p4[1]-p3[1])/(p4[0]-p3[0])
                        #     #     normal2 = -1/rico2
                        #     #     #y = mid2[1] + normal2*(x-mid2[0]) [2]

                        #     #     # [1] = [2] --> x =
                        #     #     if normal2 != normal1:
                        #     #         cx = np.round((p1[1]-p3[1]-normal1*p1[0]+normal2*p3[0])/(normal2-normal1),1)
                        #     #     else:
                        #     #         raise RuntimeError('Normals are equal, something went wrong')

                        #     #     cy = np.round(p1[1] + normal1*(cx-p1[0]),1)

                        #     # r = np.round(np.sqrt((p1[0]-cx)**2+(p1[1]-cy)**2),1)

                        #     # plot solution
                        #     plt.figure(11)
                        #     eval = np.linspace(0,2*np.pi,100)
                        #     plt.plot(cx+r*np.cos(eval),cy+r*np.sin(eval),'g-')
                        #     plt.plot(cx,cy,'gx')
                        #     plt.plot(p1[0],p1[1],'rx')
                        #     # plt.plot(p2[0],p2[1],'rx')
                        #     plt.plot(p3[0],p3[1],'rx')
                        #     # plt.plot(p4[0],p4[1],'rx')

                        if len(circle_points) > 2:
                            # find circle through first three points
                            # by finding the intersection of the two perpendicular bisectors through [p1p2] and [p2p3]
                            p1, p2, p3 = circle_points[:3]
                            mid1 = [(p2[0]+p1[0])*0.5, (p2[1]+p1[1])*0.5]  # midpoint of first bisector
                            mid2 = [(p3[0]+p2[0])*0.5, (p3[1]+p2[1])*0.5]
                            cx, cy = [], []  # will hold circle center

                            if (p1[0] == p2[0] and p2[1] == p3[1]):
                                # vertical and horizontal bisectors
                                cx = mid1[0]
                                cy = mid2[1]
                            elif (p1[1] == p2[1] and p2[0] == p3[0]):
                                # horizontal and vertical bisectors
                                cx = mid2[0]
                                cy = mid1[1]
                            elif p2[0] == p1[0]:
                                # vertical bisector1
                                cx = mid1[0]
                                rico2 = (p3[1]-p2[1])/(p3[0]-p2[0])
                                normal2 = -1/rico2
                                cy = mid2[1] + normal2*(x-mid2[0])
                            elif p2[1] == p1[1]:
                                # horizontal bisector1
                                cy = mid1[1]
                                rico2 = (p3[1]-p2[1])/(p3[0]-p2[0])
                                normal2 = -1/rico2
                                cx = (cy-mid2[1])/normal2 + mid2[0]
                            elif p2[0] == p3[0]:
                                # vertical bisector2
                                cx = mid2[0]
                                rico1 = (p2[1]-p1[1])/(p2[0]-p1[0])
                                normal1 = -1/rico1
                                cy = mid1[1] + normal1*(x-mid1[0])
                            elif p2[1] == p3[1]:
                                # horizontal bisector2
                                cy = mid2[1]
                                rico1 = (p2[1]-p1[1])/(p2[0]-p1[0])
                                normal1 = -1/rico1
                                cx = (cy-mid1[1])/normal1 + mid1[0]
                            else:
                                # two diagonal bisectors
                                rico1 = (p2[1]-p1[1])/(p2[0]-p1[0])
                                normal1 = -1/rico1
                                # y = mid1[1] + normal1*(x-mid1[0]) [1]
                                rico2 = (p3[1]-p2[1])/(p3[0]-p2[0])
                                normal2 = -1/rico2
                                # y = mid2[1] + normal2*(x-mid2[0]) [2]

                                # [1] = [2] --> x =
                                if normal2 != normal1:
                                    cx = (mid1[1]-mid2[1]-normal1*mid1[0]+normal2*mid2[0])/(normal2-normal1)
                                else:
                                    raise RuntimeError('Normals are equal, something went wrong')

                                cy = mid1[1] + normal1*(cx-mid1[0])

                            # compute radius
                            r = np.sqrt((p1[0]-cx)**2+(p1[1]-cy)**2)

                            # plot solution
                            # plt.figure(11)
                            # eval = np.linspace(0,2*np.pi,100)
                            # plt.plot(cx+r*np.cos(eval),cy+r*np.sin(eval),'g-')
                            # plt.plot(cx,cy,'gx')
                            # # plt.plot(mid1[0],mid1[1],'rx')
                            # # plt.plot(mid2[0],mid2[1],'rx')
                            # plt.plot(p2[0],p2[1],'rx')
                            # plt.plot(p1[0],p1[1],'rx')
                            # plt.plot(p3[0],p3[1],'rx')

                        else:
                            raise RuntimeError('Curve must consist of more than two points for the moment')

                        # given radius and center
                        # compute I an J from center

                        # compute end point of curves
                        curve_point2 = np.array(curve_point1)
                        for curve in filtered_path:
                            # add relative positions of all curves
                            curve_point2 += np.array([float(curve[-2]), -float(curve[-1])])

                        # now decide what is start and end of curve, because start must connect to end of previous command
                        prev_seg_end = [float(self.commands[-1].split(' ')[1].split('X')[1]),
                                        float(self.commands[-1].split(' ')[2].split('Y')[1])]
                        dist1 = np.sqrt((curve_point1[0]-prev_seg_end[0])**2+(curve_point1[1]-prev_seg_end[1])**2)
                        dist2 = np.sqrt((curve_point2[0]-prev_seg_end[0])**2+(curve_point2[1]-prev_seg_end[1])**2)
                        # curve start point is closest to the end of the previous segment
                        if dist1 < dist2:
                            start = curve_point1
                            end_curve = curve_point2
                            control_point = [start[0]+float(filtered_path[0][0]), start[1]-float(filtered_path[0][1])]
                        else:
                            start = curve_point2
                            end_curve = curve_point1
                            control_point = [start[0]-float(filtered_path[-1][4])+float(filtered_path[-1][2]),
                                             start[1]+float(filtered_path[-1][5])-float(filtered_path[-1][3])]

                        # compute I and J
                        I = cx - start[0]
                        J = cy - start[1]

                        # determine if circle goes clockwise or counter-clockwise
                        # by taking the vector product of the center to the start point & start to control point
                        v1 = [start[0]-cx, start[1]-cy]
                        v2 = [control_point[0]-start[0], control_point[1]-start[1]]
                        vector_product = v1[0]*v2[1] - v2[0]*v1[1]

                        if vector_product < 0:
                            # clockwise arc
                            self.commands.append('G02 X'+str(end_curve[0])+' Y'+str(end_curve[1])+ ' I'+str(I)+' J'+str(J))
                        else:
                            # counter-clockwise arc
                            self.commands.append('G03 X'+str(end_curve[0])+' Y'+str(end_curve[1])+ ' I'+str(I)+' J'+str(J))
                    else:
                        # there are no curves in the path, probably it just contains a move command
                        # so it represents a GCode line segment
                        self.commands.append('G01 X'+str(end_curve[0])+' Y'+str(end_curve[1]))
                else:
                    raise RuntimeError('Only absolute positioning of the start of the curve is supported for now')

        # Write .nc file
        old_name = self.data.name.split('/')[-1][:-4]
        f = open(old_name+'_gcode.nc', 'w')
        for command in self.commands:
            f.write(command + '\n')
        f.close()