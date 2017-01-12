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

import numpy as np


def distance_between_points(point1, point2):  
        # calculate distance between two points
        return np.sqrt((point2[0]-point1[0])**2+(point2[1]-point1[1])**2)

def distance_to_line(point, line):
    # returns the x- and y-direction distance from point to a line
    # taken from https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    x2, y2, x3, y3 = self.frame['border']['limits']
    # number vertices of border
    # v1--v2
    # |   |
    # v4--v3
    v1 = [x2,y2]
    v2 = [x2,y3]
    v3 = [x3,y3]
    v4 = [x3,y2]

    # dist from v1-v2
    dist12 = abs((v2[1]-v1[1])*point[0] - (v2[0]-v1[0])*point[1] + v2[0]*v1[1] - v2[1]*v1[0])/(np.sqrt((v2[1]-v1[1])**2+(v2[0]-v1[0])**2))
    #dist from v2-v3
    dist23 = abs((v3[1]-v2[1])*point[0] - (v3[0]-v2[0])*point[1] + v3[0]*v2[1] - v3[1]*v2[0])/(np.sqrt((v3[1]-v2[1])**2+(v3[0]-v2[0])**2))
    #dist from v3-v4
    # Note the minus sign! A negative shift in y-direction is required to lower the distance
    dist34 = -abs((v4[1]-v3[1])*point[0] - (v4[0]-v3[0])*point[1] + v4[0]*v3[1] - v4[1]*v3[0])/(np.sqrt((v4[1]-v3[1])**2+(v4[0]-v3[0])**2))
    #dist from v4-v1
    # Note the minus sign! A negative shift in x-direction is required to lower the distance
    dist41 = -abs((v1[1]-v4[1])*point[0] - (v1[0]-v4[0])*point[1] + v1[0]*v4[1] - v1[1]*v4[0])/(np.sqrt((v1[1]-v4[1])**2+(v1[0]-v4[0])**2))

    distance = [0.,0.]
    distance[0] = min(abs(dist12), abs(dist41))  # x-direction
    distance[1] = min(abs(dist23), abs(dist34))  # y-direction
    return distance

def ccw(A, B, C):
    # taken from http://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    # Determine if three points are listed in a counterclockwise order.
    # So say you have three points A, B and C. If the slope of the line AB is less than the slope of the line AC
    # then the three points are listed in a counterclockwise order.
    
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect_line_segments(line1, line2):
    A, B = line1
    C, D = line2
    # taken from http://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    # Think of two line segments AB, and CD. These intersect if and only if points A and B are
    # separated by segment CD and points C and D are separated by segment AB. 
    # If points A and B are separated by segment CD then ACD and BCD should have opposite orientation
    # meaning either ACD or BCD is counterclockwise but not both.
    
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def intersect_lines(line1, line2):
    # taken from https://en.wikipedia.org/wiki/Line-line_intersection
    x1, y1= line1[0]
    x2, y2= line1[1]
    x3, y3= line2[0]
    x4, y4= line2[1]

    intersection_point = [0.,0.]
    intersection_point[0] = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
    intersection_point[1] = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))

    return intersection_point