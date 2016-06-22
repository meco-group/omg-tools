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


class Shape:

    def __init__(self, dimensions):
        self.n_dim = dimensions
        self._prepare_draw()

    def draw(self, pose):
        raise ValueError('Please implement this method.')

    def _prepare_draw(self):
        raise ValueError('Please implement this method.')


class Shape2D(Shape):

    def __init__(self):
        Shape.__init__(self, 2)

    def rotate(self, orientation, coordinate):
        if isinstance(orientation, np.ndarray):
            orientation = orientation[0]
        cth = np.cos(orientation)
        sth = np.sin(orientation)
        rot = np.array([[cth, -sth], [sth, cth]])
        return rot.dot(coordinate)

    def draw(self, pose=np.zeros(3)):
        return [np.c_[pose[:2]] + line for line in self.plt_lines]

    def get_sides(self, vertices):
        n_vert = vertices.shape[1]
        return [np.c_[vertices[:, l], vertices[:, (l+1) % n_vert]] for l in range(n_vert)]


class Circle(Shape2D):

    def __init__(self, radius):
        self.radius = radius
        self.n_chck = 1
        Shape2D.__init__(self)

    def _prepare_draw(self):
        s = np.linspace(0, 1-1./48, 48)
        points = np.vstack(
            (self.radius*np.cos(s*2*np.pi), self.radius*np.sin(s*2*np.pi)))
        self.plt_lines = self.get_sides(points)

    def get_checkpoints(self):
        return [[0., 0.]], [self.radius]

    def get_canvas_limits(self):
        return [np.array([-self.radius, self.radius]),
                np.array([-self.radius, self.radius])]


class Polyhedron(Shape2D):

    def __init__(self, vertices, orientation=0., radius=1e-3):
        self.vertices = vertices
        self.n_vert = vertices.shape[1]
        Shape2D.__init__(self)
        self.orientation = orientation
        self.plt_lines = [self.rotate(orientation, line)
                          for line in self.plt_lines]
        self.vertices = self.rotate(orientation, self.vertices)
        # radius should be greater than zero for obstacle avoidance between
        # 2 polyhedra
        self.radius = radius

    def _prepare_draw(self):
        self.plt_lines = self.get_sides(self.vertices)

    def draw(self, pose=np.zeros(3)):
        return [np.c_[pose[:2]] + self.rotate(pose[2], line) for line in self.plt_lines]

    def get_checkpoints(self):
        chck = [[self.vertices[0, l], self.vertices[1, l]]
                for l in range(self.n_vert)]
        rad = [self.radius for l in range(self.n_vert)]
        return chck, rad

    def get_canvas_limits(self):
        max_xy = np.amax(self.vertices, axis=1)
        min_xy = np.amin(self.vertices, axis=1)
        return [np.array([min_xy[0], max_xy[0]]),
                np.array([min_xy[1], max_xy[1]])]

    def get_hyperplanes(self, **kwargs):
        pos = [0, 0]  # default
        if 'position' in kwargs:  # overwrite default
            pos = kwargs['position']
        vertices = np.hstack(
            (self.vertices, np.vstack(self.vertices[:, 0])))  # gives e.g. a (2, 4)
        hyperplanes = {}
        for k in range(vertices.shape[1]-1):
            vector = [vertices[0][k+1]-vertices[0][k],
                      vertices[1][k+1]-vertices[1][k]]
            normal = [-vector[1], vector[0]]/np.sqrt(vector[0]**2+vector[1]**2)
            b = normal[0]*(vertices[0][k+1]+pos[0]) + normal[1]*(vertices[1][k+1]+pos[1])
            hyperplanes[k] = {'a': normal, 'b': b}
        return hyperplanes


class Beam(Polyhedron):

    def __init__(self, width, height, orientation=0.):
        self.width = width
        self.height = height
        Polyhedron.__init__(self, np.c_[[0.5*width, 0.], [-0.5*width, 0.]],
                            orientation=orientation, radius=0.5*height)

    def _prepare_draw(self):
        s = np.linspace(0, 1, 25)
        a = [0.5*self.width+0.5*self.height*np.cos(s*np.pi - 0.5*np.pi),
             0.5*self.height*np.sin(s*np.pi - 0.5*np.pi)]
        b = [-0.5*self.width+0.5*self.height*np.cos(s*np.pi + 0.5*np.pi),
             0.5*self.height*np.sin(s*np.pi + 0.5*np.pi)]
        points = np.c_[a, b]
        self.plt_lines = self.get_sides(points)


class RegularPolyhedron(Polyhedron):

    def __init__(self, radius, n_vert, orientation=0.):
        # radius of outer circle (the one through the vertices)
        self.radius = radius
        self.n_vert = n_vert
        Polyhedron.__init__(self, self.get_vertices(), orientation)

    def get_vertices(self):
        A = np.zeros((self.n_vert, 2))
        B = np.zeros((self.n_vert, 1))
        dth = (2*np.pi)/self.n_vert
        for l in range(self.n_vert):
            A[l, :] = np.array([np.sin(l*dth), np.cos(l*dth)])
            B[l] = self.radius*np.cos(np.pi/self.n_vert)
        vertices = np.zeros((2, self.n_vert))
        for l in range(self.n_vert):
            a = np.vstack((A[l, :], A[(l+1) % self.n_vert, :]))
            b = np.vstack((B[l], B[(l+1) % self.n_vert]))
            vertices[:, l] = np.linalg.solve(a, b).ravel()
        return vertices


class Square(RegularPolyhedron):

    def __init__(self, side, orientation=0.):
        RegularPolyhedron.__init__(self, side/np.sqrt(2), 4, orientation)


class Rectangle(Polyhedron):

    def __init__(self, width, height, orientation=0.):
        self.width = width
        self.height = height
        Polyhedron.__init__(self, self.get_vertices(), orientation)

    def get_vertices(self):
        A = np.zeros((4, 2))
        B = np.zeros((4, 1))
        radius = [0.5*self.height, 0.5*self.width,
                  0.5*self.height, 0.5*self.width]
        dth = 0.5*np.pi
        for l in range(4):
            A[l, :] = np.array([np.sin(l*dth), np.cos(l*dth)])
            B[l] = radius[l]
        vertices = np.zeros((2, 4))
        for l in range(4):
            a = np.vstack((A[l, :], A[(l+1) % 4, :]))
            b = np.vstack((B[l], B[(l+1) % 4]))
            vertices[:, l] = np.linalg.solve(a, b).ravel()
        return vertices


class UFO(Rectangle):

    def __init__(self, width, height, orientation=0.):
        Rectangle.__init__(self, width, height, orientation)

    def _prepare_draw(self):
        w = self.width
        h = self.height
        plt_x = [-0.5*w, -0.2*w, 0.2*w, 0.5*w,
                 0.2*w, 0.15*w, -0.15*w, -0.2*w]
        plt_y = [-0.15*h, -0.5*h, -0.5*h, -0.15*h,
                 0.2*h, 0.5*h, 0.5*h, 0.2*h]
        points = np.vstack((plt_x, plt_y))
        self.plt_lines = self.get_sides(points)


class Shape3D(Shape):

    def __init__(self):
        Shape.__init__(self, 3)

    def draw(self, pose=np.zeros(6)):
        return [np.c_[pose[:3]] + line for line in self.plt_lines]


class Polyhedron3D(Shape3D):

    def __init__(self, vertices, radius=1e-3):
        self.vertices = vertices
        self.n_vert = vertices.shape[1]
        self.radius = radius
        Shape3D.__init__(self)

    def get_checkpoints(self):
        chck = [[self.vertices[0, l], self.vertices[1, l], self.vertices[2, l]]
                for l in range(self.n_vert)]
        rad = [self.radius for l in range(self.n_vert)]
        return chck, rad

    def get_canvas_limits(self):
        max_xyz = np.amax(self.vertices, axis=1)
        min_xyz = np.amin(self.vertices, axis=1)
        return [np.array([min_xyz[0], max_xyz[0]]),
                np.array([min_xyz[1], max_xyz[1]]),
                np.array([min_xyz[2], max_xyz[2]])]

    def get_sides(self, vertices):
        n_vert = vertices.shape[1]
        return [np.c_[vertices[:, l], vertices[:, (l+1) % n_vert]] for l in range(n_vert)]

    def _prepare_draw(self):
        # stupid implementation, hard to do this in general
        self.plt_lines = self.get_sides(self.vertices)


class RegularPrisma(Polyhedron3D):

    def __init__(self, radius, height, n_faces):
        # radius of outer circle of surface (the one through the vertices)
        self.radius = radius
        self.n_faces = n_faces
        Polyhedron3D.__init__(self, self.get_vertices())

    def get_sides(self, vertices):
        sides = []
        for l in range(self.n_faces):
            sides.append(
                np.c_[vertices[:, l], vertices[:, (l+1) % self.n_faces]])
            sides.append(np.c_[
                         vertices[:, l+self.n_faces], vertices[:, (l+1) % self.n_faces + self.n_faces]])
            sides.append(
                np.c_[vertices[:, l], self.vertices[:, l + self.n_faces]])
        return sides

    def _prepare_draw(self):
        self.plt_lines = self.get_sides(self.vertices)

    def get_vertices(self):
        A = np.zeros((self.n_faces, 2))
        B = np.zeros((self.n_faces, 1))
        dth = (2*np.pi)/self.n_faces
        for l in range(self.n_faces):
            A[l, :] = np.array([np.sin(l*dth), np.cos(l*dth)])
            B[l] = self.radius*np.cos(np.pi/self.n_faces)
        vertices = np.zeros((3, self.n_faces*2))
        for l in range(self.n_faces):
            a = np.vstack((A[l, :], A[(l+1) % self.n_faces, :]))
            b = np.vstack((B[l], B[(l+1) % self.n_faces]))
            vertices[:, l] = np.linalg.solve(a, b).ravel()
            vertices[2, l] = -0.5*self.height
            vertices[:2, l+self.n_faces] = vertices[:2, l]
            vertices[2, l+self.n_faces] = vertices[2, l] + self.height
        return vertices


class Cuboid(Polyhedron3D):

    def __init__(self, width, depth, height):
        self.width = width
        self.depth = depth
        self.height = height
        Polyhedron3D.__init__(self, self.get_vertices())

    def get_sides(self, vertices):
        sides = []
        for l in range(4):
            sides.append(np.c_[vertices[:, l], vertices[:, (l+1) % 4]])
            sides.append(np.c_[vertices[:, l+4], vertices[:, (l+1) % 4 + 4]])
            sides.append(np.c_[vertices[:, l], self.vertices[:, l + 4]])
        return sides

    def _prepare_draw(self):
        self.plt_lines = self.get_sides(self.vertices)

    def get_vertices(self):
        A = np.zeros((4, 2))
        B = np.zeros((4, 1))
        radius = [0.5*self.depth, 0.5*self.width,
                  0.5*self.depth, 0.5*self.width]
        dth = 0.5*np.pi
        for l in range(4):
            A[l, :] = np.array([np.sin(l*dth), np.cos(l*dth)])
            B[l] = radius[l]
        vertices = np.zeros((3, 8))
        for l in range(4):
            a = np.vstack((A[l, :], A[(l+1) % 4, :]))
            b = np.vstack((B[l], B[(l+1) % 4]))
            vertices[:2, l] = np.linalg.solve(a, b).ravel()
            vertices[2, l] = -0.5*self.height
            vertices[:2, l+4] = vertices[:2, l]
            vertices[2, l+4] = vertices[2, l] + self.height
        return vertices


class Cube(Cuboid):

    def __init__(self, side):
        Cuboid.__init__(self, side, side, side)


class Plate(Polyhedron3D):

    def __init__(self, shape2d, height):
        self.shape2d = shape2d
        vertices = np.r_[
            shape2d.vertices, np.zeros((1, shape2d.vertices.shape[1]))]
        Polyhedron3D.__init__(self, vertices, 0.5*height)

    def _prepare_draw(self):
        lines2d = self.shape2d.plt_lines
        self.plt_lines = [np.r_[l, np.zeros((1, 2))] for l in lines2d]

    def draw(self, pose=np.zeros(6)):
        return [np.c_[pose[:3]] + line for line in self.plt_lines]
