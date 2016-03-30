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

    def _prepare_draw():
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

    def draw(self, pose=np.zeros(2)):
        return np.c_[pose[:2]] + self.plt_co


class Circle(Shape2D):

    def __init__(self, radius):
        self.radius = radius
        self.n_chck = 1
        Shape2D.__init__(self)

    def _prepare_draw(self):
        s = np.linspace(0, 1, 50)
        self.plt_co = np.vstack(
            (self.radius*np.cos(s*2*np.pi), self.radius*np.sin(s*2*np.pi)))

    def get_checkpoints(self):
        return [[0., 0.]], [self.radius]

    def get_canvas_limits(self):
        return [np.array([-self.radius, self.radius]),
                np.array([-self.radius, self.radius])]


class Polyhedron(Shape2D):

    def __init__(self, vertices, orientation=0.):
        self.vertices = vertices
        self.n_vert = vertices.shape[1]
        Shape2D.__init__(self)
        self.orientation = orientation
        self.plt_co = self.rotate(orientation, self.plt_co)
        self.vertices = self.rotate(orientation, self.vertices)

    def _prepare_draw(self):
        self.plt_co = np.hstack(
            (self.vertices, np.vstack(self.vertices[:, 0])))

    def draw(self, pose=np.zeros(3)):
        return np.c_[pose[:2]] + self.rotate(pose[2], self.plt_co)

    def get_checkpoints(self):
        chck = [[self.vertices[0, l], self.vertices[1, l]]
                for l in range(self.n_vert)]
        # give small radius to account for anti-collision between two polyhedra
        rad = [1e-3 for l in range(self.n_vert)]
        return chck, rad

    def get_canvas_limits(self):
        max_xy = np.amax(self.vertices, axis=1)
        min_xy = np.amin(self.vertices, axis=1)
        return [np.array([min_xy[0], max_xy[0]]),
                np.array([min_xy[1], max_xy[1]])]


class RegularPolyhedron(Polyhedron):

    def __init__(self, radius, n_vert, orientation=0.):
        # radius of outer circle (the one through the vertices)
        self.radius = radius
        self.n_vert = n_vert
        Polyhedron.__init__(self, self.getVertices(), orientation)

    def getVertices(self):
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
        Polyhedron.__init__(self, self.getVertices(), orientation)

    def getVertices(self):
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


class Rocket(Rectangle):

    def __init__(self, width, height, orientation=0.):
        Rectangle.__init__(self, width, height, orientation)

    def _prepare_draw(self):
        w = self.width
        h = self.height
        plt_x = [-0.5*w, -0.25*w, 0.25*w, 0.5*w,
                 0.5*w, 0.25*w, -0.25*w, -0.5*w]
        plt_y = [0., 0.25*h, 0.25*h, 0.5*h, -0.5*h, -0.25*h, -0.25*h, 0.]
        self.plt_co = np.vstack((plt_x, plt_y))


class UFO(Rectangle):

    def __init__(self, width, height, orientation=0.):
        Rectangle.__init__(self, width, height, orientation)

    def _prepare_draw(self):
        w = self.width
        h = self.height
        plt_x = [-0.5*w, -0.2*w, 0.2*w, 0.5*w,
                 0.2*w, 0.15*w, -0.15*w, -0.2*w, -0.5*w]
        plt_y = [-0.15*h, -0.5*h, -0.5*h, -0.15*h,
                 0.2*h, 0.5*h, 0.5*h, 0.2*h, -0.15*h]
        self.plt_co = np.vstack((plt_x, plt_y))


class Shape3D(Shape):

    def __init__(self):
        Shape.__init__(self, 3)

    def draw(self, pose=np.zeros(6)):
        return [np.c_[pose[:3]] + p for p in self.plt_co]


class Polyhedron3D(Shape3D):

    def __init__(self, vertices):
        self.vertices = vertices
        self.n_vert = vertices.shape[1]
        Shape3D.__init__(self)

    def get_checkpoints(self):
        chck = [[self.vertices[0, l], self.vertices[1, l], self.vertices[2, l]]
                for l in range(self.n_vert)]
        # give small radius to account for anti-collision between two polyhedra
        rad = [1e-3 for l in range(self.n_vert)]
        return chck, rad

    def get_canvas_limits(self):
        max_xyz = np.amax(self.vertices, axis=1)
        min_xyz = np.amin(self.vertices, axis=1)
        return [np.array([min_xyz[0], max_xyz[0]]),
                np.array([min_xyz[1], max_xyz[1]]),
                np.array([min_xyz[2], max_xyz[2]])]


class RegularPrisma(Polyhedron3D):

    def __init__(self, radius, height, n_faces):
        # radius of outer circle of surface (the one through the vertices)
        self.radius = radius
        self.n_faces = n_faces
        Polyhedron3D.__init__(self, self.getVertices())

    def _prepare_draw(self):
        self.plt_co = []
        for l in range(4):
            self.plt_co.append(
                np.c_[self.vertices[:, l], self.vertics[:, (l+1) % self.n_faces]])
            self.plt_co.append(np.c_[self.vertices[
                               :, l+self.n_faces], self.vertics[:, (l+1) % self.n_faces + self.n_faces]])
            self.plt_co.append(
                np.c_[self.vertices[:, l], self.vertics[:, (l+self.n_faces)]])

    def getVertices(self):
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
        Polyhedron3D.__init__(self, self.getVertices())

    def _prepare_draw(self):
        self.plt_co = []
        for l in range(4):
            self.plt_co.append(
                np.c_[self.vertices[:, l], self.vertices[:, (l+1) % 4]])
            self.plt_co.append(
                np.c_[self.vertices[:, l+4], self.vertices[:, (l+1) % 4 + 4]])
            self.plt_co.append(
                np.c_[self.vertices[:, l], self.vertices[:, (l+4)]])

    def getVertices(self):
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
