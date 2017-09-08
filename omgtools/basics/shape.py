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

    def draw(self, pose):
        raise ValueError('Please implement this method.')


class Shape2D(Shape):

    def __init__(self, surfaces):
        self.surfaces = surfaces
        Shape.__init__(self, 2)

    def rotate(self, orientation, coordinate):
        if isinstance(orientation, np.ndarray):
            orientation = orientation[0]
        cth = np.cos(orientation)
        sth = np.sin(orientation)
        rot = np.array([[cth, -sth], [sth, cth]])
        return rot.dot(coordinate)

    def draw(self, pose=np.zeros(3)):
        return [np.c_[pose[:2]] + self.rotate(pose[2], surf) for surf in self.surfaces], []


class Circle(Shape2D):

    def __init__(self, radius):
        self.radius = radius
        self.n_chck = 1
        Shape2D.__init__(self, self.get_surfaces())

    def get_surfaces(self):
        s = np.linspace(0, 1-1./48, 48)
        points = np.vstack(
            (self.radius*np.cos(s*2*np.pi), self.radius*np.sin(s*2*np.pi)))
        return [points]

    def get_checkpoints(self):
        return [[0., 0.]], [self.radius]

    def get_canvas_limits(self):
        return [np.array([-self.radius, self.radius]),
                np.array([-self.radius, self.radius])]

class Ring(Shape2D):
    def __init__(self, radius_in, radius_out, start, end, direction):
        self.radius_in = radius_in
        self.radius_out = radius_out
        self.start = start
        self.end = end
        self.direction = direction
        Shape2D.__init__(self, self.get_surfaces())

    def get_surfaces(self):
        if (self.start == self.end).all():
            # full ring, placed in the origin
            s = linspace(0,2*np.pi,50)
        else:
            # part of a ring, placed in the origin
            start_angle = np.arctan2(self.start[1],self.start[0])
            end_angle = np.arctan2(self.end[1],self.end[0])
            if self.direction == 'CW':
                if start_angle < end_angle:
                    start_angle += 2*np.pi  # arctan2 returned a negative start_angle, make positive
            elif self.direction == 'CCW':
                if start_angle > end_angle:  # arctan2 returned a negative end_angle, make positive
                    end_angle += 2*np.pi
            s = np.linspace(start_angle, end_angle, 50)
        # inner radius
        points_in = np.vstack(
            (self.radius_in*np.cos(s), self.radius_in*np.sin(s)))
        # outer radius
        # move over outer points in other direction, to obtain a closed figure without diagonal connection
        # between end of inner circle and beginning of outer circle
        s = np.flipud(s)
        points_out = np.vstack(
            (self.radius_out*np.cos(s), self.radius_out*np.sin(s)))
        points = np.c_[points_in, points_out]
        return [points]

    def get_canvas_limits(self):
        points = np.c_[self.start, self.end]
        max_xy = np.amax(points, axis=1)
        min_xy = np.amin(points, axis=1)
        return [np.array([min_xy[0], max_xy[0]]),
                np.array([min_xy[1], max_xy[1]])]

    # def get_checkpoints():  # not applicable because non-convex shape

class Polyhedron(Shape2D):

    def __init__(self, vertices, orientation=0., radius=1e-3):
        self.vertices = vertices
        self.n_vert = vertices.shape[1]
        Shape2D.__init__(self, self.get_surfaces(vertices))
        self.orientation = orientation
        self.vertices = self.rotate(orientation, self.vertices)
        self.surfaces = [self.rotate(orientation, surf) for surf in self.surfaces]
        # radius should be greater than zero for obstacle avoidance between
        # 2 polyhedra
        self.radius = radius

    def get_surfaces(self, vertices):
        return [vertices]

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

    def get_surfaces(self, vertices):
        s = np.linspace(0, 1-1./24, 24)
        a = [vertices[0, 0]+0.5*self.height*np.cos(s*np.pi - 0.5*np.pi),
             vertices[1, 0]+0.5*self.height*np.sin(s*np.pi - 0.5*np.pi)]
        b = [vertices[0, 1]+0.5*self.height*np.cos(s*np.pi + 0.5*np.pi),
             vertices[1, 1]+0.5*self.height*np.sin(s*np.pi + 0.5*np.pi)]
        return [np.c_[a, b]]


class RegularPolyhedron(Polyhedron):

    def __init__(self, radius, n_vert, orientation=0.):
        # radius of outer circle (the one through the vertices)
        self.radius = radius
        self.n_vert = n_vert
        vertices = self.get_vertices()
        Polyhedron.__init__(self, vertices, orientation)

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


class Square(Rectangle):

    def __init__(self, side, orientation=0.):
        Rectangle.__init__(self, side, side, orientation)


class UFO(Rectangle):

    def __init__(self, width, height, orientation=0.):
        Rectangle.__init__(self, width, height, orientation)

    def get_surfaces(self, vertices):
        w = self.width
        h = self.height
        plt_x = [-0.5*w, -0.2*w, 0.2*w, 0.5*w,
                 0.2*w, 0.15*w, -0.15*w, -0.2*w]
        plt_y = [-0.15*h, -0.5*h, -0.5*h, -0.15*h,
                 0.2*h, 0.5*h, 0.5*h, 0.2*h]
        return [np.vstack((plt_x, plt_y))]


class Shape3D(Shape):

    def __init__(self, surfaces):
        self.surfaces = surfaces
        Shape.__init__(self, 3)

    def draw(self, pose=np.zeros(6)):
        return [np.c_[pose[:3]] + self.rotate(pose[3:].tolist(), surf) for surf in self.surfaces], []

    def rotate(self, orientation, coordinate):
        if len(orientation) != 3:
            raise ValueError('Orientation is a list with 3 elements: roll, pitch, yaw!')
        roll, pitch, yaw = orientation
        cpsi, spsi = np.cos(yaw), np.sin(yaw)
        cth, sth = np.cos(pitch), np.sin(pitch)
        cphi, sphi = np.cos(roll), np.sin(roll)
        rot = np.array([[cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi],
                        [cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi],
                        [-sth, sphi*cth, cphi*cth]])
        return rot.dot(coordinate)


class Sphere(Shape3D):

    def __init__(self, radius):
        self.radius = radius
        self.n_chck = 1
        Shape3D.__init__(self, [])
        Shape3D.__init__(self, self.get_surfaces())

    def get_surfaces(self):
        surf = []
        pnts = []
        nk, nl = 5, 12
        for k in range(nk):
            radius = self.radius*np.cos(k*np.pi/(2*(nk-1)))
            height = self.radius*np.sin(k*np.pi/(2*(nk-1)))
            pnt0 = np.r_[radius, 0, height]
            pnts.append([self.rotate([0, 0, l*np.pi/(nl/2.)], pnt0) for l in range(12)])
        for k in range(nk):
            for l in range(nl):
                srf = np.c_[pnts[k][l], pnts[k][(l+1)%nl], pnts[(k+1)%nk][(l+1)%nl], pnts[(k+1)%nk][l]]
                srf2 = np.vstack((srf[:2, :], -srf[2, :]))
                surf += [srf, srf2]
        return surf

    # def _prepare_draw(self):
    #     self.plt_lines = []
    #     s = np.linspace(0, 1-1./48, 48)
    #     # vertical circles
    #     circle_v = np.vstack((self.radius*np.cos(s*2*np.pi), np.zeros(len(s)), self.radius*np.sin(s*2*np.pi)))
    #     for k in range(6):
    #         points = self.rotate([0, 0, k*np.pi/6], circle_v)
    #         n_vert = points.shape[1]
    #         self.plt_lines += [np.c_[points[:, l], points[:, (l+1)%n_vert]] for l in range(n_vert)]
    #     # horizontal circles
    #     for k in range(4):
    #         radius = self.radius*np.cos(k*np.pi/8)
    #         height = self.radius*np.sin(k*np.pi/8)
    #         circle_h = np.vstack((radius*np.cos(s*2*np.pi), radius*np.sin(s*2*np.pi), np.zeros(len(s))))
    #         if height > 0:
    #             for j in range(2):
    #                 points = circle_h + ((-1)**j)*np.vstack((0., 0., height))
    #                 n_vert = points.shape[1]
    #                 self.plt_lines += [np.c_[points[:, l], points[:, (l+1)%n_vert]] for l in range(n_vert)]
    #         else:
    #             points = circle_h
    #             n_vert = points.shape[1]
    #             self.plt_lines += [np.c_[points[:, l], points[:, (l+1)%n_vert]] for l in range(n_vert)]

    def get_checkpoints(self):
        return [[0., 0., 0.]], [self.radius]

    def get_canvas_limits(self):
        return [np.array([-self.radius, self.radius]),
                np.array([-self.radius, self.radius]),
                np.array([-self.radius, self.radius])]


class Polyhedron3D(Shape3D):

    def __init__(self, vertices, surfaces, orientation=[0,0,0], radius=1e-3):
        self.vertices = vertices
        self.n_vert = vertices.shape[1]
        self.radius = radius
        Shape3D.__init__(self, surfaces)
        self.orientation = orientation
        self.vertices = self.rotate(orientation, self.vertices)
        self.surfaces = [self.rotate(self.orientation, surf) for surf in self.surfaces]

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


class RegularPrisma(Polyhedron3D):

    def __init__(self, radius, height, n_faces, orientation=[0, 0, 0]):
        # radius of outer circle of surface (the one through the vertices)
        self.radius = radius
        self.height = height
        self.n_faces = n_faces
        vertices = self.get_vertices()
        surfaces = self.get_surfaces(vertices)
        Polyhedron3D.__init__(self, vertices, surfaces, orientation)

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
            vertices[:2, l] = np.linalg.solve(a, b).ravel()
            vertices[2, l] = -0.5*self.height
            vertices[:2, l+self.n_faces] = vertices[:2, l]
            vertices[2, l+self.n_faces] = vertices[2, l] + self.height
        return vertices

    def get_surfaces(self, vertices):
        surf = []
        surf.append(np.c_[[vertices[:, k] for k in range(self.n_faces)]].T)
        surf.append(np.c_[[vertices[:, k+self.n_faces] for k in range(self.n_faces)]].T)
        for l in range(self.n_faces):
            surf.append(np.c_[[vertices[:, l], vertices[:, (l+1)%self.n_faces],
                vertices[:, self.n_faces+(l+1)%self.n_faces], vertices[:, self.n_faces+l]]].T)
        return surf


class Cuboid(Polyhedron3D):

    def __init__(self, width, depth, height, orientation=[0,0,0]):
        self.width = width
        self.depth = depth
        self.height = height
        vertices = self.get_vertices()
        surfaces = self.get_surfaces(vertices)
        Polyhedron3D.__init__(self, vertices, surfaces, orientation)

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

    def get_surfaces(self, vertices):
        surf = []
        surf.append(np.c_[[vertices[:, k] for k in range(4)]].T)
        surf.append(np.c_[[vertices[:, k+4] for k in range(4)]].T)
        for l in range(4):
            surf.append(np.c_[[vertices[:, l], vertices[:, (l+1)%4],
                vertices[:, 4+(l+1)%4], vertices[:, 4+l]]].T)
        return surf


class Cube(Cuboid):

    def __init__(self, side, orientation=[0, 0, 0]):
        Cuboid.__init__(self, side, side, side, orientation)


class Plate(Polyhedron3D):

    def __init__(self, shape2d, height, orientation=[0, 0, 0]):
        self.shape2d = shape2d
        vertices = np.r_[
            shape2d.vertices, np.zeros((1, shape2d.vertices.shape[1]))]
        surfaces = [np.r_[surf, np.zeros((1, surf.shape[1]))] for surf in shape2d.surfaces]
        Polyhedron3D.__init__(self, vertices, surfaces, orientation, 0.5*height)
