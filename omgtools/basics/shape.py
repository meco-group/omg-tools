import numpy as np


class Shape:

    def __init__(self, dimensions):
        self.n_dim = dimensions
        self._prepare_draw()

    def draw(self, pose):
        return self.plt_co


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
        self.n_faces = vertices.shape[1]
        self.n_chck = self.n_faces
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
                for l in range(self.n_faces)]
        # give small radius to account for anti-collision between two polyhedra
        rad = [1e-3 for l in range(self.n_faces)]
        return chck, rad

    def get_canvas_limits(self):
        max_xy = np.amax(self.vertices, axis=1)
        min_xy = np.amin(self.vertices, axis=1)
        return [np.array([min_xy[0], max_xy[0]]),
                np.array([min_xy[1], max_xy[1]])]


class RegularPolyhedron(Polyhedron):

    def __init__(self, radius, n_faces, orientation=0.):
        # radius of outer circle (the one through the vertices)
        self.radius = radius
        self.n_faces = n_faces
        Polyhedron.__init__(self, self.getVertices(), orientation)

    def getVertices(self):
        A = np.zeros((self.n_faces, 2))
        B = np.zeros((self.n_faces, 1))
        dth = (2*np.pi)/self.n_faces
        for l in range(self.n_faces):
            A[l, :] = np.array([np.sin(l*dth), np.cos(l*dth)])
            B[l] = self.radius*np.cos(np.pi/self.n_faces)
        vertices = np.zeros((2, self.n_faces))
        for l in range(self.n_faces):
            a = np.vstack((A[l, :], A[(l+1) % self.n_faces, :]))
            b = np.vstack((B[l], B[(l+1) % self.n_faces]))
            vertices[:, l] = np.linalg.solve(a, b).ravel()
        return vertices


class Square(RegularPolyhedron):

    def __init__(self, width, orientation=0.):
        RegularPolyhedron.__init__(self, width/np.sqrt(2), 4, orientation)


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
