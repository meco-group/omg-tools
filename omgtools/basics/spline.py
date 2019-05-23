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

import functools
#import cvxopt
import numpy as np
import scipy.linalg as la
import casadi as cas
from scipy.sparse import csr_matrix
# from piecewise import PiecewisePolynomial as ppoly
# from scipy.sparse.linalg import spsolve
from collections import Counter
import hashlib

def md5(data):
  m = hashlib.md5()
  m.update(data)
  return m.digest()

NO_POINTS = 501


def memoize(f):
    """ Memoization decorator"""
    class memodict(dict):
        def __init__(self, f):
            self.f = f

        def __call__(self, *args):
            key = (args[0], md5(np.atleast_1d(args[1])))
            if key in self:
                return self[key]
            else:
                ret = self[key] = self.f(*args)
                return ret

        def __get__(self, obj, objtype):
            return functools.partial(self.__call__, obj)
    return memodict(f)


def cached_class(klass):
    """Decorator to cache class instances by constructor arguments.
    """
    cache = {}

    @functools.wraps(klass, assigned=('__name__', '__module__'), updated=())
    class _decorated(klass):
        __doc__ = klass.__doc__

        def __new__(cls, *args, **kwds):
            key = (cls,) + tuple([md5(np.atleast_1d(k)) for k in args]) + tuple(kwds.items())
            try:
                inst = cache.get(key, None)
            except TypeError:  # Can't cache this set of arguments
                inst = key = None
            if inst is None:
                inst = klass(*args, **kwds)
                inst.__class__ = cls
                if key is not None:
                    cache[key] = inst
            return inst

        def __init__(self, *args, **kwds):
            pass

    return _decorated


def get_module(var):
    """Return the module of the variable"""
    return getattr(type(var), '__module__', '').split('.')[0]


class csr_matrix_alt(csr_matrix):
    """Subclass csr_matrix to overload dot operator for MX/SX classes and
    cvxpy classes"""
    def __init__(self, *args, **kwargs):
        csr_matrix.__init__(self, *args, **kwargs)

    def dot(self, other):
        if isinstance(other, (cas.MX, cas.SX)):
            # compatible with casadi 3.0 -- added by ruben
            return cas.mtimes(cas.DM(csr_matrix(self)), other)
            # NOT COMPATIBLE WITH CASADI 2.4
            # return cas.DMatrix(csr_matrix(self)).mul(other)
        elif get_module(other) in ['cvxpy', 'cvxopt']:
            return cvxopt.sparse(cvxopt.matrix(self.toarray())) * other
            # A = self.tocoo()
            # B = cvxopt.spmatrix(
            #     A.data, A.row.tolist(), A.col.tolist(), A.shape
            #     )
            # return B * other
        else:
            try:  # Scipy sparse matrix
                return super(csr_matrix_alt, self).dot(other)
            except:  # Regular numpy matrix
                return np.dot(self.toarray(), other)


class Basis(object):
    """A generic spline basis with a knot sequence and degree
    """
    def __init__(self, knots, degree):
        self.knots = np.array(knots)
        self.degree = degree
        self._x = np.linspace(knots[0], knots[-1], NO_POINTS)

    def __len__(self):
        return len(self.knots) - self.degree - 1

    def __call__(self, x):
        return self.eval_basis(x)

    def _ind(self, i, x):
        """Indicator function between knots[i] and knots[i + 1]
        """
        if i < self.degree + 1 and self.knots[0] == self.knots[i]:
            return (x >= self.knots[i]) * (x <= self.knots[i + 1])
        return (x > self.knots[i]) * (x <= self.knots[i + 1])

    def _combine(self, other, degree):
        """Combine two bases to a new basis of specified degree"""
        c_self = Counter(self.knots)
        c_other = Counter(other.knots)
        breaks = set(self.knots).union(other.knots)
        # Should be corrected!
        multiplicity = [max(c_self.get(b, -np.inf) + degree - self.degree,
                            c_other.get(b, -np.inf) + degree - other.degree)
                        for b in breaks]
        knots = sum([[b] * m for b, m in zip(breaks, multiplicity)], [])
        return self.__class__(sorted(knots), degree)

    def __add__(self, other):
        if isinstance(other, self.__class__):
            degree = max(self.degree, other.degree)
            return self._combine(other, degree)
        elif isinstance(other, float) or isinstance(other, int):
            return self
        else:
            raise TypeError("Not a basis error")

    __radd__ = __add__
    __sub__ = __add__
    __rsub__ = __sub__

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            degree = self.degree + other.degree
            return self._combine(other, degree)
        elif isinstance(other, float) or isinstance(other, int):
            return self
        else:
            raise TypeError("Not a basis error")

    __rmul__ = __mul__

    def __pow__(self, pow):
        if isinstance(pow, int):
            degree = pow * self.degree
            return self._combine(self, degree)
        else:
            raise TypeError("Power must be integer")

    def __eq__(self, other):
        return self.knots.shape == other.knots.shape and all(self.knots == other.knots) and self.degree == other.degree

    def __hash__(self):
        sh = [hash(self.degree)] + [hash(e) for e in self.knots]
        r = sh[0]
        for e in sh[1:]:
          r = r ^ e
        return r

    def insert_knots(self, knots):
        unique_knots = np.setdiff1d(knots, self.knots)
        knots = np.sort(np.append(self.knots, unique_knots))
        return self.__class__(knots, self.degree)

    def greville(self):
        """Return the Greville abscissae of the basis"""
        return [1. / self.degree * sum(self.knots[k + 1:k + self.degree + 1])
                for k in range(len(self))]

    def scale(self, factor, shift=0):
        # by default the domain is [0,1]
        # this function scales the basis domain and shifts it
        knots = self.knots*factor + shift
        return self.__class__(knots, self.degree)


@cached_class
class BSplineBasis(Basis):
    """
    A numerical Bspline basis
    """
    @memoize
    def eval_basis(self, x):
        """Evaluate the BSplineBasis at x.

        This function implements the Cox-de Boor formula for B-splines
        """
        x = np.array(x)
        k = self.knots
        basis = [[self._ind(i, x) * 1.0 for i in range(len(k) - 1)]]
        for d in range(1, self.degree + 1):
            basis.append([])
            for i in range(len(k) - d - 1):
                b = 0 * x
                bottom = k[i + d] - k[i]
                if bottom != 0:
                    b = (x - k[i]) * basis[d - 1][i] / bottom
                bottom = k[i + d + 1] - k[i + 1]
                if bottom != 0:
                    b += (k[i + d + 1] - x) * basis[d - 1][i + 1] / bottom
                basis[-1].append(b)
        # Consider sparse matrices?
        return csr_matrix_alt(np.c_[basis[-1]].T)

    def derivative(self, o=1):
        """Returns derivative of the basisfunctions

        Computes the derivative using eq. (16) in [de Boor, Chapter X, 2001].

        Args:
            x (numpy.array): grid on which to evaluate basisfunctions
            o (int): order of the derivative (default is 1)

        Returns:
            Numpy.array: columns contain the value of the derivative of the
                basisfunction evaluated at x
        """
        B = self.__class__(self.knots[o:-o], self.degree - o)
        P = np.eye(len(self))
        knots = self.knots
        for i in range(o):
            knots = knots[1:-1]
            delta_knots = knots[self.degree - i:] - knots[:- self.degree + i]
            T = np.zeros((len(self) - 1 - i, len(self) - i))
            j = np.arange(len(self) - 1 - i)
            T[(j, j)] = -1. / delta_knots
            T[(j, j + 1)] = 1. / delta_knots
            P = (self.degree - i) * np.dot(T, P)
        return B, csr_matrix_alt(P)

    def support(self):
        """Return a list of support intervals for each basis function"""
        return list(zip(
            self.knots[:-(self.degree + 1)],
            self.knots[(self.degree + 1):]
            ))

    def pairs(self, other):
        """Return which pairs remain when multiplying two bases"""
        def is_valid(a, b):
            """Return True if intervals a, b overlap"""
            return max(a[0], b[0]) < min(a[1], b[1])
        i_self = self.support()
        i_other = other.support()
        pairs = np.where([[is_valid(j, x) for x in i_other]
                          for j in i_self])
        # Additionaly build a selection matrix for the product
        S = np.zeros((len(self), len(self) * len(other)))
        # S[[pairs[0], pairs[0] * len(self) + pairs[1]]] = 1.
        return pairs, S

    def transform(self, other, TOL=1e-10):
        """Transformation from one basis to another.

        Returns a transformation matrix T such that

            self(x).T = other(x)

        TODO: Can we use the greville points instead of max?
        """
        b = self(self._x).toarray()
        m = np.argmax(b, axis=0)
        # x = np.linspace(self.knots[0], self.knots[-1], NO_POINTS)
        xmax = self._x[m]
        if isinstance(other, BSplineBasis):
            # if (self.knots[0] == other.knots[0] and
            #    self.knots[-1] == other.knots[-1]):
            T = la.solve(b[m, :], other(xmax).toarray())
        else:
            try:
                T = la.solve(b[m, :], other(xmax))
            except:  # In case of multiplication
                T = la.solve(b[m, :], other(m))
        T[abs(T) < TOL] = 0.
        return csr_matrix_alt(T)

    def as_poly(self):
        """Returns polynomial description of the basis functions"""
        k = self.knots
        k_min, k_max = min(self.knots), max(self.knots)
        basis = [[ppoly([a, b], [[1]]) for (a, b) in zip(k[:-1], k[1:])]]
        for d in range(1, self.degree + 1):
            basis.append([])
            for i in range(len(k) - d - 1):
                b = ppoly([k_min, k_max], [0])
                bottom = k[i + d] - k[i]
                if bottom != 0:
                    b += ppoly([k_min, k_max], [[-k[i], 1]]) * basis[d - 1][i] * (1. / bottom)
                bottom = k[i + d + 1] - k[i + 1]
                if bottom != 0:
                    b += ppoly([k_min, k_max], [[k[i + d + 1], -1]]) * basis[d - 1][i + 1] * (1. / bottom)
                basis[-1].append(b)
        return basis[-1]


class NurbsBasis(Basis):
    def __init__(self, knots, degree, weights):
        self.weights = weights
        self.bbasis = BSplineBasis(knots, degree)
        super(NurbsBasis, self).__init__(knots, degree)

    def eval_basis(self, x):
        B = self.bbasis(x)
        denom = B.dot(self.weights)
        if isinstance(self.weights, cas.MX):
            pass
            # B.dot(cas.diag(self.weights))
        else:
            return ((B.toarray() * self.weights).T / denom).T


class TSplineBasis(Basis):
    """A trigonometric spline basis"""
    def eval_basis(self, x):
        """
        Basisfunction of degree d evaluated on x
        """
        k = self.knots
        basis = [[self._ind(i, x) * 1.0 for i in range(len(k) - 1)]]
        for d in range(1, self.degree + 1):
            basis.append([])
            for i in range(len(k) - d - 1):
                b = 0 * x
                bottom = np.sin(0.5 * (k[i + d] - k[i]))
                if bottom != 0:
                    b = np.sin(0.5 * (x - k[i])) * basis[d - 1][i] / bottom
                bottom = np.sin(0.5 * (k[i + d + 1] - k[i + 1]))
                if bottom != 0:
                    b += np.sin(0.5 * (k[i + d + 1] - x)) * basis[d - 1][i + 1] / bottom
                basis[-1].append(b)
        return csr_matrix_alt(np.c_[basis[-1]].T)


class Spline(object):
    def __init__(self, basis, coeffs):
        # self.coeffs = np.array(coeffs).ravel()
        self.coeffs = coeffs
        self.basis = basis
        # if isinstance(coeffs, (cas.SXMatrix, cas.SX)):
        #     self.basis._basis = cas.DMatrix(self.basis._basis)

    def __call__(self, x):
        return self.basis(x).dot(self.coeffs)

    def __len__(self):
        return len(self.basis)

    def __eq__(self, other):
        return (self.basis == other.basis and
                type(self.coeffs) == type(other.coeffs) and
                all(self.coeffs == other.coeffs))

    def __hash__(self):
        sh = [hash(self.basis)] + [hash(e) for e in self.coeffs]
        r = sh[0]
        for e in sh[1:]:
          r = r ^ e
        return r


class BSpline(Spline):
    """Construct a Bspline curve from the basis B and coefficients c
    """
    def __add__(self, other):
        if isinstance(other, self.__class__):
            basis = self.basis + other.basis
            coeffs = (basis.transform(self.basis).dot(self.coeffs) +
                      basis.transform(other.basis).dot(other.coeffs))
        else:
            try:
                basis = self.basis
                coeffs = self.coeffs + other  # Only for BSpline!
            except:
                NotImplementedError("Incompatible datatype")
        return self.__class__(basis, coeffs)

    __radd__ = __add__

    def __neg__(self):
        return self.__class__(self.basis, -self.coeffs)

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return other + (-self)

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            basis = self.basis * other.basis
            pairs, S = self.basis.pairs(other.basis)
            b_self = self.basis(basis._x)
            b_other = other.basis(basis._x)
            basis_product = b_self[:, pairs[0]].multiply(b_other[:, pairs[1]])
            T = basis.transform(lambda y: basis_product.toarray()[y, :])
            try:
                coeffs_product = (self.coeffs[pairs[0].tolist()] *
                                  other.coeffs[pairs[1].tolist()])
            except:  # cvxopt, cvxpy, assuming other.coeffs is not a variable
                S = np.zeros((len(pairs[0]), len(self)))
                S[[list(range(len(pairs[0]))), pairs[0]]] = 1.
                S = cvxopt.matrix(S)
                coeffs_product = cvxopt.spdiag(other.coeffs[pairs[1].tolist()]) * S * self.coeffs
                # coeffs_product = cp.vstack(*[self.coeffs[p0] * other.coeffs[p1] for (p0, p1) in zip(*pairs)])
            return self.__class__(basis, T.dot(coeffs_product))
        else:
            try:
                basis = self.basis
                coeffs = other * self.coeffs
                return self.__class__(basis, coeffs)
            except:
                NotImplementedError("Incompatible datatype")

    def __rmul__(self, other):
        return self.__mul__(other)

    def __pow__(self, power):
        """A naive implementation of the power function..."""
        if isinstance(power, int):
            a = self
            for i in range(1, power):
                a *= self
            return a
        else:
            TypeError("Exponent must be integer")

    def __div__(self, other):
        basis = self.basis + other.basis
        weights = basis.transform(other.basis).dot(other.coeffs)
        coeffs = basis.transform(self.basis).dot(self.coeffs) / weights
        return Nurbs(NurbsBasis(basis.knots, basis.degree, weights), coeffs)

    def derivative(self, o=1):
        if o == 0:
            return self
        else:
            Bd, Pd = self.basis.derivative(o=o)
            return self.__class__(Bd, Pd.dot(self.coeffs))

    def insert_knots(self, knots):
        """Returns an equivalent spline with knot insertion"""
        basis = self.basis.insert_knots(knots)
        coeffs = basis.transform(self.basis).dot(self.coeffs)
        return self.__class__(basis, coeffs)

    def integral(self):
        """Returns the value of the integral over the support.

        This is a literal implementation of formula X.33 from deBoor and
        assumes that at x = knots[-1], only the last basis function is active
        """
        knots = self.basis.knots
        coeffs = self.coeffs
        d = self.basis.degree
        K = csr_matrix_alt(np.diag((knots[d + 1:] - knots[:-(d + 1)]) / (d + 1)))
        return sum(K.dot(coeffs))
        # try:
        #     return sum(coeffs * (knots[d + 1:] - knots[:-(d + 1)])) / (d + 1)

    def roots(self):
        """Return the roots of the B-spline

        Algorithm:
        * Determine polynomial description
        * Determine roots of each polynomial subpiece
        * Check if root is in the support of the polynomial piece
        """
        basis = self.basis.as_poly()
        spline = np.sum([self.coeffs[i] * basis[i] for i in range(len(basis))])
        roots = []
        for i, f in enumerate(spline.functions):
            root = f.roots()
            roots.extend([r for r in root
                         if spline.knots[i] <= r < spline.knots[i + 1]])
        return roots

    def scale(self, factor, shift=0):
        # by default the domain is [0,1]
        # this function scales the domain of the spline and shifts it
        basis = self.basis.scale(factor, shift=shift)
        return self.__class__(basis, self.coeffs)


class Nurbs(Spline):
    def __init__(self, basis, coeffs):
        super(Nurbs, self).__init__(basis, coeffs)
        self.num = BSpline(self.basis.bbasis, self.coeffs * self.basis.weights)
        self.denom = BSpline(self.basis.bbasis, self.basis.weights)

    def __mul__(self, other):
        num = self.num * other
        return num / self.denom

    __rmul__ = __mul__

    def __add__(self, other):
        if isinstance(other, Nurbs):
            return (self.num * other.denom + self.denom * other.num) / (self.denom * other.denom)

    def derivative(self, o=1):
        """Derivative of a Nurbs"""
        if o == 1:
            b = self.basis.bbasis
            db = b.derivative()[0]
            # The denominator
            denom2 = self.denom ** 2
            # coeffs of the numerator
            pairs, S = b.pairs(db)
            dnum = self.num.derivative()
            ddenom = self.denom.derivative()
            coeffs_product = dnum.coeffs[pairs[1].tolist()] * self.denom.coeffs[pairs[0].tolist()] - self.num.coeffs[pairs[0].tolist()] * ddenom.coeffs[pairs[1].tolist()]
            bx = b(denom2.basis._x)
            dbx = db(denom2.basis._x)
            basis_product = bx[:, pairs[0]].multiply(dbx[:, pairs[1]])
            T = denom2.basis.transform(lambda y: basis_product.toarray()[y, :])
            coeffs = T.dot(coeffs_product) / denom2.coeffs
            basis = NurbsBasis(denom2.basis.knots, denom2.basis.degree, denom2.coeffs)
            return self.__class__(basis, coeffs)
            # num = BSpline(self.basis.bbasis, self.coeffs * self.basis.weights)
            # denom = BSpline(self.basis.bbasis, self.basis.weights)
            # Compute numerator efficiently
            # return ((dnum * denom - num * ddenom) /  # This numerator has twice the same basis!
            #         denom ** 2)  # Can we simplify this, make it faster? -> The same basis is created multiple times!
        else:
            return self.derivative().derivative(o=o-1)

    def insert_knots(self, knots):
        """Returns an equivalent spline with knot insertion"""
        b = self.basis.bbasis.insert_knots(knots)
        weights = b.transform(self.basis.bbasis).dot(self.basis.weights)
        coeffs = b.transform(self.basis.bbasis).dot(self.coeffs)
        basis = NurbsBasis(b.knots, b.degree, weights)
        return self.__class__(basis, coeffs)


class TensorBSpline(object):
    """A multidimensional spline"""
    def __init__(self, basis, coeffs, var):
        self.basis = tuple(basis)
        self.var = tuple(var)
        self.coeffs = coeffs

    # def _reduce(self):
    #     """Set irrelevant coefficients to zero"""
    #     def is_valid(a, b):
    #         """Return True if intervals a, b overlap"""
    #         return max(a[0], b[0]) < min(a[1], b[1])
    #     i = [zip(b.knots[:-(b.degree + 1)], b.knots[b.degree + 1:])
    #          for b in self.basis]
    #     map(lambda x: is_valid(j, x), ii) for j in i[0]
    #     pairs = np.where([map(lambda x: is_valid(j, x), i_other)
    #                       for j in i_self])

    def dims(self):
        """The number of dimensions of the spline"""
        return len(self.basis)

    def __call__(self, x):
        """Evaluate TensorBSpline
        There still seems to be something wrong here...
        """
        s = np.inner(self.basis[-1](x[-1]).toarray(), self.coeffs)
        for i in reversed(list(range(self.dims() - 1))):
            s = np.inner(self.basis[i](x[i]).toarray(), s)
        return s

    def __add__(self, other):
        if isinstance(other, TensorBSpline) and other.var == self.var:
            if self.dims() == 2 and get_module(self.coeffs) in ['cvxpy', 'cvxopt']:
                basis = list(map(lambda x, y: x + y, self.basis, other.basis))
                Tself = list(map(lambda x, y: cvxopt.matrix(x.transform(y).toarray()), basis, self.basis))
                Tother = list(map(lambda x, y: cvxopt.matrix(x.transform(y).toarray()), basis, other.basis))
                cself = Tself[0] * self.coeffs * Tself[1].T
                cother = Tother[0] * other.coeffs * Tother[1].T
                coeffs = cself + cother
            else:
                basis = list(map(lambda x, y: x + y, self.basis, other.basis))
                Tself = list(map(lambda x, y: x.transform(y).toarray(), basis, self.basis))
                Tother = list(map(lambda x, y: x.transform(y).toarray(), basis, other.basis))
                cself = self.coeffs
                for i in range(self.dims()):
                    cself = np.tensordot(Tself[i], cself.swapaxes(0, i), axes=[1, 0]).swapaxes(0, i)
                cother = other.coeffs
                for i in range(other.dims()):
                    cother = np.tensordot(Tother[i], cother.swapaxes(0, i), axes=[1, 0]).swapaxes(0, i)
                coeffs = cself + cother
        else:
            try:
                basis = self.basis
                coeffs = self.coeffs + other  # Only for BSpline!
            except:
                NotImplementedError("Incompatible datatype")
        return self.__class__(basis, coeffs, self.var)

    __radd__ = __add__

    def __neg__(self):
        return self.__class__(self.basis, -self.coeffs, self.var)

    def __sub__(self, other):
        return self + (-other)

    __rsub__ = __sub__

    def __mul__(self, other):
        if isinstance(other, TensorBSpline):
            if len(self.basis) > 2:
                return NotImplementedError("Too complex to implement :-)")
            basis = list(map(lambda x, y: x * y, self.basis, other.basis))
            pairs = list(map(lambda x, y: x.pairs(y)[0], self.basis, other.basis))
            basis_product = list(map(lambda x, y, p, b: x(b._x)[:, p[0]].multiply(y(b._x)[:, p[1]]), self.basis, other.basis, pairs, basis))
            coeffs_product = (self.coeffs[pairs[0][0]].T[pairs[1][0]] *
                              other.coeffs[pairs[0][1]].T[pairs[1][1]])
            T = list(map(lambda x, b: b.transform(lambda y: x.toarray()[y, :]), basis_product, basis))
            coeffs = coeffs_product.T
            for i, t in enumerate(T):
                coeffs = np.tensordot(t.toarray(), coeffs.swapaxes(0, i), axes=[1, 0]).swapaxes(0, i)
            return self.__class__(basis, coeffs, self.var)
            return self.coeffs[pairs[0][0].tolist(), pairs[1][0].tolist()] * other.coeffs[pairs[0][1].tolist(), pairs[1][1].tolist()]
            # coeffs_product = np.kron(self.coeffs, other.coeffs)

        else:
            try:
                basis = self.basis
                coeffs = self.coeffs * other
            except:
                NotImplementedError("Incompatible datatype")
        return self.__class__(basis, coeffs, self.var)

    __rmul__ = __mul__

    def integral(self):
        """Returns the value of the integral over the support.

        This is a literal implementation of formula X.33 from deBoor and
        assumes that at x = knots[-1], only the last basis function is active
        """
        knots = [b.knots for b in self.basis]
        coeffs = self.coeffs
        deg = [b.degree for b in self.basis]
        K = [(k[d + 1:] - k[:-(d + 1)]) / (d + 1)
             for (k, d) in zip(knots, deg)]
        if self.dims() == 2: #get_module(self.coeffs) in ['cvxpy', 'cvxopt']:
            i = cvxopt.matrix(K[0]).T * self.coeffs * cvxopt.matrix(K[1])
            return i
        i = np.inner(K[-1], coeffs)
        for ki in K[:-1]:
            i = np.inner(ki, i)
        return i
