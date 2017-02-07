import meco_binaries; meco_binaries(cpp_splines='master')
import Basis as spl
import spline_old as spl_old
import spline_extra as spl_ex
import numpy as np
from scipy.interpolate import splev
from casadi import SX, MX, mtimes, Function


""" helper functions -> should vanish """
""" ================================= """


def old2new(spline_old):
    basis = spl.BSplineBasis(spline_old.basis.knots, spline_old.basis.degree)
    coeffs = spline_old.coeffs
    return spl.Function(basis, coeffs)


def new2old(spline_new):
    basis = spl_old.BSplineBasis(spline_new.getBasis().getKnots(), spline_new.getBasis().getDegree())
    coeffs = spline_new.getCoefficient().getData()
    return spl_old.BSpline(basis, coeffs)


def new2oldbasis(basis_new):
    return spl_old.BSplineBasis(basis_new.getKnots(), basis_new.getDegree())


""" methods desired to implement in spline toolbox -> should vanish """
""" =============================================================== """


def __add__(self, other):
    if isinstance(other, self.__class__):
        return old2new(new2old(self).__add__(new2old(other)))
    else:
        return old2new(new2old(self).__add__(other))


def __neg__(self):
    return old2new(new2old(self).__neg__())


def __sub__(self, other):
    return self + (-other)


def __rsub__(self, other):
    return other + (-self)


def __mul__(self, other):
    if isinstance(other, self.__class__):
        return old2new(new2old(self).__mul__(new2old(other)))
    else:
        return old2new(new2old(self).__mul__(other))


def __rmul__(self, other):
    return self.__mul__(other)


def __pow__(self, power):
    return old2new(new2old(self).__pow__(power))


def derivative(self, o=1):
    return old2new(new2old(self).derivative(o))


def antiderivative(self):
    return old2new(spl_ex.running_integral(new2old(self)))


def integral(self, lb=None, ub=None):
    knots = self.getBasis().getKnots()
    if lb is None:
        lb = knots[0]
    if ub is None:
        ub = knots[-1]
    return spl_ex.definite_integral(new2old(self), lb, ub)


def crop(self, lb, ub):
    return old2new(spl_ex.crop(new2old(self), lb, ub))


def extrapolate(self, t_extra, m=None):
    return old2new(spl_ex.extrapolate(new2old(self), t_extra, m))


def insert_knots(self, knots):
    return old2new(spl_ex.insert_knots(new2old(self), knots))


def get_crop_tf(basis, lb, ub):
    return spl_ex.crop_T(new2oldbasis(basis), lb, ub)


def get_extrapolate_tf(basis, t_extra, m=None):
    return spl_ex.extrapolate_T(new2oldbasis(basis), t_extra, m)


def get_insert_knots_tf(basis, knots):
    return spl_ex.knot_insertion_T(new2oldbasis(basis), knots)


def transform(basis, other_basis):
    return new2oldbasis(basis).transform(new2oldbasis(other_basis))


spl.Function.__add__ = __add__
spl.Function.__radd__ = __add__
spl.Function.__neg__ = __neg__
spl.Function.__sub__ = __sub__
spl.Function.__rsub__ = __rsub__
spl.Function.__mul__ = __mul__
spl.Function.__rmul__ = __rmul__
spl.Function.__pow__ = __pow__
spl.Function.derivative = derivative
spl.Function.antiderivative = antiderivative
spl.Function.integral = integral
spl.Function.crop = crop

spl.BSplineBasis.get_crop_tf = get_crop_tf
spl.BSplineBasis.get_extrapolate_tf = get_extrapolate_tf
spl.BSplineBasis.transform = transform


""" methods that should stay: extensions on spline toolbox, but too specific for integration """
""" ========================================================================================== """


def concat_splines(segments, segment_times):
    spl0 = segments[0]
    knots = [np.r_[s.getBasis().getKnots()]*segment_times[0] for s in spl0]
    degree = [s.getBasis().getDegree() for s in spl0]
    coeffs = [s.getCoefficient().getData() for s in spl0]
    for k in range(1, len(segments)):
        for l, s in enumerate(segments[k]):
            if s.getBasis().getDegree() != degree[l]:
                raise ValueError(
                    'Splines at index ' + l + 'should have same degree.')
            knots[l] = np.r_[knots[l], s.basis.knots[degree[l]+1:]*segment_times[k] + knots[l][-1]]
            coeffs[l] = np.r_[coeffs[l], s.coeffs]
    bases = [spl.BSplineBasis(knots[l], degree[l])
             for l in range(len(segments[0]))]
    return [spl.Function(bases[l], coeffs[l]) for l in range(len(segments[0]))]


def sample_splines(spline, time):
    if isinstance(spline, list):
        return [splev(time, (s.getBasis().getKnots(), s.getCoefficient().getData(), s.getBasis().getDegree())) for s in spline]
    else:
        return splev(time, (spline.getBasis().getKnots(), spline.getCoefficient().getData(), spline.getBasis().getDegree()))


def get_shiftoverknot_tf(basis):
    knots = basis.getKnots()
    deg = basis.getDegree()
    N = basis.getLength()
    # number of basis fun discarted by cropping
    n = len(np.where(np.r_[knots] == knots[deg+1])[0])
    # number of basis fun added by extrapolation
    m = 1
    while knots[-deg-2-m] >= knots[-deg-2]:
        m += 1
    T_crp, _ = basis.get_crop_tf(knots[deg+1], knots[-1])
    T_extr, _ = basis.get_extrapolate_tf(knots[-1] - knots[-deg-2])
    T = np.zeros((N + m - n, N))
    T[:N-n, :] = T_crp
    T[-(deg+1):, -(deg+1):] = T_extr[-(deg+1):, -(deg+1):]
    knots2 = np.r_[knots[deg+n]*np.ones(deg+1), knots[deg+n+1:-deg-1], knots[-deg-1]*np.ones(m), (knots[-1]+(knots[-1] - knots[-deg-2]))*np.ones(deg+1)]
    return T, knots2


def shiftoverknot(self):
    T, knots = self.getBasis().get_shiftoverknot_tf()
    coeffs = self.getCoefficient().getData()
    if isinstance(coeffs, (SX, MX)):
        coeffs2 = mtimes(T, coeffs)
    else:
        coeffs2 = T.dot(coeffs)
    basis2 = spl.BSplineBasis(knots, self.getBasis().getDegree())
    return spl.Function(basis2, coeffs2)


def get_shiftfirstknot_tf(basis, t_shift, inverse=False):
    return spl_ex.shiftfirstknot_T(new2oldbasis(basis), t_shift, inverse)


def shiftfirstknot_fwd(coeffs, basis, t_shift):
    if isinstance(coeffs, (SX, MX)):
        cfs_sym = MX.sym('cfs', coeffs.shape)
        t_shift_sym = MX.sym('t_shift')
        T = basis.get_shiftfirstknot_tf(t_shift_sym)
        cfs2_sym = mtimes(T, cfs_sym)
        fun = Function('fun', [cfs_sym, t_shift_sym], [cfs2_sym]).expand()
        coeffs2 = fun(coeffs, t_shift)
    else:
        T = basis.get_shiftfirstknot_tf(t_shift)
        coeffs2 = T.dot(coeffs)
    return coeffs2


def shiftfirstknot_bwd(coeffs, basis, t_shift):
    if isinstance(coeffs, (SX, MX)):
        cfs_sym = MX.sym('cfs', coeffs.shape)
        t_shift_sym = MX.sym('t_shift')
        _, Tinv = basis.get_shiftfirstknot_tf(t_shift_sym, inverse=True)
        cfs2_sym = mtimes(Tinv, cfs_sym)
        fun = Function('fun', [cfs_sym, t_shift_sym], [cfs2_sym]).expand()
        coeffs2 = fun(coeffs, t_shift)
    else:
        _, Tinv = basis.get_shiftfirstknot_tf(t_shift, inverse=True)
        coeffs2 = Tinv.dot(coeffs)
    return coeffs2


def get_crop_inexact_tf(basis, min_value, max_value):
    degree = basis.getDegree()
    n_knots = basis.getLength() - degree + 1
    knots2 = np.r_[min_value*np.ones(degree), np.linspace(min_value, max_value, n_knots), max_value*np.ones(degree)]
    basis2 = spl.BSplineBasis(knots2, degree)
    T = basis2.transform(basis)
    return T, knots2


def crop_inexact(self, min_value, max_value):
    # Extract spline piece in [t_shift, T] and express it in an equidistant
    # basis. This is not exact as de knot positions change.
    basis = self.getBasis()
    coeffs = self.getCoefficient().getData()
    T, knots = basis.get_crop_inexact_tf(min_value, max_value)
    if isinstance(coeffs, (SX, MX)):
        coeffs2 = mtimes(T, coeffs)
    else:
        coeffs2 = T.dot(coeffs)
    basis2 = spl.BSplineBasis(knots, basis.getDegree())
    return spl.Function(basis2, coeffs2)


spl.BSplineBasis.get_shiftoverknot_tf = get_shiftoverknot_tf
spl.BSplineBasis.get_shiftfirstknot_tf = get_shiftfirstknot_tf
spl.BSplineBasis.get_crop_inexact_tf = get_crop_inexact_tf

spl.Function.shiftoverknot = shiftoverknot
spl.Function.crop_inexact = crop_inexact
