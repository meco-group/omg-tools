import meco_binaries; meco_binaries(cpp_splines='master')
import Basis as spl
import spline as spl_old
import spline_extra as spl_ex
# from omgtools.basics.spline_extra import *
import numpy as np

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

def integrate(self):
    return old2new(spl_ex.running_integral(new2old(self)))

def integral(self, lb=None, ub=None):
    knots = self.getBasis().getKnots()
    if lb is None:
        lb = knots[0]
    if ub is None:
        ub = knots[-1]
    return spl_ex.definite_integral(new2old(self), lb, ub)



spl.Function.__add__ = __add__
spl.Function.__radd__ = __add__
spl.Function.__neg__ = __neg__
spl.Function.__sub__ = __sub__
spl.Function.__rsub__ = __rsub__
spl.Function.__mul__ = __mul__
spl.Function.__rmul__ = __rmul__
spl.Function.__pow__ = __pow__

spl.Function.derivative = derivative
spl.Function.integrate = integrate
spl.Function.integral = integral

def sample_splines(spline, time):
    if isinstance(spline, list):
        return spl_ex.sample_splines([new2old(s) for s in spline], time)
    else:
        return spl_ex.sample_splines(new2old(spline), time)

def concat_splines(segments, segment_times):
    segments_old = [[new2old(spl) for spl in segment] for segment in segments]
    splines_old = spl_ex.concat_splines(segments_old, segment_times)
    return [old2new(spl_old) for spl_old in splines_old]

def shiftoverknot(spline):
    return old2new(spl_ex.shiftoverknot(new2old(spline)))

def shiftoverknot_T(basis):
    return spl_ex.shiftoverknot_T(new2oldbasis(basis))

def shift_spline(coeffs, t_shift, basis):
    return spl_ex.shift_spline(coeffs, t_shift, new2oldbasis(basis))

def shift_knot1_fwd(cfs, basis, t_shift):
    return spl_ex.shift_knot1_fwd(cfs, new2oldbasis(basis), t_shift)

def shift_knot1_bwd(cfs, basis, t_shift):
    return spl_ex.shift_knot1_bwd(cfs, new2oldbasis(basis), t_shift)
