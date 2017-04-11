try:
    import meco_binaries; meco_binaries(cpp_splines='develop')
except ImportError:
    print "meco_binaries not found."
import splines as spl
import numpy as np
from scipy.interpolate import splev
from casadi import SX, MX, DM, Function


""" extra BSplineBasis functions """
""" ============================ """


def crop_basis(basis, lb, ub):
    knots = basis.knots()
    degree = basis.degree()
    n_min = len(np.where(np.array(knots) == lb)[0])
    n_max = len(np.where(np.array(knots) == ub)[0])
    min_knots = [lb]*(degree + 1 - n_min)
    max_knots = [ub]*(degree + 1 - n_max)
    b2, T = basis.insert_knots(min_knots+max_knots)
    knots2 = b2.knots()
    jmin = np.searchsorted(knots2, lb, side='left')
    jmax = np.searchsorted(knots2, ub, side='right')
    return spl.BSplineBasis(knots2[jmin:jmax], degree), T[jmin:jmax-degree-1, :]


def extrapolate_basis(basis, t_extra, m=None):
    if m is None:
        m = 1
    knots = basis.knots()
    deg = basis.degree()
    b2, Tex = basis.kick_boundary([knots[0], knots[-1]+t_extra])
    b3, Tin = b2.insert_knots([knots[-1]]*m)
    return b3, Tin.dot(Tex)


def shiftoverknot_basis(basis):
    knots = basis.knots()
    deg = basis.degree()
    N = basis.dimension()
    # number of basis fun discarted by cropping
    n = len(np.where(np.r_[knots] == knots[deg+1])[0])
    # number of basis fun added by extrapolation
    m = 1
    while knots[-deg-2-m] >= knots[-deg-2]:
        m += 1
    _, T_crp = basis.crop(knots[deg+1], knots[-1])
    _, T_extr = basis.extrapolate(knots[-1] - knots[-deg-2])
    T = np.zeros((N + m - n, N))
    T[:N-n, :] = T_crp
    T[-(deg+1):, -(deg+1):] = T_extr[-(deg+1):, -(deg+1):]
    knots2 = np.r_[knots[deg+n]*np.ones(deg+1), knots[deg+n+1:-deg-1], knots[-deg-1]*np.ones(m), (knots[-1]+(knots[-1] - knots[-deg-2]))*np.ones(deg+1)]
    return spl.BSplineBasis(knots2, deg), T


def shiftfirstknot_basis(basis, t_shift):
    degree = basis.degree()
    knots_ins = [t_shift]*(degree+1)
    b2, T = basis.insert_knots(knots_ins)
    knots2 = b2.knots()
    return spl.BSplineBasis(knots2[degree+1:], degree), T[degree+1:, :]


# def shiftfirstknot_basis(basis, t_shift):
#     knots = basis.knots()
#     b2, T =  basis.kick_boundary([t_shift, knots[-1]])
#     return b2, T


def crop_inexact_basis(basis, lb, ub):
    degree = basis.degree()
    n_knots = basis.dimension() - degree + 1
    knots2 = np.r_[lb*np.ones(degree), np.linspace(lb, ub, n_knots), ub*np.ones(degree)]
    basis2 = spl.BSplineBasis(knots2, degree)
    T = basis.project_to(basis2)
    return basis2, T

spl.BSplineBasis.crop = crop_basis
spl.Basis.extrapolate = extrapolate_basis
spl.BSplineBasis.shiftoverknot = shiftoverknot_basis
spl.BSplineBasis.shiftfirstknot = shiftfirstknot_basis
spl.Basis.crop_inexact = crop_inexact_basis

""" extra BSpline functions """
""" ======================= """


def crop(self, lb, ub):
    b2, T = self.basis().crop(lb, ub)
    coeff2 = self.coeff().transform(T)
    return spl.Function(b2, coeff2)


def extrapolate(self, t_extra, m=None):
    b2, T = self.basis().extrapolate(t_extra, m)
    coeff2 = self.coeff().transform(T)
    return spl.Function(b2, coeff2)


def shiftoverknot(self):
    b2, T = self.basis().shiftoverknot()
    coeff2 = self.coeff().transform(T)
    return spl.Function(b2, coeff2)


def shiftfirstknot_fwd(self, t_shift):
    b2, T = self.basis().shiftfirstknot(t_shift)
    coeffs = self.data()
    if False and isinstance(coeffs, (SX, MX)):
        cfs_sym = MX.sym('cfs', coeffs.shape)
        t_shift_sym = MX.sym('t_shift')
        b2, T = self.basis().shiftfirstknot(t_shift_sym)
        cfs2_sym = spl.Coefficient(cfs_sym).transform(T)
        fun = Function('fun', [cfs_sym, t_shift_sym], [cfs2_sym])
        coeffs2 = fun(coeffs, t_shift)
    else:
        coeffs2 = self.coeff().transform(T)
    return spl.Function(b2, coeffs2)


def shiftfirstknot_bwd(self, t_shift):
    deg = self.basis().degree()
    coeffs = self.data()
    if False and isinstance(coeffs, (SX, MX)):
        cfs_sym = MX.sym('cfs', coeffs.shape)
        t_shift_sym = MX.sym('t_shift')
        b2, T = self.basis().shiftfirstknot(t_shift_sym)
        Tinv = DM.eye(self.basis().dimension())
        for i in range(deg, -1, -1):
            Tinv[i, i] = 1./T[i, i]
            for j in range(deg, i, -1):
                Tinv[i, j] = (-1./T[i, i])*sum([T[i, k]*Tinv[k, j]
                                                for k in range(i+1, deg+2)])
        cfs2_sym = spl.Coefficient(cfs_sym).transform(Tinv)
        fun = Function('fun', [cfs_sym, t_shift_sym], [cfs2_sym])
        coeffs2 = fun(coeffs, t_shift)
    else:
        b2, T = self.basis().shiftfirstknot(t_shift)
        # T is upper triangular: easy inverse
        Tinv = MX.eye(self.basis().dimension())
        for i in range(deg, -1, -1):
            Tinv[i, i] = 1./T[i, i]
            for j in range(deg, i, -1):
                Tinv[i, j] = (-1./T[i, i])*sum([T[i, k]*Tinv[k, j]
                                                for k in range(i+1, deg+2)])
        coeffs2 = self.coeff().transform(Tinv)
    return spl.Function(b2, coeffs2)


def crop_inexact(self, lb, ub):
    # Extract spline piece in [t_shift, T] and express it in an equidistant
    # basis. This is not exact as de knot positions change.
    degree = self.basis().degree()
    n_knots = self.basis().dimension() - degree + 1
    knots2 = np.r_[lb*np.ones(degree), np.linspace(lb, ub, n_knots), ub*np.ones(degree)]
    basis2 = spl.BSplineBasis(knots2, degree)
    ret = self.project_to(basis2)
    return ret

spl.Function.crop = crop
spl.Function.extrapolate = extrapolate
spl.Function.shiftoverknot = shiftoverknot
spl.Function.shiftfirstknot_fwd = shiftfirstknot_fwd
spl.Function.shiftfirstknot_bwd = shiftfirstknot_bwd
spl.Function.crop_inexact = crop_inexact


""" other spline-related functions """
""" ============================== """


def concat_splines(segments, segment_times):
    spl0 = segments[0]
    knots = [np.r_[s.basis().knots()]*segment_times[0] for s in spl0]
    degree = [s.basis().degree() for s in spl0]
    coeffs = [s.data() for s in spl0]
    for k in range(1, len(segments)):
        for l, s in enumerate(segments[k]):
            if s.basis().degree() != degree[l]:
                raise ValueError(
                    'Splines at index ' + l + 'should have same degree.')
            knots[l] = np.r_[knots[l], s.basis.knots[degree[l]+1:]*segment_times[k] + knots[l][-1]]
            coeffs[l] = np.r_[coeffs[l], s.coeffs]
    bases = [spl.BSplineBasis(knots[l], degree[l])
             for l in range(len(segments[0]))]
    return [spl.Function(bases[l], coeffs[l]) for l in range(len(segments[0]))]


def sample_splines(spline, time):
    if isinstance(spline, list):
        return [splev(time, (s.basis().knots(), s.data(), s.basis().degree())) for s in spline]
    else:
        return splev(time, (spline.basis().knots(), spline.data(), spline.basis().degree()))
