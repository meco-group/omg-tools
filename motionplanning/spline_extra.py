from spline import BSpline, BSplineBasis
from casadi import SX, MX, mul, SXFunction, MXFunction
import numpy as np


def evalspline(s, x):
    # Evaluate spline with symbolic variable
    # This is possible not the best way to implement this. The conditional node
    # from casadi should be considered
    Bl = s.basis
    coeffs = s.coeffs
    k = Bl.knots
    basis = [[]]
    for i in range(len(k) - 1):
        if i < Bl.degree + 1 and Bl.knots[0] == Bl.knots[i]:
            basis[-1].append((x >= Bl.knots[i])*(x <= Bl.knots[i + 1]))
        else:
            basis[-1].append((x > Bl.knots[i])*(x <= Bl.knots[i + 1]))
    for d in range(1, Bl.degree + 1):
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
    result = 0.
    for l in range(len(Bl)):
        result += mul(coeffs[l], basis[-1][l])
    return result


def running_integral(spline):
    # Compute running integral from spline
    basis = spline.basis
    coeffs = spline.coeffs
    knots = basis.knots
    degree = basis.degree

    knots_int = np.r_[knots[0], knots, knots[-1]]
    degree_int = degree + 1
    basis_int = BSplineBasis(knots_int, degree_int)
    coeffs_int = [0.]
    for i in range(len(basis_int)-1):
        coeffs_int.append(coeffs_int[i]+(knots[degree+i+1]-knots[i])*coeffs[i])
    coeffs_int = [(1./degree_int)*c for c in coeffs_int]
    spline_int = BSpline(basis_int, coeffs_int)
    return spline_int


def definite_integral(spline, a, b):
    # Compute definite integral of spline in interval [a, b]
    spline_int = running_integral(spline)
    int_0a = evalspline(spline_int, a)
    int_0b = evalspline(spline_int, b)
    int_ab = int_0b - int_0a
    return int_ab


def shift_spline(coeffs, t_shift, basis):
    # Extract spline piece in [t_shift, T] and express it in an equidistant
    # basis. This is not exact as de knot positions change.
    n_knots = len(basis) - basis.degree + 1
    knots = basis.knots
    degree = basis.degree
    knots2 = np.r_[t_shift*np.ones(degree),
                   np.linspace(t_shift, knots[-1], n_knots),
                   knots[-1]*np.ones(degree)]
    basis2 = BSplineBasis(knots2, degree)
    T_tf = basis2.transform(basis)
    return T_tf.dot(coeffs)


def extrapolate(coeffs, t_extra, basis):
    T = extrapolate_T(basis, t_extra)
    return T.dot(coeffs)


def extrapolate_T(basis, t_extra):
    # Create transformation matrix that extrapolates the spline over an extra
    # knot interval of t_extra long.
    knots = basis.knots
    deg = basis.degree
    N = len(basis)
    knots2 = np.r_[knots[:-deg], (knots[-1]+t_extra)*np.ones(deg+1)]
    basis2 = BSplineBasis(knots2, deg)
    B = basis.eval_basis(knots[-2*deg:-deg]).toarray()[:, -deg:]
    A = basis2.eval_basis(knots[-2*deg:-deg]).toarray()[:, -deg-1:-1]
    _T = np.linalg.solve(A, B)
    _T[abs(_T) < 1e-10] = 0.
    T = np.zeros((N+1, N))
    T[:N-deg,:N-deg] = np.eye(N-deg)
    T[N-deg:-1, N-deg:] = _T
    # last row of T such that degree th derivative is continuous
    a1, a2 = np.identity(deg+1), np.identity(deg+1)
    for i in range(1, deg+1):
        a1_tmp = np.zeros((deg+1-i, deg+1-i+1))
        a2_tmp = np.zeros((deg+1-i, deg+1-i+1))
        for j in range(deg+1-i):
            a1_tmp[j, j] = -(deg+1-i)/(knots[j+N] - knots[j+N-deg-1+i])
            a1_tmp[j, j+1] = (deg+1-i)/(knots[j+N] - knots[j+N-deg-1+i])
            a2_tmp[j, j] = -(deg+1-i)/(knots2[j+N+1] - knots2[j+N-deg+i])
            a2_tmp[j, j+1] = (deg+1-i)/(knots2[j+N+1] - knots2[j+N-deg+i])
        a1, a2 = a1_tmp.dot(a1), a2_tmp.dot(a2)
    T[-1, -deg-1] = a1[:, 0]/a2[:, -1]
    T[-1, -deg:] = (a1[:, 1:]-a2[:, :-1].dot(_T))/a2[:, -1]
    return T


def shift_over_knot(coeffs, basis):
    T = shiftoverknot_T(basis)
    return T.dot(coeffs)


def shiftoverknot_T(basis):
    # Create transformation matrix that moves the horizon to
    # [knot[degree+1], T+knots[-1]-knots[-deg-2]]. The spline is extrapolated
    # in the interval the spline over an extra knot interval
    # [T, T+knot[N_shift]] of t_extra long. Move horizon to
    # [knot[N_shift], T+knot[N_shift]]. The spline is extrapolated in last
    # interval.
    knots = basis.knots
    deg = basis.degree
    t_shift = knots[deg+1] - knots[0]
    T = np.diag(np.ones(len(basis)-1), 1)
    _T = np.eye(deg+1)
    for k in range(deg):
        _t = np.zeros((deg+1+k+1, deg+1+k))
        for j in range(deg+1+k+1):
            if j >= deg+1:
                _t[j, j-1] = 1.
            elif j <= k:
                _t[j, j] = 1.
            else:
                _t[j, j-1] = (knots[j+deg-k]-t_shift)/(knots[j+deg-k]-knots[j])
                _t[j, j] = (t_shift-knots[j])/(knots[j+deg-k]-knots[j])
        _T = _t.dot(_T)
    T[:deg, :deg+1] = _T[deg+1:, :]
    T_extr = extrapolate_T(basis, knots[-1] - knots[-deg-2])
    T[-deg-1:, -deg-1:] = T_extr[-deg-1:, -deg-1:]
    return T


def shift_knot1_fwd(cfs, basis, t_shift):
    if isinstance(cfs, (SX, MX)):
        cfs_sym = MX.sym('cfs', cfs.shape)
        t_shift_sym = MX.sym('t_shift')
        T = shiftfirstknot_T(basis, t_shift_sym)
        cfs2_sym = mul(T, cfs_sym)
        fun = MXFunction('fun', [cfs_sym, t_shift_sym], [cfs2_sym])
        return fun([cfs, t_shift])[0]
    else:
        T = shiftfirstknot_T(basis, t_shift)
        return T.dot(cfs)


def shift_knot1_bwd(cfs, basis, t_shift):
    if isinstance(cfs, (SX, MX)):
        cfs_sym = SX.sym('cfs', cfs.shape)
        t_shift_sym = SX.sym('t_shift')
        T, Tinv = shiftfirstknot_T(basis, t_shift_sym, inverse=True)
        cfs2_sym = mul(Tinv, cfs_sym)
        fun = SXFunction('fun', [cfs_sym, t_shift_sym], [cfs2_sym])
        return fun([cfs, t_shift])[0]
    else:
        T, Tinv = shiftfirstknot_T(basis, t_shift, inverse=True)
        return Tinv.dot(cfs)


def shiftfirstknot_T(basis, t_shift, inverse=False):
    # Create transformation matrix that shift the first (degree+1) knots over
    # t_shift. With inverse = True, the inverse transformation is also computed.
    knots, deg = basis.knots, basis.degree
    N = len(basis)
    if isinstance(t_shift, SX):
        typ, sym = SX, True
    elif isinstance(t_shift, MX):
        typ, sym = MX, True
    else:
        typ, sym = np, False
    _T = typ.eye(deg+1)
    for k in range(deg+1):
        _t = typ.zeros((deg+1+k+1, deg+1+k))
        for j in range(deg+1+k+1):
            if j >= deg+1:
                _t[j, j-1] = 1.
            elif j <= k:
                _t[j, j] = 1.
            else:
                _t[j, j-1] = (knots[j+deg-k]-t_shift)/(knots[j+deg-k]-knots[j])
                _t[j, j] = (t_shift-knots[j])/(knots[j+deg-k]-knots[j])
        _T = mul(_t, _T) if sym else _t.dot(_T)
    T = typ.eye(N)
    T[:deg+1, :deg+1] = _T[deg+1:, :]
    if inverse:  # T is upper triangular: easy inverse
        Tinv = typ.eye(len(basis))
        for i in range(deg, -1, -1):
            Tinv[i, i] = 1./T[i, i]
            for j in range(deg, i, -1):
                Tinv[i, j] = (-1./T[i, i])*sum([T[i, k]*Tinv[k, j]
                                                for k in range(i+1, deg+2)])
        return T, Tinv
    else:
        return T


def integral_sqbasis(basis):
    # Compute integral of squared bases.
    basis_prod = basis*basis
    pairs, S = basis.pairs(basis)
    b_self = basis(basis_prod._x)
    basis_product = b_self[:, pairs[0]].multiply(b_self[:, pairs[1]])
    T = basis_prod.transform(lambda y: basis_product.toarray()[y, :])

    knots = basis_prod.knots
    d = basis_prod.degree
    K = np.array((knots[d + 1:] - knots[:-(d + 1)]) / (d + 1))

    L = len(basis)
    B = np.zeros((L, L))
    degree = basis.degree
    k = 0
    for i in range(L):
        c1 = np.zeros(L)
        c1[i] = 1
        for j in range(i, i+degree+1-k):
            c2 = np.zeros(L)
            c2[j] = 1
            coeffs_product = (c1[pairs[0].tolist()]*c2[pairs[1].tolist()])
            c_prod = T.dot(coeffs_product)
            bb = K.T.dot(c_prod)
            B[i, j] = bb
            B[j, i] = bb
        if i >= L-degree-1:
            k += 1
    return B


def def_integral_sqbasisMX(basis, a, b):
    # Compute integral of squared bases.
    L = len(basis)
    degree = basis.degree
    if isinstance(a, MX) or isinstance(b, MX):
        B = MX.zeros(L, L)
    elif isinstance(a, SX) or isinstance(b, SX):
        B = SX.zeros(L, L)
    else:
        B = np.zeros((L, L))
    k = 0
    for i in range(L):
        c1 = np.zeros(L)
        c1[i] = 1
        for j in range(i, i+degree+1-k):
            c2 = np.zeros(L)
            c2[j] = 1
            s1 = BSpline(basis, c1)
            s2 = BSpline(basis, c2)
            bb = definite_integral((s1*s2), a, b)
            B[i, j] = bb
            B[j, i] = bb
        if i >= L-degree-1:
            k += 1
    return B
