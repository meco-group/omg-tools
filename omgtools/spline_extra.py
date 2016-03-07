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
    m = 1  # number of repeating internal knots
    while knots[-deg-2-m] >= knots[-deg-2]:
        m += 1
    knots2 = np.r_[knots[:-deg-1], knots[-deg-1]*np.ones(m),
                   (knots[-1]+t_extra)*np.ones(deg+1)]
    basis2 = BSplineBasis(knots2, deg)
    A = np.zeros((deg+1, deg+1))
    B = np.zeros((deg+1, deg+1))
    # (deg+1-m) relations based on evaluation of basis functions on (deg+1-m)
    # last greville points
    if m < deg+1:
        eval_points = basis.greville()[-(deg+1-m):]
        a = basis2.eval_basis(eval_points).toarray()[:, -(deg+1+m):-m]
        b = basis.eval_basis(eval_points).toarray()[:, -(deg+1):]
        a1, a2 = a[:, :m], a[:, m:]
        b1, b2 = b[:, :m], b[:, m:]
        A[:(deg+1-m), -(deg+1):-m] = a2
        B[:(deg+1-m), :m] = b1 - a1  # this should be zeros
        B[:(deg+1-m), m:] = b2
    else:
        A[0, -(deg+1)] = 1.
        B[0, -1] = 1.
    # m relations based on continuity of m last derivatives
    A1, B1 = np.identity(deg+1), np.identity(deg+1)
    for i in range(1, deg+1):
        A1_tmp = np.zeros((deg+1-i, deg+1-i+1))
        B1_tmp = np.zeros((deg+1-i, deg+1-i+1))
        for j in range(deg+1-i):
            B1_tmp[j, j] = -(deg+1-i)/(knots[j+N] - knots[j+N-deg-1+i])
            B1_tmp[j, j+1] = (deg+1-i)/(knots[j+N] - knots[j+N-deg-1+i])
            A1_tmp[j, j] = -(deg+1-i)/(knots2[j+N+m] - knots2[j+N-deg-1+m+i])
            A1_tmp[j, j+1] = (deg+1-i)/(knots2[j+N+m] - knots2[j+N-deg-1+m+i])
        A1, B1 = A1_tmp.dot(A1), B1_tmp.dot(B1)
        if i >= deg+1-m:
            b1 = B1[-1, :]
            a1 = A1[-(deg-i+1), :]
            A[i, :] = a1
            B[i, :] = b1
    # put everything in transformation matrix
    _T = np.linalg.solve(A, B)
    _T[abs(_T) < 1e-10] = 0.
    T = np.zeros((N+m, N))
    T[:N, :N] = np.eye(N)
    T[-(deg+1):, -(deg+1):] = _T
    return T


def shift_over_knot(coeffs, basis):
    T = shiftoverknot_T(basis)
    return T.dot(coeffs)


def shiftoverknot_T(basis):
    # Create transformation matrix that moves the horizon to
    # [knot[degree+1], T+knots[-1]-knots[-deg-2]]. The spline is extrapolated
    # over the last knot interval.
    knots = basis.knots
    deg = basis.degree
    m = 1  # number of repeating internal knots
    while knots[-deg-2-m] >= knots[-deg-2]:
        m += 1
    t_shift = knots[deg+1] - knots[0]
    T = np.diag(np.ones(len(basis)-m), m)
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
    T[-(deg+1):, -(deg+1):] = T_extr[-(deg+1):, -(deg+1):]
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
