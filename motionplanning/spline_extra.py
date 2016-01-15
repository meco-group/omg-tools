from spline import BSpline, BSplineBasis
from scipy.interpolate import splev
from casadi import SX, MX, mul, solve, SXFunction
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


def shift_knot1_fwd(cfs, knots, degree, t_shift):
    # Extract spline piece in [t_shift, T] and express it in a basis composed
    # of the original future knots and shift first knots. This is exact.
    if isinstance(cfs, list):
        return [shift_knot1_fwd(c_i, knots, degree, t_shift) for c_i in cfs]
    elif isinstance(cfs, (MX, SX)):
        return _shift_knot1_fwd_sym(cfs, knots, degree, t_shift)
    else:
        return _shift_knot1_fwd_num2(cfs, knots, degree, t_shift)


def _shift_knot1_fwd_sym(cfs, knots, degree, t_shift):
    coeffs = SX.sym('coeffs', cfs.shape)
    t0 = SX.sym('t_shift')
    knots2 = SX.zeros(len(knots), 1)
    coeffs2 = SX.zeros(coeffs.shape)
    for k in range(len(knots)):
        if k in range(degree+1):
            knots2[k] = knots[0]+t0
        else:
            knots2[k] = knots[k]
    for n in range(coeffs.size2()):
        for k in range(coeffs.size1()):
            coeffs2[k, n] = coeffs[k, n]
    A = SX.eye(degree)
    A0 = SX.zeros((degree, degree))
    A0[0, 0] = 1.
    for i in range(1, degree):
        A_tmp = SX.zeros((degree-i, degree-i+1))
        for j in range(degree-i):
            A_tmp[j, j] = -(degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
            A_tmp[j, j+1] = (degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
        A = mul(A_tmp, A)
        A0[i, :] = A[0, :]
    basis = BSplineBasis(knots, degree)
    for n in range(coeffs.size2()):
        spline = BSpline(basis, coeffs[:, n])
        b0 = SX.zeros((degree, 1))
        b0[0] = evalspline(spline, knots[0]+t0)
        for i in range(1, degree):
            ds = spline.derivative(i)
            ds.coeffs = SX(ds.coeffs)
            b0[i] = evalspline(ds, knots[0]+t0)
        coeffs2[:degree, n] = solve(A0, b0)
    fun = SXFunction('fun', [coeffs, t0], [coeffs2])
    return fun([cfs, t_shift])[0]


def _shift_knot1_fwd_num(coeffs, knots, degree, t_shift):
    if t_shift > (knots[degree+1]-knots[degree]):
        raise ValueError('t_shift is bigger than knot distance!')
    coeffs = np.c_[coeffs]
    knots2 = np.copy(knots)
    knots2[:degree+1] = (knots[0]+float(t_shift))*np.ones(degree+1)
    coeffs2 = np.copy(coeffs)
    A = np.identity(degree)
    A0 = np.zeros((degree, degree))
    A0[0, 0] = 1.
    for i in range(1, degree):
        A_tmp = np.zeros((degree-i, degree-i+1))
        for j in range(degree-i):
            A_tmp[j, j] = -(degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
            A_tmp[j, j+1] = (degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
        A = A_tmp.dot(A)
        A0[i, :] = A[0, :]
    for n in range(coeffs.shape[1]):
        b0 = np.zeros((degree, 1))
        for i in range(degree):
            b0[i] = splev(knots2[0], (knots, coeffs[:, n], degree), der=i)
        coeffs2[:degree, n] = np.linalg.solve(A0, b0).ravel()
    return coeffs2


# def _shift_knot1_fwd_num2(coeffs, knots, degree, t_shift):
#     if t_shift > (knots[degree+1]-knots[degree]):
#         raise ValueError('t_shift is bigger than knot distance!')
#     basis1 = BSplineBasis(knots, degree)
#     knots2 = np.copy(knots)
#     knots2[:degree+1] = (knots[0]+float(t_shift))*np.ones(degree+1)
#     basis2 = BSplineBasis(knots2, degree)
#     T_tf = basis2.transform(basis1)
#     return T_tf.dot(coeffs)

def _shift_knot1_fwd_num2(coeffs, knots, degree, t_shift):
    if t_shift > (knots[degree+1]-knots[degree]):
        raise ValueError('t_shift is bigger than knot distance!')
    basis1 = BSplineBasis(knots, degree)
    knots2 = np.copy(knots)
    knots2[:degree+1] = (knots[0]+float(t_shift))*np.ones(degree+1)
    basis2 = BSplineBasis(knots2, degree)
    T_tf = basis2.transform(basis1)
    return T_tf.dot(coeffs)

def shift_knot1_bwd(cfs, knots, degree, t_shift):
    # This is the reverse transformation of shift_knot1_fwd. Given your original
    # knot sequence ('knots') and transformed coefficients ('cfs'), give original
    # coefficients before the time shift.
    if isinstance(cfs, list):
        return [shift_knot1_bwd(c_i, knots, degree, t_shift) for c_i in cfs]
    elif isinstance(cfs, (MX, SX)):
        return _shift_knot1_bwd_sym(cfs, knots, degree, t_shift)
    else:
        return _shift_knot1_bwd_num(cfs, knots, degree, t_shift)


def _shift_knot1_bwd_sym(cfs, knots, degree, t_shift):
    coeffs = SX.sym('coeffs', cfs.shape)
    t0 = SX.sym('t_shift')

    knots1 = SX.zeros(len(knots), 1)
    for k in range(knots1.size1()):
        if k in range(degree+1):
            knots1[k] = knots[0]+t0
        else:
            knots1[k] = knots[k]
    knots2 = knots
    coeffs2 = SX.zeros(coeffs.shape)
    for n in range(coeffs.size2()):
        for k in range(coeffs.size1()):
            coeffs2[k, n] = coeffs[k, n]
    A = SX.eye(degree+1)
    A0 = SX.zeros((degree+1, degree+1))
    A0[0, 0] = 1.
    for i in range(1, degree+1):
        A_tmp = SX.zeros((degree-i+1, degree-i+2))
        for j in range(degree-i+1):
            A_tmp[j, j] = -(degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
            A_tmp[j, j+1] = (degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
        A = mul(A_tmp, A)
        A0[i, :] = A[0, :]
    b0_ = SX.zeros((degree+1, coeffs.size2()))
    for n in range(coeffs.size2()):
        b0_[0, n] = coeffs[0, n]
    A_ = SX.eye(degree+1)
    for i in range(1, degree+1):
        A_tmp = SX.zeros((degree-i+1, degree-i+2))
        for j in range(degree-i+1):
            A_tmp[j, j] = -(degree+1-i)/(knots1[j+degree+1] - knots1[j+i])
            A_tmp[j, j+1] = (degree+1-i)/(knots1[j+degree+1] - knots1[j+i])
        A_ = mul(A_tmp, A_)
        for n in range(coeffs.size2()):
            b0_[i, n] = mul(A_[0, :], coeffs[:degree+1, n])
    for n in range(coeffs.size2()):
        b0 = SX.zeros((degree+1, 1))
        for i in range(degree, -1, -1):
            b0[i] = b0_[i, n]
            for j in range(i+1, degree+1):
                b0[i] -= b0[j]*(t0**(j-i))/np.math.factorial(j-i)
        coeffs2[:degree+1, n] = solve(A0, b0)
    fun = SXFunction('fun', [coeffs, t0], [coeffs2])
    return fun([cfs, t_shift])[0]


def _shift_knot1_bwd_num(coeffs, knots, degree, t_shift):
    coeffs = np.c_[coeffs]
    t_shift = float(t_shift)
    knots1 = np.copy(knots)
    knots1[:degree+1] = (knots[0]+t_shift)*np.ones(degree+1)
    coeffs2 = np.copy(coeffs)
    knots2 = np.copy(knots)
    A = np.identity(degree+1)
    A0 = np.zeros((degree+1, degree+1))
    A0[0, 0] = 1.
    for i in range(1, degree+1):
        A_tmp = np.zeros((degree-i+1, degree-i+2))
        for j in range(degree-i+1):
            A_tmp[j, j] = -(degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
            A_tmp[j, j+1] = (degree+1-i)/(knots2[j+degree+1] - knots2[j+i])
        A = A_tmp.dot(A)
        A0[i, :] = A[0, :]
    b0_ = np.zeros((degree+1, coeffs.shape[1]))
    for n in range(coeffs.shape[1]):
        b0_[0, n] = coeffs[0, n]
    A_ = np.identity(degree+1)
    for i in range(1, degree+1):
        A_tmp = np.zeros((degree-i+1, degree-i+2))
        for j in range(degree-i+1):
            A_tmp[j, j] = -(degree+1-i)/(knots1[j+degree+1] - knots1[j+i])
            A_tmp[j, j+1] = (degree+1-i)/(knots1[j+degree+1] - knots1[j+i])
        A_ = A_tmp.dot(A_)
        for n in range(coeffs.shape[1]):
            b0_[i, n] = A_[0, :].dot(coeffs[:degree+1, n])
    for n in range(coeffs.shape[1]):
        b0 = np.zeros((degree+1, 1))
        for i in range(degree, -1, -1):
            b0[i] = b0_[i, n]
            for j in range(i+1, degree+1):
                b0[i] -= b0[j]*(t_shift**(j-i))/np.math.factorial(j-i)
        coeffs2[:degree+1, n] = np.linalg.solve(A0, b0).ravel()
    return coeffs2


# def _shift_knot1_bwd_num2(coeffs, knots, degree, t_shift):
#     basis1 = BSplineBasis(knots, degree)
#     knots2 = np.copy(knots)
#     knots2[:degree+1] = (knots[0]+float(t_shift))*np.ones(degree+1)
#     basis2 = BSplineBasis(knots2, degree)
#     T_tf = basis1.transform(basis2)
#     return T_tf.dot(coeffs)


def shift_over_knot_old(coeffs, knots, degree, N_shift=1):
    # Move horizon to [knot[N_shift], T+knot[N_shift]]. The spline is extrapolated
    # in the interval [T, T+knot[N_shift]]
    # Assumption: spline has zero derivatives at the end!
    if isinstance(coeffs, list):
        return [shift_over_knot(c_i, knots, degree, N_shift) for c_i in coeffs]
    coeffs = np.c_[coeffs]
    coeffs2 = np.zeros(coeffs.shape)
    coeffs2[:-1, :] = coeffs[1:, :]
    coeffs2[-1, :] = coeffs2[-2, :]
    for i in range(degree+1):
        coeffs2[-2-i, :] = coeffs2[-1, :]
    if degree > 1:
        A = np.identity(degree-1)
        A0 = np.zeros((degree-1, degree-1))
        A0[0, 0] = 1.
        for i in range(1, degree-1):
            A_tmp = np.zeros((degree-i-1, degree-i))
            for j in range(degree-i-1):
                A_tmp[j, j] = -(degree+1-i)/(knots[j+degree+1] - knots[j+i])
                A_tmp[j, j+1] = (degree+1-i)/(knots[j+degree+1] - knots[j+i])
            A = A_tmp.dot(A)
            A0[i, :] = A[0, :]
        for n in range(coeffs.shape[1]):
            b0 = np.zeros((degree-1, 1))
            for i in range(degree-1):
                b0[i] = splev(knots[degree+N_shift],
                              (knots, coeffs[:, n].ravel(), degree), der=i)
            coeffs2[:degree-1, n] = np.linalg.solve(A0, b0).ravel()
    return coeffs2


def shift_over_knot(coeffs, knots, degree, N_shift=2):
    # No assumption on zero derivatives at end
    if isinstance(coeffs, list):
        return [shift_over_knot_old(c_i, knots, degree, N_shift)
                for c_i in coeffs]
    coeffs = np.c_[coeffs]
    L = coeffs.shape[0]
    A = np.identity(L)
    A0 = np.zeros((degree, L))
    A0[0, 0] = 1.
    for i in range(1, degree+1):
        A_tmp = np.zeros((L-i, L-i+1))
        for j in range(L-i):
            A_tmp[j, j] = -(degree+1-i)/(knots[j+degree+1] - knots[j+i])
            A_tmp[j, j+1] = (degree+1-i)/(knots[j+degree+1] - knots[j+i])
        A = A_tmp.dot(A)
        if i < degree:
            A0[i, :] = A[0, :]
    A = np.vstack((A, A0))

    coeffs2 = np.zeros(coeffs.shape)
    for n in range(coeffs.shape[1]):
        coeffs_n = coeffs[:, n]
        dcoeffs = A.dot(coeffs_n)
        b = np.zeros((L-degree, 1))
        b0 = np.zeros((degree, 1))
        for j in range(N_shift):
            b[j] = dcoeffs[N_shift]
        for j in range(N_shift, L-degree-1):
            b[j] = dcoeffs[j+1]
        b[L-degree-1] = b[L-degree-2]
        for i in range(degree):
            b0[i] = splev(knots[degree+N_shift],
                          (knots, coeffs_n.ravel(), degree), der=i)
        b = np.vstack((b, b0))
        coeffs2_n = np.linalg.solve(A, b).ravel()

        # This is not necessary if der=0 constraints are added in upd x and z.
        # But is included for certainty (wrt numerical effects)
        # coeffs2_n[-1] = splev(knots[-1],
        #                       (knots, coeffs_n.ravel(), degree), der=0)
        # for i in range(degree+1):
        #     coeffs2_n[-2-i] = coeffs2_n[-1]

        coeffs2[:, n] = coeffs2_n
    return coeffs2

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
