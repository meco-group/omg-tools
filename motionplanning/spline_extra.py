from spline import BSpline, BSplineBasis
from scipy.interpolate import splev
from casadi import SX, MX, mul, solve, SXFunction
import numpy as np


def evalspline(s, x):
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
    spline_int = running_integral(spline)
    int_0a = evalspline(spline_int, a)
    int_0b = evalspline(spline_int, b)
    int_ab = int_0b - int_0a
    return int_ab


def shift_spline(coeffs, update_time, target_time, basis):
    n_knots = len(basis) - basis.degree + 1
    knots_ = np.r_[round(update_time/target_time, 5)*np.ones(basis.degree),
                   np.linspace(round(update_time/target_time, 5), 1., n_knots),
                   np.ones(basis.degree)]
    B_ = BSplineBasis(knots_, basis.degree)
    T_tf = B_.transform(basis)
    return T_tf.dot(coeffs)


def shift_knot1_fwd(cfs, knots, degree, t_shift):
    if isinstance(cfs, list):
        return [shift_knot1_fwd(c_i, knots, degree, t_shift) for c_i in cfs]
    elif isinstance(cfs, (MX, SX)):
        return _shift_knot1_fwd_sym(cfs, knots, degree, t_shift)
    else:
        return _shift_knot1_fwd_num(cfs, knots, degree, t_shift)


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
    coeffs = np.array(coeffs)
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


def shift_knot1_bwd(cfs, knots, degree, t_shift):
    # knots is the 'original' knot sequence: the one that you want to arrive
    # after performing the backward shift
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
    coeffs = np.array(coeffs)
    t_shift = float(t_shift)
    knots1 = np.copy(knots1)
    knots1[:degree+1] = (knots[0]+t_shift)*np.ones(degree+1)
    coeffs2 = np.copy(coeffs)
    knots2 = np.copy(knots2)
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


def shift_over_knot(coeffs, knots, degree, N_shift=1):
    # Assumption: spline has zero derivatives at the end!
    if isinstance(coeffs, list):
        return [shift_over_knot(c_i, knots, degree, N_shift) for c_i in coeffs]
    coeffs = np.array(coeffs)
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
