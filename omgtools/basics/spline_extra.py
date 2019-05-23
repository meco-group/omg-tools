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

from .spline import BSpline, BSplineBasis
from casadi import SX, MX, mtimes, Function, vertcat
from scipy.interpolate import splev
import scipy.linalg as la
import numpy as np

import warnings

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
        result += mtimes(coeffs[l], basis[-1][l])
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
        coeffs_int.append(coeffs_int[i]+(knots[degree+i+1]-knots[i])/float(degree_int)*coeffs[i])
    if isinstance(coeffs, (MX, SX)):
        coeffs_int = vertcat(*coeffs_int)
    else:
        coeffs_int = np.array(coeffs_int)
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
        cfs2_sym = mtimes(T, cfs_sym)
        fun = Function('fun', [cfs_sym, t_shift_sym], [cfs2_sym]).expand()
        return fun(cfs, t_shift)
    else:
        T = shiftfirstknot_T(basis, t_shift)
        return T.dot(cfs)


def shift_knot1_bwd(cfs, basis, t_shift):
    if isinstance(cfs, (SX, MX)):
        cfs_sym = SX.sym('cfs', cfs.shape)
        t_shift_sym = SX.sym('t_shift')
        _, Tinv = shiftfirstknot_T(basis, t_shift_sym, inverse=True)
        cfs2_sym = mtimes(Tinv, cfs_sym)
        fun = Function('fun', [cfs_sym, t_shift_sym], [cfs2_sym]).expand()
        return fun(cfs, t_shift)
    else:
        _, Tinv = shiftfirstknot_T(basis, t_shift, inverse=True)
        return Tinv.dot(cfs)


def shiftfirstknot_T(basis, t_shift, inverse=False):
    # Create transformation matrix that shifts the first (degree+1) knots over
    # t_shift. With inverse = True, the inverse transformation is also
    # computed.
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
        _T = mtimes(_t, _T) if sym else _t.dot(_T)
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


def knot_insertion_T(basis, knots_to_insert):
    # Create transformation matrix that transforms spline after inserting knots
    N = len(basis)
    knots = basis.knots.tolist()
    degree = basis.degree
    T = np.eye(N)
    for knot in knots_to_insert:
        _T = np.zeros((N+1, N))
        for j in range(N+1):
            if knot <= knots[j]:
                w = 0.
            elif knots[j] < knot and knot < knots[j+degree+1-1]:
                w = (knot - knots[j])/(knots[j+degree+1-1] - knots[j])
            else:
                w = 1.
            if j != 0:
                _T[j, j-1] = (1.-w)
            if j != N:
                _T[j, j] = w
        T = _T.dot(T)
        N += 1
        knots = sorted(knots + [knot])
    return T, knots


def get_interval_T(basis, min_value, max_value):
    # Create transformation matrix that extract piece of spline from min_value
    # to max_value
    knots = basis.knots
    degree = basis.degree
    n_min = len(np.where(knots == min_value)[0])
    n_max = len(np.where(knots == max_value)[0])
    min_knots = [min_value]*(degree + 1 - n_min)
    max_knots = [max_value]*(degree + 1 - n_max)
    T, knots2 = knot_insertion_T(basis, min_knots+max_knots)
    jmin = np.searchsorted(knots2, min_value, side='left')
    jmax = np.searchsorted(knots2, max_value, side='right')
    return T[jmin:jmax-degree-1, :], knots2[jmin:jmax]


def crop_spline(spline, min_value, max_value):
    T, knots2 = get_interval_T(spline.basis, min_value, max_value)
    if isinstance(spline.coeffs, (SX, MX)):
        coeffs2 = mtimes(T, spline.coeffs)
    else:
        coeffs2 = T.dot(spline.coeffs)
    basis2 = BSplineBasis(knots2, spline.basis.degree)
    return BSpline(basis2, coeffs2)


def concat_splines(segments, segment_times, n_insert=None):
    # While concatenating check continuity of segments, this determines
    # the required amount of knots to insert. If segments are continuous
    # up to degree, no extra knots are required. If they are not continuous
    # at all, degree+1 knots are inserted in between the splines.
    spl0 = segments[0]
    knots = [s.basis.knots*segment_times[0] for s in spl0]  # give dimensions
    degree = [s.basis.degree for s in spl0]
    coeffs = [s.coeffs for s in spl0]
    prev_segment = [s.scale(segment_times[0], shift=0) for s in spl0]  # save the combined segment until now (with dimenions)
    prev_time = segment_times[0]  # save the motion time of combined segment
    for k in range(1, len(segments)):
        for l, s in enumerate(segments[k]):
            if s.basis.degree != degree[l]:
                # all concatenated splines should be of the same degree
                raise ValueError(
                    'Splines at index ' + l + 'should have same degree.')
            if n_insert is None:
                # check continuity, n_insert can be different for each l
                n_insert = degree[l]+1  # starts at max value
                for d in range(degree[l]+1):
                    # use ipopt default tolerance as a treshold for check (1e-3)
                    # give dimensions, to compare using the same time scale
                    # use scale function to give segments[k][l] dimensions
                    # prev_segment already has dimensions

                    # Todo: sometimes this check fails, or you get 1e-10 and 1e-11 values that should actually be equal
                    # this seems due to the finite tolerance of ipopt? and due to the fact that these are floating point numbers
                    val1 = segments[k][l].scale(segment_times[k], shift = prev_time).derivative(d)(prev_time)
                    val2 = prev_segment[l].derivative(d)(prev_time)
                    if ( abs(val1 - val2)*0.5/(val1 + val2) <= 1e-3):
                        # more continuity = insert less knots
                        n_insert -= 1
                    else:
                        # spline values were not equal, stop comparing and use latest n_insert value
                        break
            # else:  # keep n_insert from input

            if n_insert!= degree[l]+1:
                # concatenation requires re-computing the coefficients and/or adding extra knots
                # here we make the knot vector and coeffs of the total spline (the final output),
                # but we also compute the knot vector and coeffs for concatenating just two splines,
                # to limit the size of the system that we need to solve
                # i.e.: when concatenating 3 splines, we first concatenate segments 1 and 2,
                # and afterwards segments 2 and 3, this happens due to the main for loop

                # make total knot vector
                end_idx = len(knots[l])-(degree[l]+1)+n_insert
                knots[l] = np.r_[
                    knots[l][:end_idx], s.basis.knots[degree[l]+1:]*segment_times[k] + knots[l][-1]]  # last term = time shift

                # make knot vector for two segments to concatenate
                end_idx_concat = len(prev_segment[l].basis.knots)-(degree[l]+1)+n_insert
                knots1 = prev_segment[l].basis.knots[:end_idx_concat]
                knots2 = s.basis.knots[degree[l]+1:]*segment_times[k] + knots1[-1]  # last term = time shift
                knots_concat = np.r_[knots1, knots2]

                # make union basis for two segments to concatenate
                basis_concat = BSplineBasis(knots_concat, degree[l])
                grev_bc = basis_concat.greville()
                # shift first and last greville point inwards, to avoid numerical problems,
                # in which the spline evaluates to 0 because the evaluation lies just outside the domain
                grev_bc[0] = grev_bc[0] + (grev_bc[1]-grev_bc[0])*0.01
                grev_bc[-1] = grev_bc[-1] - (grev_bc[-1]-grev_bc[-2])*0.01
                # evaluate basis on its greville points
                eval_bc = basis_concat(grev_bc).toarray()

                # only the last degree coeffs of segment1 and the first degree coeffs
                # of segment2 will change --> we can reduce the size of the system to solve
                # Todo: implement this reduction

                # first give the segment dimensions, by scaling it and shifting it appropriatly
                # this is necessary, since by default the domain is [0,1]
                # then evaluate the splines that you want to concatenate on greville points
                # of union basis, will give 0 outside their domain
                s_1 = prev_segment[l](grev_bc)
                s_2 = segments[k][l].scale(segment_times[k], shift = prev_time)(grev_bc)
                # sum to get total evaluation
                eval_sc = s_1 + s_2

                # solve system to find new coeffs
                coeffs_concat = la.solve(eval_bc, eval_sc)
                # save new coefficients
                coeffs[l] = coeffs_concat
            else:
                #there was no continuity, just compute new knot vector and stack coefficients
                knots[l] = np.r_[
                           knots[l], s.basis.knots[degree[l]+1:]*segment_times[k] + knots[l][-1]]
                coeffs[l] = np.r_[coeffs[l], s.coeffs]
        # going to next segment, update time shift
        new_bases = [BSplineBasis(knots[l], degree[l])for l in range(len(segments[0]))]
        prev_segment = [BSpline(new_bases[l], coeffs[l]) for l in range(len(segments[0]))]
        prev_time += segment_times[k]

    bases = [BSplineBasis(knots[l], degree[l])
             for l in range(len(segments[0]))]
    return [BSpline(bases[l], coeffs[l]) for l in range(len(segments[0]))]

def sample_splines(spline, time):
    if isinstance(spline, list):
        return [splev(time, (s.basis.knots, s.coeffs, s.basis.degree)) for s in spline]
    else:
        return splev(time, (spline.basis.knots, spline.coeffs, spline.basis.degree))


# def integral_sqbasis(basis):
#     # Compute integral of squared bases.
#     basis_prod = basis*basis
#     pairs, _ = basis.pairs(basis)
#     b_self = basis(basis_prod._x)
#     basis_product = b_self[:, pairs[0]].multiply(b_self[:, pairs[1]])
#     T = basis_prod.transform(lambda y: basis_product.toarray()[y, :])

#     knots = basis_prod.knots
#     d = basis_prod.degree
#     K = np.array((knots[d + 1:] - knots[:-(d + 1)]) / (d + 1))

#     L = len(basis)
#     B = np.zeros((L, L))
#     degree = basis.degree
#     k = 0
#     for i in range(L):
#         c1 = np.zeros(L)
#         c1[i] = 1
#         for j in range(i, i+degree+1-k):
#             c2 = np.zeros(L)
#             c2[j] = 1
#             coeffs_product = (c1[pairs[0].tolist()]*c2[pairs[1].tolist()])
#             c_prod = T.dot(coeffs_product)
#             bb = K.T.dot(c_prod)
#             B[i, j] = bb
#             B[j, i] = bb
#         if i >= L-degree-1:
#             k += 1
#     return B


# def definite_integral_sqbasisMX(basis, a, b):
#     # Compute integral of squared bases. (in a dirty way)
#     L = len(basis)
#     degree = basis.degree
#     if isinstance(a, MX) or isinstance(b, MX):
#         typ = MX
#     elif isinstance(a, SX) or isinstance(b, SX):
#         typ = SX
#     else:
#         typ = np
#     B = typ.zeros((L, L))
#     k = 0
#     for i in range(L):
#         c1 = np.zeros(L)
#         c1[i] = 1
#         for j in range(i, i+degree+1-k):
#             c2 = np.zeros(L)
#             c2[j] = 1
#             s1 = BSpline(basis, c1)
#             s2 = BSpline(basis, c2)
#             bb = definite_integral((s1*s2), a, b)
#             B[i, j] = bb
#             B[j, i] = bb
#         if i >= L-degree-1:
#             k += 1
#     return B
