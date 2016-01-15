from spline_extra import *
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import numpy as np
import time

degree = 3
T = 10
knots = np.r_[np.zeros(degree), np.linspace(0., T, 11), T*np.ones(degree)]
basis = BSplineBasis(knots, degree)
cfs_y = np.array([ 0., 0., 0.014, 0.074, 0.176, 0.318, 0.5, 0.682, 0.824, 0.926, 0.986, 1., 1.])
y = BSpline(basis, cfs_y)

t0 = time.time()
B = integral_sqbasis(basis)
t1 = time.time()
C = np.linalg.solve(B, np.eye(len(basis)))
print t1-t0

cfs_x_ad = 5.*np.array([ 0., 0., 0.014, 0.074, 0.23, 0.318, 0.5, 0.82, 0.824, 0.926, 0.986, 1., 1.])
cfs_x = C.T.dot(cfs_x_ad)

x = BSpline(basis, cfs_x)

res1 = (x*y).integral()
res2 = cfs_x_ad.T.dot(cfs_y)

print res1
print res2

t_shift = 0.6

cfs_y2 = shift_knot1_fwd(cfs_y, knots, degree, t_shift)

knots2 = np.r_[t_shift*np.ones(degree+1), knots[degree+1:]]
basis2 = BSplineBasis(knots2, degree)
y2 = BSpline(basis2, cfs_y2)

B = integral_sqbasis(basis2)
cfs_x2 = shift_knot1_fwd(cfs_x, knots, degree, t_shift)
cfs_x2_ad = B.dot(cfs_x2)

x2 = BSpline(basis2, cfs_x2)


res1 = (x2*y2).integral()
res2 = cfs_x2_ad.T.dot(cfs_y2)
res3 = definite_integral((x*y), t_shift, T)

print res1
print res2
print res3


# t_ax2 = t_ax[t_ax >= t_shift]
