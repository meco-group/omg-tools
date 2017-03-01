import sys, os
sys.path.insert(0, os.getcwd()+'/../..')

import meco_binaries; meco_binaries(cpp_splines='develop')
import splines as spl
import numpy as np
import omgtools.basics.spline_extra as spl_ex
import omgtools.basics.spline_old as spl_ol
import time

# create basis similar as in quadrotor3d example
degree = 2
knots = np.r_[np.zeros(degree), np.linspace(0, 5, 11), 5*np.ones(degree)]

# basis creation
t0 = time.time()
B1 = spl.BSplineBasis(knots, degree)
t1 = time.time()
B2 = spl_ol.BSplineBasis(knots, degree)
t2 = time.time()
print 'basis creation: ' + str(t1-t0) + ' vs ' + str(t2-t1)

# spline creation
ca = np.random.rand(B1.dimension())
t0 = time.time()
a1 = spl.Function(B1, ca)
t1 = time.time()
a2 = spl_ol.BSpline(B2, ca)
t2 = time.time()
print 'spline creation: ' + str(t1-t0) + ' vs ' + str(t2-t1)

cb = np.random.rand(B1.dimension())
cc = np.random.rand(B1.dimension())
b1 = spl.Function(B1, cb)
c1 = spl.Function(B1, cc)
b2 = spl_ol.BSpline(B2, cb)
c2 = spl_ol.BSpline(B2, cc)

# artithmitics
t0 = time.time()
x1 = a1*(1-b1**2)*(1-c1**2)
t1 = time.time()
x2 = a2*(1-b2**2)*(1-c2**2)
t2 = time.time()
print 'arithmitic operations: ' + str(t1-t0) + ' vs ' + str(t2-t1)

# 2nd derivative
t0 = time.time()
ddx1 = x1.derivative(2)
t1 = time.time()
ddx2 = x2.derivative(2)
t2 = time.time()
print '2nd derivative: ' + str(t1-t0) + ' vs ' + str(t2-t1)

# 2nd antiderivative
t0 = time.time()
iet1 = ddx1.antiderivative(2)
t1 = time.time()
dx2 = spl_ex.running_integral(ddx2)
iet2 = spl_ex.running_integral(dx2)
t2 = time.time()
print '2nd antiderivative: ' + str(t1-t0) + ' vs ' + str(t2-t1)

# project to
knots2 = np.r_[0.1*np.ones(degree), np.linspace(0.1, 5, 11), 5*np.ones(degree)]
t0 = time.time()
B1b = spl.BSplineBasis(knots2, degree)
T1 = B1.project_to(B1b)
t1 = time.time()
B2b = spl_ol.BSplineBasis(knots2, degree)
T2 = B2b.transform(B2)
t2 = time.time()
print 'project_to: ' + str(t1-t0) + ' vs ' + str(t2-t1)

# knot insertion
t0 = time.time()
B1c, T = B1b.insert_knots(0.1*np.ones(degree+1))
t1 = time.time()
T = spl_ex.insert_knots_T(B2, (0.1*np.ones(degree+1)).tolist())
t2 = time.time()
print 'insert_knots: ' + str(t1-t0) + ' vs ' + str(t2-t1)
