import sys, os
sys.path.insert(0, os.getcwd()+'/..')
from omgtools.basics.spline import *
from omgtools.basics.spline_extra import definite_integral, shiftoverknot_T
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from casadi import *

vmax = 5
xT = 3

# create splines
degree = 3
knots = np.r_[np.zeros(degree), np.linspace(0, 1, 11), np.ones(degree)]
basis = BSplineBasis(knots, degree)
x_cfs = MX.sym('x_cfs', len(basis))
x = BSpline(basis, x_cfs) # motion trajectory
s_cfs = MX.sym('s_cfs', len(basis))
s = BSpline(basis, s_cfs) # slack spline
dx = x.derivative()

x0 = MX.sym('x0')
v0 = MX.sym('v0')

# build up optimization problem
con, lb, ub = [], [], []
con.append(dx - vmax)
lb += [-inf for _ in range(len(basis)-1)]
ub += [0 for _ in range(len(basis)-1)]
con.append(-dx - vmax)
lb += [-inf for _ in range(len(basis)-1)]
ub += [0 for _ in range(len(basis)-1)]
con.append(x-xT - s)
lb += [-inf for _ in range(len(basis))]
ub += [0 for _ in range(len(basis))]
con.append(-(x-xT) -s)
lb += [-inf for _ in range(len(basis))]
ub += [0 for _ in range(len(basis))]
con = [c.coeffs for c in con] # spline relaxation
con.append(x(0) - x0)
lb.append(0)
ub.append(0)
con.append(x(1) - xT) # watch out with this on -> can be infeasible when T too small. You can also leave this out
lb.append(0)
ub.append(0)
con.append(dx(0) - v0)
lb.append(0)
ub.append(0)
con.append(dx(1))
lb.append(0)
ub.append(0)

obj = definite_integral(s, 0, 1)

nlp = {'x': vertcat(x_cfs, s_cfs), 'p': vertcat(x0, v0), 'f': obj, 'g': vertcat(*con)}
solver = nlpsol('solver', 'ipopt', nlp)
solver2 = nlpsol('solver', 'blocksqp', nlp, {'verbose':True, 'qp_init': False, 'hess_lim_mem': 0, 'print_header': False})

# solve it
sol = solver(lbg=lb, ubg=ub, p=vertcat(0,0))
var1 = sol['x']
dual_var1 = sol['lam_g']

sol = solver2(x0=var1, lam_g0=dual_var1, lbg=lb, ubg=ub, p=vertcat(0, 0))

var = sol['x']
dual_var = sol['lam_g']

x_cfs = var[:len(basis)]
s_cfs = var[len(basis):]
basis = BSplineBasis(knots, degree)
x = BSpline(basis, x_cfs)
s = BSpline(basis, s_cfs)
dx = x.derivative()

lam1 = np.array(dual_var[:len(basis)-1])
lam2 = np.array(dual_var[len(basis)-1:2*len(basis)-2])
lam3 = np.array(dual_var[2*len(basis)-2:3*len(basis)-2])
lam4 = np.array(dual_var[3*len(basis)-2:4*len(basis)-2])
lamrest = np.array(dual_var[4*len(basis)-2:])
con1 = (dx - vmax).coeffs
con2 = (-dx - vmax).coeffs
con3 = (x-xT - s).coeffs
con4 = (-(x-xT) -s).coeffs

for k in range(len(basis)-1):
    print lam1[k]*con1[k]
    print lam2[k]*con2[k]
    print lam3[k]*con3[k]
    print lam4[k]*con4[k]

T1 = shiftoverknot_T(dx.basis)
T2 = shiftoverknot_T(basis)

lam1_tf = T1.dot(lam1)
lam2_tf = T1.dot(lam2)
lam3_tf = T2.dot(lam3)
lam4_tf = T2.dot(lam4)

con1_tf = T1.dot(con1)
con2_tf = T1.dot(con2)
con3_tf = T2.dot(con3)
con4_tf = T2.dot(con4)

print 'here'

for k in range(len(basis)-1):
    print lam1_tf[k]*con1_tf[k]
    print lam2_tf[k]*con2_tf[k]
    print lam3_tf[k]*con3_tf[k]
    print lam4_tf[k]*con4_tf[k]


x_cfs_tf = T2.dot(x_cfs)
s_cfs_tf = T2.dot(s_cfs)
var0 = vertcat(x_cfs_tf, s_cfs_tf)

dual_var0 = vertcat(lam1_tf, lam2_tf, lam3_tf, lam4_tf, lamrest)


sol = solver2(x0=var0, lam_g0=dual_var0, lbg=lb, ubg=ub, p=vertcat(x(0.1), dx(0.1)))
var = sol['x']
dual_var = np.array(sol['lam_g'])
x_cfs = var[:len(basis)]
s_cfs = var[len(basis):]
basis = BSplineBasis(knots, degree)
x = BSpline(basis, x_cfs)
s = BSpline(basis, s_cfs)
dx = x.derivative()

x2 = BSpline(basis, x_cfs_tf)
dx2 = x2.derivative()
s2 = BSpline(basis, s_cfs_tf)


for k in range(dual_var.shape[0]):
    print str(dual_var[k]) + " vs " + str(dual_var0[k])

t = np.linspace(0, 1, 1001)

plt.figure()
plt.subplot(2,1,1)
plt.plot(t, x(t), t, x2(t))
plt.xlabel('t (s)')
plt.ylabel('x (m)')
plt.subplot(2,1,2)
plt.plot(t, dx(t), t, dx2(t))
plt.xlabel('t (s)')
plt.ylabel('v (m/s)')

plt.figure()
plt.plot(t, s(t), t, s2(t))

plt.show(block=True)
