import meco_binaries; meco_binaries(cpp_splines='develop')
import splines as spl
import spline_old as splold
import numpy as np
import casadi as cas
from scipy.interpolate import splev
import matplotlib.pyplot as plt
import time

# p2p bicycle stand-alone example
# -------------------------------

# toggle switch
erik = True

# settings
x0, y0, theta0, delta0 = 0., 0., 0., 0.
xT, yT, thetaT, deltaT = 3., 3., 0., 0.
vmax = 0.8
amax = 1.
dmax = np.pi/6.
dmin = -np.pi/6.
ddmax = np.pi/4.
ddmin = -np.pi/4.
length = 0.4
obst_x, obst_y, obst_r = 1., 1., 0.5

# parameters
t = cas.MX.sym('t')
v_til0 = cas.MX.sym('v_til0')
tg_ha0 = cas.MX.sym('tg_ha0')
dtg_ha0 = cas.MX.sym('dtg_ha0')
hop0 = cas.MX.sym('hop0')
tdelta0 = cas.MX.sym('tdelta0')
pos0 = cas.MX.sym('pos0', 2)
v_tilT = cas.MX.sym('v_tilT')
dv_tilT = cas.MX.sym('dv_tilT')
tg_haT = cas.MX.sym('tg_haT')
dtg_haT = cas.MX.sym('dtg_haT')
ddtg_haT = cas.MX.sym('ddtg_haT')
posT = cas.MX.sym('posT', 2)

par = cas.vertcat(t, v_til0, tg_ha0, dtg_ha0, hop0, tdelta0, pos0,
                  v_tilT, dv_tilT, tg_haT, dtg_haT, ddtg_haT, posT)

# variables
degree = 2
knintervals = 5
knots = np.r_[np.zeros(degree), np.linspace(0, 1, knintervals+1), np.ones(degree)]
knots_hp = np.r_[np.zeros(1), np.linspace(0, 1, knintervals+1), np.ones(1)]
if erik:
    basis = spl.BSplineBasis(knots, degree)
    basis_hp = spl.BSplineBasis(knots, degree)
else:
    basis = splold.BSplineBasis(knots, degree)
    basis_hp = splold.BSplineBasis(knots_hp, 1)
cfsx = cas.MX.sym('cfsx', basis.dimension())
cfsy = cas.MX.sym('cfsy', basis.dimension())
cfs_ax = cas.MX.sym('cfs_ax', basis_hp.dimension())
cfs_ay = cas.MX.sym('cfs_ay', basis_hp.dimension())
cfs_b = cas.MX.sym('cfs_b', basis_hp.dimension())
T = cas.MX.sym('T')

var = cas.vertcat(cfsx, cfsy, cfs_ax, cfs_ay, cfs_b, T)

# construct derived splines
if erik:
    v_til = spl.Function(basis, cfsx)
    tg_ha = spl.Function(basis, cfsy)
    ax = spl.Function(basis_hp, cfs_ax)
    ay = spl.Function(basis_hp, cfs_ay)
    b = spl.Function(basis_hp, cfs_b)
else:
    v_til = splold.BSpline(basis, cfsx)
    tg_ha = splold.BSpline(basis, cfsy)
    ax = splold.BSpline(basis_hp, cfs_ax)
    ay = splold.BSpline(basis_hp, cfs_ay)
    b = splold.BSpline(basis_hp, cfs_b)
dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
ddtg_ha = tg_ha.derivative(2)
dx = v_til*(1-tg_ha**2)
dy = v_til*(2*tg_ha)
dx_int = T*dx.antiderivative()
dy_int = T*dy.antiderivative()
x = dx_int - dx_int(t/T) + pos0[0]
y = dy_int - dy_int(t/T) + pos0[1]

# constraints on spline domain
con = []
con.append(-v_til)

con.append(v_til*(1+tg_ha**2) - vmax)
con.append(dv_til*(1+tg_ha**2) + 2*v_til*tg_ha*dtg_ha - T*amax)
con.append(2*dtg_ha*length - v_til*((1+tg_ha**2)**2)*np.tan(dmax)*T)
con.append(-2*dtg_ha*length + v_til*((1+tg_ha**2)**2)*np.tan(dmin)*T)
con.append(2*length*ddtg_ha*(v_til*(1+tg_ha**2)**2) -
           2*length*dtg_ha*(dv_til*(1+tg_ha**2)**2 +
           v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) - ((T**2)*v_til**2*(1+tg_ha**2)**4 +
           (2*length*dtg_ha)**2)*ddmax)
con.append(-2*length*ddtg_ha*(v_til*(1+tg_ha**2)**2) +
           2*length*dtg_ha*(dv_til*(1+tg_ha**2)**2 +
           v_til*(4*tg_ha+4*tg_ha**3)*dtg_ha) + ((T**2)*v_til**2*(1+tg_ha**2)**4 +
           (2*length*dtg_ha)**2)*ddmin)
con.append(ax*ax + ay*ay - 1)
con.append(-(ax*obst_x + ay*obst_y) + b + obst_r)
con.append(ax*x + ay*y - b + length/2.)

if erik:
    con = [c.data() for c in con]
else:
    con = [c.coeffs for c in con]

lbg = [-cas.inf for _ in range(cas.vertcat(*con).shape[0])]
ubg = [0 for _ in range(cas.vertcat(*con).shape[0])]

# scalar constraints
con.append(-T)
lbg += [-cas.inf]
ubg += [0]
con.append(hop0*(2.*ddtg_ha(t/T)*length - tdelta0*(dv_til(t/T)*(1.+tg_ha0**2)**2)*T))
con.append(v_til(t/T) - v_til0)
con.append(tg_ha(t/T) - tg_ha0)
con.append(dtg_ha(t/T) - T*dtg_ha0)
con.append(x(1) - posT[0])
con.append(y(1) - posT[1])
con.append(tg_ha(1) - tg_haT)
con.append(v_til(1) - v_tilT)
con.append(dtg_ha(1) - T*dtg_haT)
con.append(dv_til(1) - dv_tilT)
con.append(ddtg_ha(1) - T**2*ddtg_haT)

lbg += [0 for _ in range(11)]
ubg += [0 for _ in range(11)]

# construct nlp
nlp = {'x': var, 'p': par, 'f': T, 'g': cas.vertcat(*con)}
solver = cas.nlpsol('solver', 'ipopt', nlp, {'ipopt.tol': 1e-3, 'ipopt.linear_solver': 'ma57', 'ipopt.warm_start_init_point': 'yes', 'expand': True})

# let's roll...
var0 = np.zeros(var.shape[0])
len_basis = basis.dimension()
var0[len_basis:2*len_basis] = np.linspace(np.tan(0.5*theta0), np.tan(0.5*thetaT), len_basis)
var0[-1] = 10.
par0 = np.r_[0., 0., np.tan(0.5*theta0), 0., 1., np.tan(delta0), x0, y0, 0., 0., np.tan(0.5*thetaT), 0., 0., xT, yT]
# t, v_til0, tg_ha0, dtg_ha0, hop0, tdelta0, pos0, v_tilT, dv_tilT, tg_haT, dtg_haT, ddtg_haT, posT
t0_ = time.time()
sol = solver(x0=var0, p=par0, lbg=lbg, ubg=ubg)
t1_ = time.time()
print 'it took ' + str(t1_-t0_) + 's'

# extract results & splines
var = sol['x']
cfsx = np.array(var[:len_basis]).ravel()
cfsy = np.array(var[len_basis:2*len_basis]).ravel()
T = float(var[-1])
knots2 = np.r_[np.zeros(degree), np.linspace(0, T, knintervals+1), T*np.ones(degree)]
if erik:
    basis2 = spl.BSplineBasis(knots2, degree)
    v_til = spl.Function(basis2, cfsx)
    tg_ha = spl.Function(basis2, cfsy)
else:
    basis2 = splold.BSplineBasis(knots2, degree)
    v_til = splold.BSpline(basis2, cfsx)
    tg_ha = splold.BSpline(basis2, cfsy)
dv_til, dtg_ha = v_til.derivative(), tg_ha.derivative()
ddtg_ha = tg_ha.derivative(2)
dx = v_til*(1-tg_ha**2)
dy = v_til*(2*tg_ha)
dx_int = dx.antiderivative()
dy_int = dy.antiderivative()
x = dx_int - dx_int(0) + x0
y = dy_int - dy_int(0) + y0

# sample splines
time = np.linspace(0, T, 501)
if erik:
    tg_ha_s = splev(time, (tg_ha.basis().knots(), tg_ha.data(), tg_ha.basis().degree()))
    v_til_s = splev(time, (v_til.basis().knots(), v_til.data(), v_til.basis().degree()))
    dtg_ha_s = splev(time, (dtg_ha.basis().knots(), dtg_ha.data(), dtg_ha.basis().degree()))
    dv_til_s = splev(time, (dv_til.basis().knots(), dv_til.data(), dv_til.basis().degree()))
    ddtg_ha_s = splev(time, (ddtg_ha.basis().knots(), ddtg_ha.data(), ddtg_ha.basis().degree()))
    x_s = splev(time, (x.basis().knots(), x.data(), x.basis().degree()))
    y_s = splev(time, (y.basis().knots(), y.data(), y.basis().degree()))
else:
    tg_ha_s = splev(time, (tg_ha.basis.knots, tg_ha.coeffs, tg_ha.basis.degree))
    v_til_s = splev(time, (v_til.basis.knots, v_til.coeffs, v_til.basis.degree))
    dtg_ha_s = splev(time, (dtg_ha.basis.knots, dtg_ha.coeffs, dtg_ha.basis.degree))
    dv_til_s = splev(time, (dv_til.basis.knots, dv_til.coeffs, dv_til.basis.degree))
    ddtg_ha_s = splev(time, (ddtg_ha.basis.knots, ddtg_ha.coeffs, ddtg_ha.basis.degree))
    x_s = splev(time, (x.basis.knots, x.coeffs, x.basis.degree))
    y_s = splev(time, (y.basis.knots, y.coeffs, y.basis.degree))
theta_s = 2*np.arctan2(tg_ha_s, 1)
v_s = v_til_s*(1+tg_ha_s**2)
delta_s = np.arctan2(2*dtg_ha_s*length, v_til_s*(1+tg_ha_s**2)**2)
ddelta_s = (2*ddtg_ha_s*length*(v_til_s*(1+tg_ha_s**2)**2)-2*dtg_ha_s*length*(dv_til_s*(1+tg_ha_s**2)**2 + v_til_s*(4*tg_ha_s+4*tg_ha_s**3)*dtg_ha_s))/(v_til_s**2*(1+tg_ha_s**2)**4+(2*dtg_ha_s*length)**2)

# check if you needed to use l'Hopital's rule to find delta and ddelta above, if so adapt signals
# start
if (v_til_s[0] <= 1e-4 and dtg_ha_s[0] <= 1e-4):
    delta_s[0] = np.arctan2(2*ddtg_ha_s[0]*length, (dv_til_s[0]*(1+tg_ha_s[0]**2)**2))
    ddelta_s[0] = ddelta_s[1]  # choose next input
# middle
for k in range(1, len(time)-1):
    if (v_til_s[k] <= 1e-3 and dtg_ha_s[k] <= 1e-3):
        if (ddtg_ha_s[k] <= 1e-4 and dv_til_s[k] <= 1e-4):  # l'Hopital won't work
            delta_s[k] = delta_s[k-1]  # choose previous steering angle
        else:
            delta_s[k] = np.arctan2(2*ddtg_ha_s[0, k]*length, (dv_til_s[0, k]*(1+tg_ha_s[k]**2)**2))
        ddelta_s[k] = ddelta_s[k-1]  # choose previous input
# end
if (v_til_s[-1] <= 1e-4 and dtg_ha_s[-1] <= 1e-4):  # correct at end point
    delta_s[-1] = delta_s[-2]
    ddelta_s[-1] = ddelta_s[-2]

# plot plot plot
plt.figure()
plt.subplot(6, 1, 1)
plt.plot(time, x_s)
plt.xlabel('t (s)')
plt.ylabel('x (m)')
plt.subplot(6, 1, 2)
plt.plot(time, y_s)
plt.xlabel('t (s)')
plt.ylabel('y (m)')
plt.subplot(6, 1, 3)
plt.plot(time, theta_s)
plt.xlabel('t (s)')
plt.ylabel('theta (rad)')
plt.subplot(6, 1, 4)
plt.plot(time, delta_s)
plt.xlabel('t (s)')
plt.ylabel('delta (rad)')
plt.subplot(6, 1, 5)
plt.plot(time, v_s)
plt.xlabel('t (s)')
plt.ylabel('v (m/s)')
plt.subplot(6, 1, 6)
plt.plot(time, ddelta_s)
plt.xlabel('t (s)')
plt.ylabel('ddelta (rad/s)')

plt.show()
