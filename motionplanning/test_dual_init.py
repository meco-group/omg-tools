from spline_extra import *
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import numpy as np
import time
from casadi import *
from casadi.tools import *
import time

degree = 3
T = 10
knots = np.r_[np.zeros(degree), np.linspace(0., 1, 11), np.ones(degree)]
basis = BSplineBasis(knots, degree)
t_ax = np.linspace(0., T, 500)

vmax, amax = 0.5, 1.
v0 = 0.
x0, xT = 0., 5.
a0 = None

mu = 0.1
lam_g0 = None
var0 = None

update_time = (knots[degree+1] - knots[0])*T/10.
current_time = 0.
for k in range(2):
    var_struct = struct([entry('x', shape=len(basis)),
                         entry('g', shape=len(basis))])

    var = struct_symMX(var_struct)
    cfs_x, cfs_g = var['x'], var['g']
    x, g = BSpline(basis, cfs_x), BSpline(basis, cfs_g)
    v, a = x.derivative(), x.derivative(2)

    obj = g.integral()
    constr = []
    constr.append( x - xT - g)
    constr.append(-x + xT - g)
    constr.append( v - T*vmax)
    constr.append(-v - T*vmax)
    constr.append( a - (T**2)*amax)
    constr.append(-a - (T**2)*amax)
    constr.append(x(current_time/T) - x0)
    constr.append(v(current_time/T) - T*v0)

    con_entries = []
    spline_con = {}
    for ind, c in enumerate(constr):
        lbl = 'c'+str(ind)
        if isinstance(c, BSpline):
            spline_con[lbl] = c
            con_entries.append(entry(lbl, expr = c.coeffs))
        else:
            con_entries.append(entry(lbl, expr = c))

    con_struct = struct(con_entries)
    con = struct_MX(con_entries)

    nlp = MXFunction('nlp', nlpIn(x=var), nlpOut(f=obj, g=(con)))
    ubg = con(0.)
    lbg = con(-inf)
    for key in ['c6', 'c7']:
        lbg[key] = 0.
        n_eq = 3
    else:
        n_eq = 2

    if var0 is None:
        var0 = var(0)
    if lam_g0 is None:
        lam_g0 = con(0)

    options = {'linear_solver': 'ma57', 'warm_start_init_point': 'yes',
               'print_level': 5, 'print_time': 0, 'mu_init': mu, 'tol': 1e-3}
    solver = NlpSolver('solver', 'ipopt', nlp, options)
    options = {'lbg': lbg, 'ubg': ubg, 'x0': var0, 'lam_g0': lam_g0}
    t0 = time.time()
    sol = solver(options)
    t1 = time.time()

    stats = solver.getStats()
    if stats.get("return_status") != "Solve_Succeeded":
        print stats.get("return_status")
    print 'It %d: solved in %d it and %f ms' % (k, stats.get('iter_count'), (t1-t0)*1e3)

    # shift splines
    var_res = var_struct(sol['x'])
    cfs_x, cfs_g = var_res['x'].toArray(), var_res['g'].toArray()
    cfs_x2 = shift_knot1_fwd(cfs_x, knots, degree, update_time/T)
    cfs_g2 = shift_knot1_fwd(cfs_g, knots, degree, update_time/T)

    x_ = BSpline(basis, cfs_x)
    v_ = (1./T)*x_.derivative()
    a_ = (1./(T**2))*x_.derivative(2)



    # new spline basis
    current_time += update_time

    knots2 = np.r_[(current_time/T)*np.ones(degree+1), knots[degree+1:]]
    basis = BSplineBasis(knots2, degree)
    x = BSpline(basis, cfs_x2)
    v, a = (1./T)*x.derivative(), (1./(T**2))*x.derivative(2)
    x0 = x(current_time/T)
    v0 = v(current_time/T)

    # t_ax2 = t_ax[t_ax >= current_time]
    # plt.figure()
    # plt.subplot(311)
    # plt.plot(t_ax, x_(t_ax/T), 'b', t_ax2, x(t_ax2/T), 'r')
    # plt.subplot(312)
    # plt.plot(t_ax, v_(t_ax/T), 'b', t_ax2, v(t_ax2/T), 'r')
    # plt.subplot(313)
    # plt.plot(t_ax, a_(t_ax/T), 'b', t_ax2, a(t_ax2/T), 'r')
    # t_ax = t_ax2
    # plt.show()

    # create initial data for next problem
    # primal variables
    var0 = var_struct(0)
    var0['x'] = cfs_x2
    var0['g'] = cfs_g2
    # var0 = sol['x'].toArray()
    # dual variables
    lam_g = con_struct(sol['lam_g'])
    con = con_struct(sol['g'])
    for key, spl in spline_con.items():
        spl_basis = spl.basis
        spl_knots = spl_basis.knots
        spl_degree = spl_basis.degree
        B = integral_sqbasis(spl_basis)
        C = np.linalg.solve(B, np.eye(len(spl_basis)))
        lam_g_old = lam_g[key].toArray()
        cfs_lam_g = C.dot(lam_g[key].toArray())
        cfs_lam_g2 = shift_knot1_fwd(cfs_lam_g, spl_knots, spl_degree, update_time/T)
        spl_knots2 = np.r_[(current_time/T)*np.ones(spl_degree+1), spl_knots[spl_degree+1:]]
        spl_basis2 = BSplineBasis(spl_knots2, spl_degree)
        B = integral_sqbasis(spl_basis2)
        lam_g_new = B.dot(cfs_lam_g2)
        lam_g[key] = lam_g_new

        con_cfs = con[key].toArray()
        con_cfs2 = shift_knot1_fwd(con_cfs, spl_knots, spl_degree, update_time/T)
        con[key] = con_cfs2
        # test
        # c1 = BSpline(spl_basis, con_cfs)
        # c2 = BSpline(spl_basis2, con_cfs2)
        # l1 = BSpline(spl_basis, cfs_lam_g)
        # l2 = BSpline(spl_basis2, cfs_lam_g2)
        # res1 = definite_integral(l1*c1, current_time/T, 1.)
        # res2 = (l2*c2).integral().ravel()[0]
        # res3 = lam_g_new.T.dot(con_cfs2).ravel()[0]
        # print res1
        # print res2
        # print res3

    lam_g0 = lam_g.cat.toArray()
    con0 = con.cat.toArray()
    mu = np.mean(-np.multiply(abs(lam_g0), con0)[:-n_eq])

    # lam_g0 = sol['lam_g'].toArray()
    # con = sol['g'].toArray()
    # mu = np.mean(-np.multiply(lam_g0, con)[:-n_eq])
