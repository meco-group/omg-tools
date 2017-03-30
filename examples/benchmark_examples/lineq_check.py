import numpy as np
import meco_binaries; meco_binaries(cpp_splines='develop')
import splines as spl
import sys, os
sys.path.insert(0, os.getcwd()+'/../..')
import omgtools.basics.spline_extra as spl_ex
import omgtools.basics.spline_old as spl_ol
from casadi import *

# new splines
degree = 3
knots = np.r_[np.zeros(degree), np.linspace(0, 1, 11), np.ones(degree)]
basis = spl.BSplineBasis(knots, degree)

x_cfs = MX.sym('x_cfs', basis.dimension())
y_cfs = MX.sym('y_cfs', basis.dimension())
z_cfs = MX.sym('z_cfs', basis.dimension())

x = spl.Function(basis, x_cfs)
y = spl.Function(basis, y_cfs)
z = spl.Function(basis, z_cfs)

expr = 2*x + 5*y.antiderivative() + 6*z.derivative()
expr_cfs = expr.data()

# doel: check of expr_cfs is lineair in x_cfs, y_cfs, z_cfs
# vroeger werkende aanpak: jacobiaan berekenen en kijken naar symvar -> zou leeg moeten zijn

jac = jacobian(expr_cfs, vertcat(x_cfs, y_cfs, z_cfs))

print symvar(jac) # niet leeg :(

# old splines
basis = spl_ol.BSplineBasis(knots, degree)
x = spl_ol.BSpline(basis, x_cfs)
y = spl_ol.BSpline(basis, y_cfs)
z = spl_ol.BSpline(basis, z_cfs)

expr = 2*x + 5*spl_ex.running_integral(y) + 6*z.derivative()
expr_cfs = expr.coeffs

jac = jacobian(expr_cfs, vertcat(x_cfs, y_cfs, z_cfs))

print symvar(jac) # leeg :)
