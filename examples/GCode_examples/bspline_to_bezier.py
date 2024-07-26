import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import BSpline
from gcode_helpers import bspline_to_bezier, plot_bezier_curves

knots_x = [0., 0., 0., 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1., 1., 1., 1.]
coeffs_x = [-3.49996757, -3.76937575, -4.45035443, -4.6201482,  -3.93826761, -2.40472289,
            -0.01954526,  2.37387283,  3.91565799,  4.60577904,  4.44422619,  3.76874982,
             3.49994937]
knots_y = [0., 0., 0., 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1., 1., 1., 1.]
coeffs_y = [44.75740819, 45.70061656, 47.34243535, 49.93532838, 52.02680779, 53.26709532,
            53.65593375, 53.19325628, 51.87913104, 49.71383664, 47.5120274 , 45.71360231,
            44.75744556]

spline_x = BSpline(knots_x, coeffs_x, 3)
spline_y = BSpline(knots_y, coeffs_y, 3)
xs = np.linspace(0, 1, 1000)

def bezier(t, X0, I0, P0, X1):
    return (1-t)**3*X0 + 3*(1-t)**2*t*I0 + 3*(1-t)*t**2*P0 + t**3*X1

# Example usage
bezier_control_points_x = bspline_to_bezier(knots_x, coeffs_x, 3)
bezier_control_points_y = bspline_to_bezier(knots_y, coeffs_y, 3)
x = plot_bezier_curves(bezier_control_points_x)
y = plot_bezier_curves(bezier_control_points_y)

plt.figure()
plt.subplot(121)
plt.plot([spline_x(x) for x in xs], [spline_y(x) for x in xs], 'g')
plt.plot(coeffs_x[0], coeffs_y[0], 'rx')
plt.plot(coeffs_x[-1], coeffs_y[-1], 'ro')

plt.plot(x, y, 'g.')
for B_x, By in zip(bezier_control_points_x, bezier_control_points_y):
    plt.plot(B_x[0], By[0], 'bx')
    plt.plot(B_x[-1], By[-1], 'bo')
plt.xlabel('x')
plt.ylabel('y')
plt.axis('equal')
plt.title('Bezier Curves')
plt.grid(True)

gcodes = ['G0 X' + str(bezier_control_points_x[0][0]) + ' Y' + str(bezier_control_points_y[0][0])]

plt.subplot(122)
plt.axis('equal')
for i, (B_x, B_y) in enumerate(zip(bezier_control_points_x, bezier_control_points_y)):
    t = np.linspace(0, 1, 100)
    bezier_x = bezier(t, B_x[0], B_x[1], B_x[2], B_x[3])
    bezier_y = bezier(t, B_y[0], B_y[1], B_y[2], B_y[3])
    plt.plot(bezier_x, bezier_y, 'g')

    if i == 0:
        X0, Y0 = B_x[0], B_y[0]
        I0, J0 = B_x[1], B_y[1]
        P0, Q0 = B_x[2], B_y[2]
        X1, Y1 = B_x[3], B_y[3]
        dI0 = I0 - X0
        dJ0 = J0 - Y0
        dP0 = P0 - X1
        dQ0 = Q0 - Y1
        gcodes.append('G5 I' + str(dI0) + ' J' + str(dJ0) + 
                      ' P' + str(dP0) + ' Q' + str(dQ0) + 
                      ' X' + str(X1) + ' Y' + str(Y1))
    else:
        P1 = B_x[2]
        Q1 = B_y[2]
        X2 = B_x[3]
        Y2 = B_y[3]
        dP1 = P1 - X2
        dQ1 = Q1 - Y2
        gcodes.append('G5 P' + str(dP1) + ' Q' + str(dQ1) + 
                      ' X' + str(X2) + ' Y' + str(Y2))

for gcode in gcodes:
    values = {}
    for command in gcode.split(' '):
        key = command[0]
        value = float(command[1:])
        if key != 'G':
            values[key] = value

    t = np.linspace(0, 1, 10)
    if len(values) == 2:
        X0, Y0 = values['X'], values['Y']
        plt.plot(X0, Y0, 'ro')
    elif len(values) == 6:
        dI0, dJ0 = values['I'], values['J']
        I0, J0 = X0 + dI0, Y0 + dJ0
        X1, Y1 = values['X'], values['Y']
        dP0, dQ0 = values['P'], values['Q']
        P0, Q0 = X1 + dP0, Y1 + dQ0
        plt.plot(bezier(t, X0, I0, P0, X1), bezier(t, Y0, J0, Q0, Y1), 'b.')
        X2, Y2 = X1, Y1
        dP1, dQ1 = dP0, dQ0
    else:
        X1, Y1 = X2, Y2
        dP0, dQ0 = dP1, dQ1
        I1, J1 = X1 - dP0, Y1 - dQ0
        X2, Y2 = values['X'], values['Y']
        dP1, dQ1 = values['P'], values['Q']
        P1, Q1 = X2 + dP1, Y2 + dQ1
        plt.plot(bezier(t, X1, I1, P1, X2), bezier(t, Y1, J1, Q1, Y2), 'b.')
    
plt.show()




