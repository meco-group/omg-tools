
def bspline_to_bezier(knots, coeffs, degree):
    num_knots = len(knots)
    num_coeffs = len(coeffs)
    n = num_coeffs
    
    bezier_control_points = []
    for i in range(3, n):
        P0 = coeffs[i - 3]
        P1 = coeffs[i - 2]
        P2 = coeffs[i - 1]
        P3 = coeffs[i]

        if i == 3:
            B0 = (1*P0                     ) / 1
            B1 = (       1*P1              ) / 1
            B2 = (       1*P1 + 1*P2       ) / 2
            B3 = (       3*P1 + 7*P2 + 2*P3) / 12
        elif i == 4:
            B0 = (3*P0 + 7*P1 + 2*P2       ) / 12
            B1 = (       2*P1 + 1*P2       ) / 3
            B2 = (       1*P1 + 2*P2       ) / 3
            B3 = (       1*P1 + 4*P2 + 1*P3) / 6
        elif i == n - 2:
            B0 = (1*P0 + 4*P1 + 1*P2       ) / 6
            B1 = (       2*P1 + 1*P2       ) / 3
            B2 = (       1*P1 + 2*P2       ) / 3
            B3 = (       2*P1 + 7*P2 + 3*P3) / 12
        elif i == n - 1:
            B0 = (2*P0 + 7*P1 + 3*P2       ) / 12
            B1 = (       1*P1 + 1*P2       ) / 2
            B2 = (              1*P2       ) / 1
            B3 = (                     1*P3) / 1
        else:
            B0 = (1*P0 + 4*P1 + 1*P2       ) / 6
            B1 = (       2*P1 + 1*P2       ) / 3
            B2 = (       1*P1 + 2*P2       ) / 3
            B3 = (       1*P1 + 4*P2 + 1*P3) / 6

        bezier_control_points.append([B0, B1, B2, B3])
    return bezier_control_points

def get_bezier_full(pos_splines):
    x_bezier_full, y_bezier_full = [], []
    for splines in pos_splines:
        x_coeffs = splines[0].coeffs
        y_coeffs = splines[1].coeffs
        x_knots = splines[0].basis.knots
        y_knots = splines[1].basis.knots
        x_beziers = bspline_to_bezier(x_knots, x_coeffs, 3)
        y_beziers = bspline_to_bezier(y_knots, y_coeffs, 3)
        for x_bezier, y_bezier in zip(x_beziers, y_beziers):
            x_bezier_full.append(x_bezier)
            y_bezier_full.append(y_bezier)
    return x_bezier_full, y_bezier_full

def write_to_file_G01(x_sol, y_sol, filename):
    file = open(filename, 'w')
    x_prev = round(x_sol[0], 3)
    y_prev = round(y_sol[0], 3)
    file.write(f'G00 X{x_prev} Y{y_prev}\n')
    for x, y in zip(x_sol, y_sol):
        x_new = round(x, 3)
        y_new = round(y, 3)
        if x_new != x_prev or y_new != y_prev:
            file.write(f'G01 X{x_new} Y{y_new}\n')
        x_prev = x_new
        y_prev = y_new
    file.close()

def write_to_file_G05(x_bezier_full, y_bezier_full, filename):
    file = open(filename, 'w')
    file.write(f'G0 X{round(x_bezier_full[0][0], 3)} Y{round(y_bezier_full[0][0], 3)}\n')
    for i, (B_x, B_y) in enumerate(zip(x_bezier_full, y_bezier_full)):
        if i == 0:
            X0, Y0 = B_x[0], B_y[0]
            I0, J0 = B_x[1], B_y[1]
            P0, Q0 = B_x[2], B_y[2]
            X1, Y1 = B_x[3], B_y[3]
            dI0 = I0 - X0
            dJ0 = J0 - Y0
            dP0 = P0 - X1
            dQ0 = Q0 - Y1
            file.write(f'G5 I{round(dI0, 3)} J{round(dJ0, 3)} P{round(dP0, 3)} Q{round(dQ0, 3)} X{round(X1, 3)} Y{round(Y1, 3)}\n') 
        else:
            P1 = B_x[2]
            Q1 = B_y[2]
            X2 = B_x[3]
            Y2 = B_y[3]
            dP1 = P1 - X2
            dQ1 = Q1 - Y2
            file.write(f'G5 P{round(dP1, 3)} Q{round(dQ1, 3)} X{round(X2, 3)} Y{round(Y2, 3)}\n') 
    file.close()