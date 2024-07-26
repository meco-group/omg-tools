import numpy as np
import matplotlib.pyplot as plt

class BSpline:
    def __init__(self, degree, knots):
        self.degree = degree
        self.knots = knots
    
    def _ind(self, i, x):
        """Indicator function for the zeroth degree basis function."""
        k = self.knots
        return np.where((k[i] <= x) & (x < k[i+1]), 1.0, 0.0)
    
    def eval_basis(self, x):
        """Evaluate the B-spline basis functions at x using the Cox-de Boor recursion formula."""
        x = np.array(x)
        k = self.knots
        basis = [[self._ind(i, x) for i in range(len(k) - 1)]]
        
        for d in range(1, self.degree + 1):
            basis.append([])
            for i in range(len(k) - d - 1):
                b = 0.0
                bottom = k[i + d] - k[i]
                if bottom != 0:
                    b = (x - k[i]) * basis[d - 1][i] / bottom
                bottom = k[i + d + 1] - k[i + 1]
                if bottom != 0:
                    b += (k[i + d + 1] - x) * basis[d - 1][i + 1] / bottom
                basis[-1].append(b)
        
        return np.array(basis[-1])

# Example usage
degree = 3
knots = [0., 0., 0., 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1., 1., 1., 1.]
bspline = BSpline(degree, knots)

x = np.linspace(0, 1, 1000)
basis_functions = bspline.eval_basis(x)

# Plotting the basis functions
plt.figure(figsize=(10, 6))
for knot in knots:
    plt.axvline(knot, color='k', linestyle='--', linewidth=0.5)
for i in range(len(basis_functions)):
    plt.plot(x, basis_functions[i], label=f'N_{i},{degree}')

def mon_to_bezier(x0, x1, x2, x3):
    B0 = x0
    B1 = (x1 + 3*B0)/3
    B2 = (x2 - 3*B0 + 6*B1)/3
    B3 = x3 + B0 - 3*B1 + 3*B2
    return B0, B1, B2, B3

u_0_1 = np.linspace(0, 1, 100)
u_1_2 = np.linspace(1, 2, 100)
x_0_01 = np.linspace(0., .1, 100)
x_01_02 = np.linspace(.1, .2, 100)
x_08_09 = np.linspace(.8, .9, 100)
x_09_1 = np.linspace(.9, 1., 100)

plt.plot(x_0_01, [(1-u)**3 for u in u_0_1], 'r.')
print(mon_to_bezier(1, -3, 3, -1))
plt.plot(x_0_01, [7/4*u**3 - 9/2*u**2 + 3*u for u in u_0_1], 'r.')
print(mon_to_bezier(0, 3, -9/2, 7/4))
plt.plot(x_0_01, [-11/12*u**3 + 3/2*u**2 for u in u_0_1], 'r.')
print(mon_to_bezier(0, 0, 3/2, -11/12))
plt.plot(x_0_01, [1/6*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(0, 0, 0, 1/6))

plt.plot(x_01_02, [-1/4*u**3 + 3/2*u**2 - 3*u + 2 for u in u_1_2], 'r.')
plt.plot(x_01_02, [-1/4*u**3 + 3/4*u**2 - 3/4*u + 1/4 for u in u_0_1], 'r.')
print(mon_to_bezier(1/4, -3/4, 3/4, -1/4))
plt.plot(x_01_02, [7/12*u**3 - 3*u**2 + 9/2*u - 3/2 for u in u_1_2], 'r.')
plt.plot(x_01_02, [7/12 + u/4 - 5/4*u**2 + 7/12*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(7/12, 1/4, -5/4, 7/12))
plt.plot(x_01_02, [-1/2*u**3 + 2*u**2 - 2*u + 2/3 for u in u_1_2], 'r.')
plt.plot(x_01_02, [1/6 + u/2 + 1/2*u**2 - 1/2*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(1/6, 1/2, 1/2, -1/2))
plt.plot(x_01_02, [-1/6 + 1/2*u - 1/2*u**2 + 1/6*u**3 for u in u_1_2], 'r.')
plt.plot(x_01_02, [1/6*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(0, 0, 0, 1/6))

plt.plot(x_08_09, [1/6 - 1/2*u + 1/2*u**2 - 1/6*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(1/6, -1/2, 1/2, -1/6))
plt.plot(x_08_09, [2/3 - u**2 + 1/2*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(2/3, 0, -1, 1/2))
plt.plot(x_08_09, [1/6 + 1/2*u + 1/2*u**2 - 7/12*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(1/6, 1/2, 1/2, -7/12))
plt.plot(x_08_09, [1/4*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(0, 0, 0, 1/4))

plt.plot(x_09_1, [1/6 - 1/2*u + 1/2*u**2 - 1/6*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(1/6, -1/2, 1/2, -1/6))
plt.plot(x_09_1, [7/12 - 1/4*u - 5/4*u**2 + 11/12*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(7/12, -1/4, -5/4, 11/12))
plt.plot(x_09_1, [1/4 + 3/4*u + 3/4*u**2 - 7/4*u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(1/4, 3/4, 3/4, -7/4))
plt.plot(x_09_1, [u**3 for u in u_0_1], 'r.')
print(mon_to_bezier(0, 0, 0, 1))

plt.title('Cubic B-Spline Basis Functions')
plt.xlabel('x')
plt.ylabel('N_{i,3}(x)')
plt.legend()
plt.grid(True)
plt.show()
