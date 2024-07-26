
import numpy as np
import matplotlib.pyplot as plt

# Define the control points based on the G-code

# G0 X0 Y0
# G5 I0 J3 P0 Q-3 X1 Y1
# G5 P0 Q-3 X2 Y2
# G0 X0 Y0
# G5 dI0 dJ3 dP0 dQ-3 X1 Y1
# G5 dP0 dQ-3 X2 Y2

X0, Y0 = 0, 0
dI0, dJ0 = 0, 3
I0, J0 = X0 + dI0, Y0 + dJ0
X1, Y1 = 1, 1
dP0, dQ0 = 0, -3
P0, Q0 = X1 + dP0, Y1 + dQ0

I1, J1 = X1 - dP0, Y1 - dQ0
X2, Y2 = 2, 2
dP1, dQ1 = 0, -3
P1, Q1 = X2 + dP1, Y2 + dQ1

print(f'X0: {X0}, Y0: {Y0}')
print(f'I0: {I0}, J0: {J0}')
print(f'P0: {P0}, Q0: {Q0}')
print(f'X1: {X1}, Y1: {Y1}')

# Define the Bézier curve functions
def B0_x(t):
    return (1-t)**3*X0 + 3*(1-t)**2*t*I0 + 3*(1-t)*t**2*P0 + t**3*X1

def B0_y(t):
    return (1-t)**3*Y0 + 3*(1-t)**2*t*J0 + 3*(1-t)*t**2*Q0 + t**3*Y1

def B1_x(t):
    return (1-t)**3*X1 + 3*(1-t)**2*t*I1 + 3*(1-t)*t**2*P1 + t**3*X2

def B1_y(t):
    return (1-t)**3*Y1 + 3*(1-t)**2*t*J1 + 3*(1-t)*t**2*Q1 + t**3*Y2

# Generate points on the Bézier curve
t_values = np.linspace(0, 1, 100)
x0_values = B0_x(t_values)
y0_values = B0_y(t_values)
x1_values = B1_x(t_values)
y1_values = B1_y(t_values)

# Plot the Bézier curve
plt.plot(x0_values, y0_values, label='Bézier Curve')
plt.plot(x1_values, y1_values)
plt.plot([X0, I0, P0, X1], [Y0, J0, Q0, Y1], 'ro--', label='Control Points')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Cubic Bézier Curve')
plt.axis('equal')
plt.legend()
plt.grid(True)

plt.show()