import numpy as np
import matplotlib.pyplot as plt

# Parameters for Lissajous curve
A = 1  # Amplitude for x and y
a = 1  # Frequency for x
b = 2  # Frequency for y
delta = 0  # Phase shift

# Time values for plotting the curve
t = np.linspace(0, 2 * np.pi, 1000)

# Lissajous curve equations
x = A * np.sin(a * t + delta)
y = A * np.sin(b * t)

# Create the plot
plt.figure(figsize=(6,6))
plt.plot(x, y, label=r'$x = \sin(t), y = \sin(2t)$')
plt.title('Lissajous Curve: A=1, a=1, b=2')
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio
plt.legend()

# Save the plot as an image
plt.savefig('lissajous_curve.png')
plt.show()
