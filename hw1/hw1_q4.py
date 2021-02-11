import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')

t = np.linspace(-10 * np.pi, 10 * np.pi, 100)

px = np.cos(0.1*t)
py = pz = np.sin(0.1*t)

c = 0.25
x = c * np.cos(t) + np.cos(0.1*t)
y = c * np.sin(t) + np.sin(0.1*t)
z = np.sin(0.1*t)
ax.plot(x, y, z, label='Ap')
ax.plot(px, py, pz, label='position')
ax.legend()
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_title('Plot of P and Robot Position in Frame A from [-Pi,Pi]')

plt.show()