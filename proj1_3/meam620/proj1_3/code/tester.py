import numpy as np
import matplotlib.pyplot as plt
from flightsim.axes3ds import Axes3Ds

xx = [0]*200
xx2 = [0]*200
y = [0]*200
z = [0]*200

t1 = 1
t2 = 2
b_x = np.array([0,1,0,2,0,0,1,1,0,0,0,0])
b_y = np.array([0,0,0,2,1,0,1,1,0,0,0,0])
b_z = np.array([0,1,0,2,0,0,1,1,0,0,0,0])

A = np.array([[0,0,0,0,0,1,0,0,0,0,0,0],
              [0,0,0,0,1,0,0,0,0,0,0,0],
              [0,0,0,2,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,t2**5, t2**4, t2**3, t2**2, t2, 1],
              [0,0,0,0,0,0,5*t2**4, 4*t2**3, 3*t2**2, 2*t2, 1,0],
              [0,0,0,0,0,0,20*t2**3, 12*t2**2, 6*t2, 2, 0, 0],
              [t1**5, t1**4, t1**3, t1**2, t1, 1, 0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,1],
              [5*t1**4, 4*t1**3, 3*t1**2, 2*t1, 1, 0,0,0,0,0,-1,0],
              [20*t1**3, 12*t1**2, 6*t1, 2, 0,0,0,0,0,-2,0,0],
              [60*t1**2, 24*t1, 6, 0,0,0,0,0,-6,0,0,0],
              [120*t1, 24,0,0,0,0,0,-24,0,0,0,0]])
c_x = np.linalg.solve(A,b_x)
c_y = np.linalg.solve(A,b_y)
c_z = np.linalg.solve(A,b_z)

t_x = np.linspace(0,t1,100)

for j in range(0,2):
    for i, x in enumerate(t_x):
        print(j*6+i)
        xx[j*100+i] = c_x[j*6]*x**5 + c_x[j*6+1]*x**4 + c_x[j*6+2]*x**3 + c_x[j*6+3]*x**2 + c_x[j*6+4]*x + c_x[j*6+5]
        y[j*100+i] = c_y[j*6]*x**5 + c_y[j*6+1]*x**4 + c_y[j*6+2]*x**3 + c_y[j*6+3]*x**2 + c_y[j*6+4]*x + c_y[j*6+5]
        z[j*100+i] = c_z[j*6]*x**5 + c_z[j*6+1]*x**4 + c_z[j*6+2]*x**3 + c_z[j*6+3]*x**2 + c_z[j*6+4]*x + c_z[j*6+5]

print(xx)
# t_x = np.linspace(0,t,200)
fig = plt.figure('all')
ax = Axes3Ds(fig)
# ax = plt.axes()
plt.plot(xx,y, z, 'r.')
plt.xlabel('x')
plt.ylabel('y')
plt.show()