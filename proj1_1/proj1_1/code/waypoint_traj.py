import numpy as np
import math
from scipy import interpolate
import matplotlib.pyplot as plt

class WaypointTraj(object):
    """

    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE

        # self.total_time = 10
        # self.total_num_steps = self.total_time*503
        # self.x = []
        # self.x_dot = []
        # self.x_ddot = []
        #self.yaw = np.linspace(0,0,self.total_num_steps)
        self.v = 1
        self.unit_vecs = []
        self.duration = []
        self.points = points
        for i in range(len(points)-1):
            length = np.linalg.norm(points[i+1]-points[i])
            unit = (points[i+1]-points[i])/length
            self.unit_vecs.append(unit)
            if len(self.duration) == 0:
                self.duration.append(length / self.v)
            else:
                self.duration.append(length / self.v + self.duration[i-1])

        #self.compute_waypoints(points)

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0


        for i in range(len(self.duration)-1):
            if 0 <= t < self.duration[i]:
                x_dot = self.v * self.unit_vecs[i]
                x = [0,0,0] + x_dot * t
            elif self.duration[i] <= t < self.duration[i+1]:
                x_dot = self.v*self.unit_vecs[i+1]
                x = self.unit_vecs[i+1] + x_dot*(t-self.duration[i])
            else:
                x_dot = [0,0,0]
                x = self.points[i+2]

        print("X: "+str(x)+", Xdot: "+str(x_dot))
        # STUDENT CODE HERE
        # if t != math.inf and len(self.x) > 0:
        #     idx = math.floor(t*self.total_num_steps/self.total_time)
        #     if 0 < idx < len(self.x)-2:
        #         x = self.x[idx]
        #         x_dot = self.x_dot[idx]
        #         x_ddot = self.x_ddot[idx]
        #         yaw = self.yaw[idx]


        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output

    # def compute_waypoints(self, points):
    #     # compute position
    #     tck, u = interpolate.splprep([points[:, 0], points[:, 1], points[:, 2]], s=2)
    #     u_fine = np.linspace(0, 1, self.total_num_steps)
    #     x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
    #     self.x = np.array([x_fine, y_fine, z_fine]).T
    #
    #     # compute velocity
    #     for i in range(1, len(self.x)):
    #         x_dot_i = (self.x[i] - self.x[i - 1]) / 0.002
    #         self.x_dot.append(x_dot_i)
    #
    #     # compute acceleration
    #     for i in range(1, len(self.x_dot)):
    #         x_ddot_i = (self.x_dot[i] - self.x_dot[i - 1]) / 0.002
    #         self.x_ddot.append(x_ddot_i)
    #
    #     self.x_dot = np.asarray(self.x_dot)
    #     self.x_ddot = np.asarray(self.x_ddot)
    #
    #     fig = plt.figure()
    #     ax = plt.axes(projection='3d')
    #     ax.set_xlim([-1, 1])
    #     ax.set_ylim([-1, 1])
    #     ax.set_zlim([-1, 1])
    #     plt.plot(x_fine, y_fine, z_fine)
    #     plt.plot(points[:,0],points[:,1],points[:,2],'g*')
    #     plt.plot(self.x_dot[:,0], self.x_dot[:,1], self.x_dot[:,2],'r')
    #     plt.plot(self.x_ddot[:,0], self.x_ddot[:,1], self.x_ddot[:,2],'k')


