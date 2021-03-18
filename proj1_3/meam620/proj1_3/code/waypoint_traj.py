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
        self.v = 1.5
        self.unit_vecs = [0]*len(points)
        self.unit_vel_vecs = [0]*len(points)
        self.duration = [0]*len(points)
        self.points = points

        for i in range(1,len(points)):
            length = np.linalg.norm(points[i]-points[i-1])
            unit = (points[i]-points[i-1])/length
            self.unit_vecs[i] = unit
            self.unit_vel_vecs[i] = self.unit_vecs[i]*self.v
            # if len(self.duration) == 0:
            #     self.duration.append(length / self.v)
            self.duration[i] = length / self.v + self.duration[i-1]
        #
        # print(points)
        # print(self.duration)
        # print(self.unit_vecs)

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

        if len(self.points)==0 or t == 0:
            x = self.points[0]
        else:
            if t > self.duration[-1]:
                x = self.points[-1]
            else:
                for i in range(1,len(self.points)):
                    if t <= self.duration[i]:
                        x = self.unit_vecs[i]*(t-self.duration[i])+self.points[i]
                        x_dot = self.unit_vel_vecs[i]
                        break


        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output



