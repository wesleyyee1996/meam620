import numpy as np
import math
import matplotlib.pyplot as plt
from flightsim.axes3ds import Axes3Ds
from .graph_search import graph_search

class traj_gen:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.num_segments = np.shape(waypoints)[0]-1
        self.speed = 2
        self.A = np.zeros(((self.num_segments+1)*6, (self.num_segments+1)*6))
        self.b = np.zeros((self.num_segments+1)*6)
        self.c = np.zeros(((self.num_segments+1)*6, 3))
        self.prev_total = 0
        self.curr_segment = 0
        self.curr_segment_final_time = self.get_duration(0)
        self.curr_segment_start_time = 0
        self.total_time = 0
        self.calc_total_time()
        for k in range(0,3):
            self.construct_end_point_boundary_cond(k)
            self.construct_intermediary_cond(k)
            self.construct_regular_cont_cond()
            self.construct_higher_cont_cond()
            self.solve(k)
        print("init traj")

    def get_traj(self, t):
        if t > self.total_time:
            return self.waypoints[self.num_segments]
        self.get_curr_segment(t)
        new_t = t - self.curr_segment_start_time
        return self.get_xyz_coords(self.curr_segment, new_t)

    def get_curr_segment(self, t):
        if t > self.curr_segment_final_time:
            if self.curr_segment < (self.num_segments-1):
                self.curr_segment += 1
                self.curr_segment_final_time += self.get_duration(self.curr_segment)
                self.curr_segment_start_time += self.get_duration(self.curr_segment-1)

    def calc_total_time(self):
        for i in range(self.num_segments-1):
            self.total_time += self.get_duration(i)


    def get_xyz_coords(self, curr_seg, t):
        x = self.c[curr_seg*6, 0]*t**5 + self.c[curr_seg*6+1, 0]*t**4 + self.c[curr_seg*6+2, 0]*t**3 + \
                self.c[curr_seg*6+3, 0]*t**2 + self.c[curr_seg*6+4, 0]*t + self.c[curr_seg*6+5, 0]
        y = self.c[curr_seg * 6, 1] * t ** 5 + self.c[curr_seg * 6 + 1, 1] * t ** 4 + self.c[
            curr_seg * 6 + 2, 1] * t ** 3 + \
            self.c[curr_seg * 6 + 3, 1] * t ** 2 + self.c[curr_seg * 6 + 4, 1] * t + self.c[curr_seg * 6 + 5, 1]
        z = self.c[curr_seg * 6, 2] * t ** 5 + self.c[curr_seg * 6 + 1, 2] * t ** 4 + self.c[
            curr_seg * 6 + 2, 2] * t ** 3 + \
            self.c[curr_seg * 6 + 3, 2] * t ** 2 + self.c[curr_seg * 6 + 4, 2] * t + self.c[curr_seg * 6 + 5, 2]
        return [x, y, z]

    def solve(self, k):
        self.c[:,k] = np.linalg.solve(self.A, self.b.T)

    def construct_higher_cont_cond(self):
        for i in range(self.num_segments):
            t = self.get_duration(i)
            self.A[self.prev_total + 1, 6*i:6*i+3] = [60*t**2, 24*t, 6]
            self.A[self.prev_total + 2, 6*i:6*i+2] = [120*t, 24]
            self.A[self.prev_total + 1, 6*(i+1)+2] = -6
            self.A[self.prev_total + 2, 6*(i+1)+1] = -24

            self.b[self.prev_total + 1] = 0
            self.b[self.prev_total + 2] = 0

            self.prev_total = self.prev_total + 2

    def construct_regular_cont_cond(self):
        for i in range(self.num_segments):
            t = self.get_duration(i)
            self.A[self.prev_total + 1, 6*i:6*i+5] = [5*t**4, 4*t**3, 3*t**2, 2*t, 1]
            self.A[self.prev_total + 2, 6*i:6*i+4] = [20*t**3, 12*t**2, 6*t, 2]
            self.A[self.prev_total + 1, 6*(i+1)+4] = -1
            self.A[self.prev_total + 2, 6*(i+1)+3] = -2

            self.b[self.prev_total + 1] = 0
            self.b[self.prev_total + 2] = 0

            self.prev_total = self.prev_total + 2

    def construct_intermediary_cond(self, k):
        for i in range(self.num_segments):
            t = self.get_duration(i)
            self.A[6+i*2,i*6:i*6+6] = [t**5, t**4, t**3, t**2, t, 1]
            self.A[7+i*2,6*(i+1)+5] = 1

            self.b[6+i*2] = self.waypoints[i+1, k]
            self.b[7+i*2] = self.waypoints[i+1, k]

            self.prev_total = self.prev_total + 2

    def construct_end_point_boundary_cond(self, k):
        """
        Construct the end point boundary conditions
        Only applies to 1st segment and last segment
        :param k : current axis (x=0, y=1, z=2)
        :return: none
        """
        # starting point BCs for pos, vel, accel
        self.A[0,5] = 1
        self.A[1,4] = 1
        self.A[2,3] = 2

        # end point BCs for pos, vel, accel
        t = self.get_duration(self.num_segments-1) # total duration for last segment
        self.A[3,-6:] = [t**5, t**4, t**3, t**2, t, 1]
        self.A[4,-6:-1] = [5*t**4, 4*t**3, 3*t**2, 2*t, 1]
        self.A[5,-6:-2] = [20*t**3, 12*t**2, 6*t, 2]

        self.b[0:6] = [self.waypoints[0,k], 0, 0, self.waypoints[self.num_segments, k], 0, 0]

        self.prev_total = 5


    def get_duration(self, curr_segment_idx):
        """
        calculates total time to traverse current segment assuming constant speed
        :param curr_segment_idx:
        :return: time of current segment
        """
        dist = np.linalg.norm(self.waypoints[curr_segment_idx+1] - self.waypoints[curr_segment_idx])
        return dist/self.speed

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.25, 0.25, .25])
        self.margin = 0.45

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        print("Computed path")

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        # self.points = self.simplify_path(self.path) # shape=(n_pts,3)
        self.points = np.array([[0.7, -4.3, 0.7],
                                [0.625, -3.625, 0.625],
                                [0.625, -2.625, 0.625],
                                [1.625, -1.625, 1.625],
                                [1.875, -1.375, 1.875],
                                [1.875, -0.375, 1.875],
                                [1.875, 0.125, 1.875],
                                [2.875, 1.125, 2.875],
                                [3.625, 1.875, 3.625],
                                [3.625, 2.875, 3.625],
                                [3.625, 3.875, 3.625],
                                [3.625, 4.875, 3.625],
                                [3.625, 5.875, 3.625],
                                [3.625, 6.875, 3.625],
                                [3.625, 7.875, 3.625],
                                [3.625, 8.875, 3.625],
                                [3.875, 9.125, 3.875],
                                [3.875, 9.375, 3.875],
                                [4.375, 9.875, 3.875],
                                [4.375, 10.125, 3.875],
                                [4.625, 10.375, 3.875],
                                [4.625, 10.625, 3.875],
                                [4.875, 10.875, 3.875],
                                [4.875, 11.125, 3.875],
                                [5.125, 11.375, 3.875],
                                [5.125, 11.625, 3.875],
                                [5.375, 11.875, 3.875],
                                [5.375, 12.375, 3.875],
                                [5.625, 12.625, 3.875],
                                [5.625, 12.875, 3.875],
                                [5.875, 13.125, 3.875],
                                [5.875, 13.375, 3.875],
                                [6.125, 13.625, 3.875],
                                [6.125, 13.875, 3.875],
                                [6.375, 14.125, 3.875],
                                [6.375, 14.625, 3.875],
                                [6.625, 14.875, 4.125],
                                [6.625, 15.125, 4.125],
                                [6.875, 15.375, 4.125],
                                [6.875, 16.125, 4.125],
                                [7.375, 16.625, 3.625],
                                [7.625, 16.875, 3.625],
                                [7.625, 17.125, 3.625],
                                [7.875, 17.375, 3.375],
                                [7.875, 17.625, 3.375],
                                [8.125, 17.875, 3.125],
                                [8.125, 18.125, 3.125],
                                [8., 18., 3.]])
        # print(self.points)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        # fig = plt.figure('all')
        # ax = Axes3Ds(fig)
        # # ax = plt.axes()
        # plt.plot(self.path[:,0], self.path[:,1], self.path[:,2], 'r.')
        # plt.plot(self.points[:,0], self.points[:,1], self.points[:,2], 'b*')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.show()

        self.traj = traj_gen(self.points)
        print("Computed trajectory")


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

        # STUDENT CODE HERE
        if not math.isinf(t):
            x = self.traj.get_traj(t)
            x_n = self.traj.get_traj(t+0.002)
            x_dot = [(x_n[0]-x[0])/0.002, (x_n[1]-x[1])/0.002, (x_n[2]-x[2])/0.002]
            if (np.linalg.norm(x_dot) > 4):
                print(x_dot)
            #     x_dot = np.array(x_dot)/np.linalg.norm(x_dot)*10
        if t > self.traj.total_time:
            x = self.points[-1,:]
            x_dot = np.zeros((3,))
            x_ddot = np.zeros((3,))
            x_dddot = np.zeros((3,))
            x_ddddot = np.zeros((3,))
            yaw = 0
            yaw_dot = 0


        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}

        return flat_output

    def simplify_path(self, waypoints):

        length_waypoints = np.shape(waypoints)[0]
        good_waypoints = np.ones(length_waypoints)
        good_waypoints[1:3] = 0


        start = 0
        mid = 1
        end = 2
        num_skips = 0
        while (end < length_waypoints):
            unit_vec1 = (waypoints[mid, :] - waypoints[start, :]) / np.linalg.norm(
                waypoints[mid, :] - waypoints[start, :])
            unit_vec2 = (waypoints[end, :] - waypoints[mid, :]) / np.linalg.norm(waypoints[end, :] - waypoints[mid, :])
            if (np.linalg.norm(unit_vec2 - unit_vec1) < 0.001 and num_skips < 3):
                good_waypoints[mid] = 0
                mid += 1
                end += 1
                num_skips += 1
            else:
                start += 1
                while (good_waypoints[start] == 0):
                    start += 1
                mid = start + 1
                end = mid + 1
                num_skips = 0
        good_waypoints[-5:-2] = 1
        output = []
        for i in range(length_waypoints):
            if good_waypoints[i] == 1:
                output.append(waypoints[i, :])
        return np.array(output)
