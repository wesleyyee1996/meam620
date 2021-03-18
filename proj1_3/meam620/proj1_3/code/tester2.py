
import numpy as np
import matplotlib.pyplot as plt
from flightsim.axes3ds import Axes3Ds

# waypoints = np.array([[0,0,0],
#                       [1,1,2],
#                       [2,0,2],
#                       [3,2,3],
#                       [4,4,4],
#                       [5,5,5]])
waypoints = np.array([[1. ,   1.  ,  1.   ],
 [1.125, 1.375, 1.125],
 [0.625, 1.875, 0.625],
 [0.625, 3.125, 0.625],
 [2.125, 4.625, 0.625],
 [3.375, 5.875, 1.875],
 [3.375, 7.125, 1.875],
 [2.875, 7.625, 2.375],
 [2.625, 7.625, 2.125],
 [2.5,   7.5  , 2.   ]])


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

    def get_traj(self, t):
        for k in range(0,3):
            self.construct_end_point_boundary_cond(k)
            self.construct_intermediary_cond(k)
            self.construct_regular_cont_cond()
            self.construct_higher_cont_cond()
            self.solve(k)

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

traj = traj_gen(waypoints)

new_path = []
l = np.arange(0.0, 5, 0.002)
for t in l:
    new_path.append(traj.get_traj(t))
new_path = np.array(new_path)
print(new_path)
fig = plt.figure('all')
ax = Axes3Ds(fig)
# ax = plt.axes()
plt.plot(new_path[:,0], new_path[:,1], new_path[:,2], 'r.')
plt.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2], 'b*')
plt.xlabel('x')
plt.ylabel('y')
plt.show()

