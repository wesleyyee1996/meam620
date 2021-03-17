
import numpy as np

waypoints = np.array([[0,0,0],
                      [1,1,2],
                      [2,1,2]])

num_segments = np.shape(waypoints)[0]-1

class traj_gen:
    def __init__(self, num_segments, waypoints):
        self.waypoints = waypoints
        self.num_segments = np.shape(waypoints)[0]-1
        self.speed = 2
        self.A = A = np.zeros([(num_segments+1)*6, (num_segments+1)*6])
        self.prev_total = 0

    def get_traj(self):
        self.construct_end_point_boundary_cond()
        self.construct_intermediary_cond()
        self.construct_regular_cont_cond()
        self.construct_higher_cont_cond()
        print(self.A)

    def construct_higher_cont_cond(self):
        for i in range(num_segments):
            t = self.get_duration(i)
            self.A[self.prev_total + i*2 -1, 6*i:6*i+3] = [60*t**2, 24*t, 6]
            self.A[self.prev_total + i*2, 6*i:6*i+2] = [120*t, 24]
            self.A[self.prev_total + i*2-1, 6*(i+1)+2] = -6
            self.A[self.prev_total + i*2, 6*(i+1)+1] = -24
            self.prev_total = self.prev_total + 2

    def construct_regular_cont_cond(self):
        for i in range(num_segments):
            t = self.get_duration(i)
            self.A[self.prev_total + i*2-1, 6*i:6*i+5] = [5*t**4, 4*t**3, 3*t**2, 2*t, 1]
            self.A[self.prev_total + i*2, 6*i:6*i+4] = [20*t**3, 12*t**2, 6*t, 2]
            self.A[self.prev_total + i*2-1, 6*(i+1)+4] = -1
            self.A[self.prev_total + i*2, 6*(i+1)+3] = -2
            self.prev_total = self.prev_total + 2

    def construct_intermediary_cond(self):
        for i in range(num_segments):
            t = self.get_duration(i)
            self.A[6+i*2,i*6:i*6+6] = [t**5, t**4, t**3, t**2, t, 1]
            self.A[7+i*2,6*(i+1)+5] = 1
            self.prev_total = self.prev_total + 2

    def construct_end_point_boundary_cond(self):
        """
        Construct the end point boundary conditions
        Only applies to 1st segment and last segment
        """
        # starting point BCs for pos, vel, accel
        self.A[0,5] = 1
        self.A[1,4] = 1
        self.A[2,3] = 2

        # end point BCs for pos, vel, accel
        t = self.get_duration(self.num_segments) # total duration for last segment
        self.A[3,-6:] = [t**5, t**4, t**3, t**2, t, 1]
        self.A[4,-6:-1] = [5*t**4, 4*t**3, 3*t**2, 2*t, 1]
        self.A[5,-6:-2] = [20*t**3, 12*t**2, 6*t, 2]

        self.prev_total = 5


    def get_duration(self, curr_segment_idx):
        """
        calculates total time to traverse current segment assuming constant speed
        :param curr_segment_idx:
        :return: time of current segment
        """
        dist = np.linalg.norm(self.waypoints[curr_segment_idx] - self.waypoints[curr_segment_idx-1])
        return dist/self.speed

traj = traj_gen(num_segments, waypoints)
traj.get_traj()

