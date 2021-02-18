import numpy as np
import math
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2
        self.y               = self.k_drag/self.k_thrust

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE
        self.I = np.array([[self.Ixx, 0, 0],
                          [0, self.Iyy, 0],
                          [0, 0, self.Izz]])

        self.kp = [20, 20, 20]
        self.kd = [self.kp[1]/1.25, self.kp[1]/1.25, self.kp[1]/1.25]
        self.kr = [8100, 8100, 1000]
        self.kw = [510, 510, 100]
        # self.kr = [8100, 8100, 1000]
        # self.kw = [510, 510, 100]



    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE

        # calculate desired acceleration
        des_acc = np.array([flat_output.get('x_ddot')[0]-self.kd[0]*(state.get('v')[0]-flat_output.get('x_dot')[0])-self.kp[0]*(state.get('x')[0]-flat_output.get('x')[0]),
            flat_output.get('x_ddot')[1]-self.kd[1]*(state.get('v')[1]-flat_output.get('x_dot')[1]) - self.kp[1]*(state.get('x')[1]-flat_output.get('x')[1]),
            flat_output.get('x_ddot')[2] - self.kd[2] * (state.get('v')[2]-flat_output.get('x_dot')[2]) - self.kp[2] * (state.get('x')[2]-flat_output.get('x')[2])])

        # calculate F_des
        F_des = des_acc*self.mass + np.array([0,0,self.mass*self.g])

        # calculate b3
        R = Rotation.from_quat(state.get('q')).as_matrix()
        b3 = R @ np.array([0,0,1])

        # calculate u1
        u1 = b3.T @ F_des

        # calculate R_des
        b3_des = F_des/np.linalg.norm(F_des)
        a_psi = np.array([np.cos(flat_output.get('yaw')),
                          np.sin(flat_output.get('yaw')),
                          0])
        b3_desXa_psi = np.cross(b3_des, a_psi)
        b2_des = (b3_desXa_psi)/np.linalg.norm(b3_desXa_psi)
        b2_desXb3_des = np.cross(b2_des, b3_des)
        R_des = np.array([[b2_desXb3_des[0],b2_des[0],b3_des[0]],
                          [b2_desXb3_des[1],b2_des[1],b3_des[1]],
                          [b2_desXb3_des[2],b2_des[2],b3_des[2]]])

        # calcualte e_R
        e_R = 0.5 * ((R_des.T @ R) - (R.T @ R_des))
        e_R = np.array([-e_R[1][2],
                        e_R[0][2],
                        -e_R[0][1]])

        # calculate u2
        w = np.array([state.get('w')[0], state.get('w')[1], state.get('w')[2]])
        u2 = self.I @ ((-np.diag(self.kr) @ e_R) - np.diag(self.kw) @ w)

        utot = np.array([u1, u2[0], u2[1], u2[2]])

        A = np.array([[1,1,1,1],
                      [0, self.arm_length, 0, -self.arm_length],
                      [-self.arm_length, 0, self.arm_length, 0],
                      [self.y, -self.y, self.y, -self.y]])

        F = np.linalg.inv(A) @ utot

        for i in range(len(F)):
            if F[i] < 0:
                F[i] = 0
                cmd_motor_speeds[i] = self.rotor_speed_max
            cmd_motor_speeds[i] = math.sqrt(F[i]/self.k_thrust)
            if cmd_motor_speeds[i] >= self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max


        #cmd_motor_speeds = np.where(cmd_motor_speeds > self.rotor_speed_max, rotor_speed_max, cmd_motor_speeds)

        cmd_thrust = u1
        cmd_moment = u2

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input