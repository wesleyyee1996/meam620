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
        self.kd1 = 0
        self.kp1 = 0
        self.kd2 = 0
        self.kp2 = 0

        self.kd3 = 700000
        self.kp3 = 5000000
        self.kp_roll = 250
        self.kp_pitch = 50000
        self.kd_pitch = 0
        self.kp_yaw = 0
        self.kd_yaw = 0

    # def euler_from_quaternion(self, quat):
    #     x = quat[0]
    #     y = quat[1]
    #     z = quat[2]
    #     w = quat[3]
    #
    #     t0 = 2.0 * (w*x+y*z)
    #     t1 = 1.0 - 2.0 * (x*x + y*y)
    #     roll_x = math.atan2(t0,t1)
    #
    #     t2 = 2.0 * (w*y - z*x)
    #     t2 = 1.0 if t2 > 1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch_y = math.asin(t2)
    #
    #     t3 = 2.0 * (w*z + x*y)
    #     t4 = 1.0 - 2.0*(y*y + z*z)
    #     yaw_z = math.atan2(t3, t4)
    #     return roll_x, pitch_y, yaw_z

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

        # compute commanded acceleration
        accel_comm = [flat_output.get('x_ddot')[0] - self.kd1*(state.get('v')[0]-flat_output.get('x_dot')[0]) - self.kp1*(state.get('x')[0]-flat_output.get('x')[0]),
                      flat_output.get('x_ddot')[1] - self.kd2*(state.get('v')[1]-flat_output.get('x_dot')[1]) - self.kp2*(state.get('x')[1]-flat_output.get('x')[1]),
                      flat_output.get('x_ddot')[2] - self.kd3*(state.get('v')[2]-flat_output.get('x_dot')[2]) - self.kp3*(state.get('x')[2]-flat_output.get('x')[2])]

        # compute desired thrust control
        u1 = self.mass*(accel_comm[2]+self.g)

        # compute desired roll and pitch
        b = (np.array([accel_comm[0]//self.g, accel_comm[1]//self.g]).T)
        A = np.array([[np.cos(flat_output.get('yaw')), np.sin(flat_output.get('yaw'))],
                      [np.sin(flat_output.get('yaw')), -np.cos(flat_output.get('yaw'))]])
        x = np.dot(np.linalg.inv(A),b)
        pitch_des = x[0]/180*math.pi
        roll_des = x[1]/180*math.pi

        [roll, pitch, yaw] = Rotation.from_quat(state.get('q')).as_euler('xyz', degrees=False)

        #roll, pitch, yaw = self.euler_from_quaternion(state.get('q'))

        # compute attitude control
        u2 = np.array([self.Ixx*-self.kp_roll*(roll-roll_des),
                      self.Iyy*-self.kp_pitch*(pitch-pitch_des)-(self.Iyy*-self.kd_pitch*state.get('w')[1]),
                      self.Izz*-self.kp_yaw*(yaw-flat_output.get('yaw'))])
        #print(pitch-pitch_des)

        # combine u1 and u2 into u_tot
        u_tot = np.array([u1, u2[0], u2[1], u2[2]]).T

        # solve for cmd_motor_speeds
        A = np.array([[1,1,1,1],
                      [0,self.arm_length, 0, -self.arm_length],
                      [-self.arm_length, 0, self.arm_length, 0],
                      [self.y, -self.y, self.y, -self.y]])

        cmd_motor_speeds = np.dot(np.linalg.inv(A), u_tot)

        for i in cmd_motor_speeds:
            cmd_thrust += self.k_thrust*(i**2)
            cmd_moment += self.k_drag*(i**2)

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q,
                         'roll':roll,
                         'pitch':pitch,
                         'roll_des':roll_des,
                         'pitch_des':pitch_des}
        return control_input
