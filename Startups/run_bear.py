#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for communication with BEAR actuators on BRUCE
'''

import time
import collections
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
from numba import jit
from Play.config import *
from termcolor import colored
from Settings.BRUCE_macros import *
from Library.ACTUATORS.BEAR_controller import BEARController


class BEAR_ESTOP_Exception(Exception):
    """
    Raised when E-STOP signal is detected
    """
    pass


class BEAR_ERROR_Exception(Exception):
    """
    Raised when BEAR in error
    """
    pass


@jit
def motor_to_joint_position(p1, p2, p3, p4, p5,
                            p6, p7, p8, p9, p10):
    """
    Convert motor position to DH joint position
    """
    # right
    q1  = -p1
    q2  = -0.5 * (p2 + p3) + PI_2
    q3  =  0.5 * (p2 - p3)
    q4  = -p4
    q5  =  p4 + p5

    # left
    q6  = -p6
    q7  = -0.5 * (p7 + p8) + PI_2
    q8  =  0.5 * (p7 - p8)
    q9  =  p9
    q10 = -p9 - p10
    return np.array([q1, q2, q3, q4, q5,
                     q6, q7, q8, q9, q10])


motor_to_joint_position(0.1, 0.1, 0.1, 0.1, 0.1,
                        0.1, 0.1, 0.1, 0.1, 0.1)


@jit
def motor_to_joint_velocity(v1, v2, v3, v4, v5,
                            v6, v7, v8, v9, v10):
    """
    Convert motor velocity to DH joint velocity
    """
    # right
    dq1  = -v1
    dq2  = -0.5 * (v2 + v3)
    dq3  =  0.5 * (v2 - v3)
    dq4  = -v4
    dq5  =  v4 + v5

    # left
    dq6  = -v6
    dq7  = -0.5 * (v7 + v8)
    dq8  =  0.5 * (v7 - v8)
    dq9  =  v9
    dq10 = -v9 - v10
    return np.array([dq1, dq2, dq3, dq4, dq5,
                     dq6, dq7, dq8, dq9, dq10])


motor_to_joint_velocity(0.1, 0.1, 0.1, 0.1, 0.1,
                        0.1, 0.1, 0.1, 0.1, 0.1)

@jit
def joint_to_motor_position(q1, q2, q3, q4, q5,
                            q6, q7, q8, q9, q10):
    """
    Convert DH joint position to motor position
    """
    # right
    p1  = -q1
    p2  = -q2 + q3 + PI_2
    p3  = -q2 - q3 + PI_2
    p4  = -q4
    p5  =  q4 + q5

    # left
    p6  = -q6
    p7  = -q7 + q8 + PI_2
    p8  = -q7 - q8 + PI_2
    p9  =  q9
    p10 = -q9 - q10
    return [[p1], [p2], [p3], [p4], [p5],
            [p6], [p7], [p8], [p9], [p10]]


joint_to_motor_position(0.1, 0.1, 0.1, 0.1, 0.1,
                        0.1, 0.1, 0.1, 0.1, 0.1)


@jit
def joint_to_motor_velocity(dq1, dq2, dq3, dq4, dq5,
                            dq6, dq7, dq8, dq9, dq10):
    """
    Convert DH joint velocity to motor velocity
    """
    # right
    v1  = -dq1
    v2  = -dq2 + dq3
    v3  = -dq2 - dq3
    v4  = -dq4
    v5  =  dq4 + dq5

    # left
    v6  = -dq6
    v7  = -dq7 + dq8
    v8  = -dq7 - dq8
    v9  =  dq9
    v10 = -dq9 - dq10
    return [[v1], [v2], [v3], [v4], [v5],
            [v6], [v7], [v8], [v9], [v10]]


joint_to_motor_velocity(0.1, 0.1, 0.1, 0.1, 0.1,
                        0.1, 0.1, 0.1, 0.1, 0.1)


@jit
def joint_to_motor_iq(t1, t2, t3, t4, t5,
                      t6, t7, t8, t9, t10):
    """
    Convert DH joint torque to motor iq
    """
    # right
    i1  = -t1              * TORQUE2IQ
    i2  = -0.5 * (t2 - t3) * TORQUE2IQ
    i3  = -0.5 * (t2 + t3) * TORQUE2IQ
    i4  = (t5 - t4)        * TORQUE2IQ
    i5  =  t5              * TORQUE2IQ

    # left
    i6  = -t6              * TORQUE2IQ
    i7  = -0.5 * (t7 - t8) * TORQUE2IQ
    i8  = -0.5 * (t7 + t8) * TORQUE2IQ
    i9  = (t9 - t10)       * TORQUE2IQ
    i10 = -t10             * TORQUE2IQ
    return [[i1], [i2], [i3], [i4], [i5],
            [i6], [i7], [i8], [i9], [i10]]


joint_to_motor_iq(0.1, 0.1, 0.1, 0.1, 0.1,
                  0.1, 0.1, 0.1, 0.1, 0.1)


@jit
def motor_to_joint_torque(i1, i2, i3, i4, i5,
                          i6, i7, i8, i9, i10):
    """
    Convert motor iq to DH joint torque
    """
    # right
    t1 = -i1        * IQ2TORQUE
    t2 = -(i2 + i3) * IQ2TORQUE
    t3 = (i2 - i3)  * IQ2TORQUE
    t4 = (i5 - i4)  * IQ2TORQUE
    t5 = i5         * IQ2TORQUE

    # left
    t6  = -i6        * IQ2TORQUE
    t7  = -(i7 + i8) * IQ2TORQUE
    t8  = (i7 - i8)  * IQ2TORQUE
    t9  = (i9 - i10) * IQ2TORQUE
    t10 = -i10       * IQ2TORQUE

    return np.array([t1, t2, t3, t4, t5,
                     t6, t7, t8, t9, t10])


motor_to_joint_torque(0.1, 0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1, 0.1)


@jit
def joint_to_motor_value(t1, t2, t3, t4, t5,
                         t6, t7, t8, t9, t10,
                         q1, q2, q3, q4, q5,
                         q6, q7, q8, q9, q10,
                         dq1, dq2, dq3, dq4, dq5,
                         dq6, dq7, dq8, dq9, dq10):
    """
    Convert DH joint torque to motor iq
    """
    # right
    i1  = -t1              * TORQUE2IQ
    i2  = -0.5 * (t2 - t3) * TORQUE2IQ
    i3  = -0.5 * (t2 + t3) * TORQUE2IQ
    i4  = (t5 - t4)        * TORQUE2IQ
    i5  =  t5              * TORQUE2IQ

    # left
    i6  = -t6              * TORQUE2IQ
    i7  = -0.5 * (t7 - t8) * TORQUE2IQ
    i8  = -0.5 * (t7 + t8) * TORQUE2IQ
    i9  = (t9 - t10)       * TORQUE2IQ
    i10 = -t10             * TORQUE2IQ

    """
    Convert DH joint position to motor position
    """
    # right
    p1  = -q1
    p2  = -q2 + q3 + PI_2
    p3  = -q2 - q3 + PI_2
    p4  = -q4
    p5  =  q4 + q5

    # left
    p6  = -q6
    p7  = -q7 + q8 + PI_2
    p8  = -q7 - q8 + PI_2
    p9  =  q9
    p10 = -q9 - q10

    """
    Convert DH joint velocity to motor velocity
    """
    # right
    v1  = -dq1
    v2  = -dq2 + dq3
    v3  = -dq2 - dq3
    v4  = -dq4
    v5  =  dq4 + dq5

    # left
    v6  = -dq6
    v7  = -dq7 + dq8
    v8  = -dq7 - dq8
    v9  =  dq9
    v10 = -dq9 - dq10
    return [[i1, p1, v1], [i2, p2, v2], [i3, p3, v3], [i4, p4, v4], [i5, p5, v5],
            [i6, p6, v6], [i7, p7, v7], [i8, p8, v8], [i9, p9, v9], [i10, p10, v10]]


joint_to_motor_value(0.1, 0.1, 0.1, 0.1, 0.1,
                     0.1, 0.1, 0.1, 0.1, 0.1,
                     0.1, 0.1, 0.1, 0.1, 0.1,
                     0.1, 0.1, 0.1, 0.1, 0.1,
                     0.1, 0.1, 0.1, 0.1, 0.1,
                     0.1, 0.1, 0.1, 0.1, 0.1)


class bear_controller:
    def __init__(self, port='/dev/ttyUSB0', baudrate=8000000):
        # motor port and baudrate
        self.BEAR_port = port
        self.BEAR_baudrate = baudrate

        # motor manager
        self.BEAR_controller = BEARController(self.BEAR_port, self.BEAR_baudrate)

        # motor mode
        self.BEAR_mode  = 3
        self.BEAR_modes = {'torque': 0, 'velocity': 1, 'position': 2, 'force': 3}

        # torque mode
        self.torque_mode  = 0
        self.torque_modes = {'disable': 0, 'enable': 1, 'error': 2, 'damping': 3}

        # error status
        self.error_status = {'Communication': 0, 'Overheat': 1, 'Absolute Position': 2, 'E-STOP': 3, 'Joint Limit': 4, 'Hardware Fault': 5, 'Initialization': 6}

        # bear info
        self.bear = collections.defaultdict(lambda: collections.defaultdict(int))

        # joint info
        self.joint = collections.defaultdict(lambda: collections.defaultdict(int))

        self.Q   = np.zeros(10)  # raw joint position array
        self.dQF = np.zeros(10)  # filtered joint velocity array
        self.IQ  = np.zeros(10)  # filtered raw motor iq array
        self.Tau = np.zeros(10)  # filtered joint torque array
        self.dQM = np.zeros(10)  # filtered motor velocity array
        self.trial_max = 1       # BEAR communication maximum trial number for each loop

    def close(self):
        """
        Close PyBEAR serial port
        """
        self.BEAR_controller.pbm.close()

    def start_drivers(self):
        """
        Ping all BEARs and then config their PIDs, limits, etc.
        """
        error = False
        for bear_id in BEAR_LIST:
            trail = 0
            check = False
            print("Pinging BEAR %02d..." % bear_id)
            while not check:
                ping_rtn = self.BEAR_controller.pbm.ping(bear_id)
                if bool(ping_rtn[0]):
                    check = True
                    print('Pinging BEAR %02d Succeed.' % bear_id)
                    if ping_rtn[0][1] != 128:  # ERROR!!!
                        print(colored(('BEAR %02d ERROR CODE %d: ' + self.decode_error(ping_rtn[0][1])) % (bear_id, ping_rtn[0][1]), 'red'))
                        error = True
                else:
                    trail += 1
                    if trail > 6:
                        print("ERROR: BEAR %02d offline!!!" % bear_id)
                        error = True
                        break
                    time.sleep(0.5)
            print("")

        if error:
            raise BEAR_ERROR_Exception

        self.BEAR_controller.start_drivers()

    def decode_error(self, error_code):
        """
        Decode BEAR error code
        """
        msg = ''
        if error_code >> 7 != 1:
            print(colored('Invalid Error Code!!!', 'red'))
            return msg
        else:
            error_num = 0
            for idx in range(7):
                if error_code >> idx & 1:
                    error_num += 1
                    if error_num > 1:
                        msg += ' & '
                    msg += list(self.error_status.keys())[list(self.error_status.values()).index(idx)]
            if error_num:
                return msg
            else:
                return 'No Error!'

    def set_actuator_mode(self, mode_bear):
        """
        Set the actuator into a desired mode
        """
        self.BEAR_controller.set_mode(mode_bear)
        self.BEAR_mode = self.BEAR_modes[mode_bear]

    def torque_enable_all(self, val):
        """
        Torque enable
        """
        self.torque_mode = self.torque_modes['disable'] if val == 0 else self.torque_modes['enable']
        self.BEAR_controller.torque_enable_all(val)

    def set_damping_mode(self):
        """
        Set the joints into damping mode
        """
        self.BEAR_controller.set_damping_mode()

    def get_torque_mode(self):
        """
        Get BEAR torque mode
        """
        try:
            self.torque_mode = self.BEAR_controller.get_torque_mode()
        except:
            print("Communication Fails!!!")

    def is_damping_mode(self):
        """
        Check if in damping mode
        """
        return True if self.torque_mode == 3 else False

    def update_present_temperature_and_voltage(self):
        """
        Only get knee temperature and voltage
        """
        try:
            info = self.BEAR_controller.pbm.bulk_read([BEAR_KNEE_R, BEAR_KNEE_L], ['input_voltage', 'winding_temperature'])
            vol  = min(info[0][0][0], info[1][0][0])
            temp = max(info[0][0][1], info[1][0][1])

            if vol < 14.5:
                print(colored('BEAR Low Voltage!', 'yellow'))

            if temp > 75.0:
                print(colored('BEAR High Temperature!', 'yellow'))

            MM.LEG_STATE.set({'temperature': np.array([temp]),
                              'voltage':     np.array([vol])}, opt='only')
        except:
            print("Communication Fails!!!")

    def get_error(self):
        """
        Ping to get error codes
        """
        error = False
        error_codes = self.BEAR_controller.pbm.ping(
            BEAR_HIP1_R, BEAR_HIP2_R, BEAR_HIP3_R, BEAR_KNEE_R, BEAR_ANKLE_R,
            BEAR_HIP1_L, BEAR_HIP2_L, BEAR_HIP3_L, BEAR_KNEE_L, BEAR_ANKLE_L)
        for idx, bear_id in enumerate(BEAR_LIST):
            j = self.bear[bear_id]
            if error_codes[idx]:
                # Return for this BEAR is not None
                j['error'] = error_codes[idx][1]
                if j['error'] != 128:
                    print("ERROR: BEAR %02d in error code: %d!!!" % (bear_id, j['error']))
                    error = True
            else:
                # This BEAR is offline
                j['error'] = None
                print("ERROR: BEAR %02d offline!!!" % bear_id)
                error = True
        return error

    def change_mode(self):
        """
        Change bear mode
        """
        no_error = True
        bear_mode = None
        if self.BEAR_mode == 2:
            bear_mode = 'position'
        elif self.BEAR_mode == 1:
            bear_mode = 'velocity'
        elif self.BEAR_mode == 0:
            bear_mode = 'torque'
        elif self.BEAR_mode == 3:
            bear_mode = 'force'
        else:
            no_error = False

        if no_error:
            print('Changing BEAR mode to %s!' % bear_mode)
            self.BEAR_controller.set_mode(bear_mode)
        else:
            print('Invalid BEAR operation mode!!!')

    def update_present_status(self):
        """
        Get present position, velocity, and iq
        """
        leg_info = None
        trial    = 0
        try:
            while leg_info is None and trial < self.trial_max:
                leg_info = self.BEAR_controller.pbm.bulk_read(BEAR_LIST, ['present_position', 'present_velocity', 'present_iq'])
                trial += 1

            for idx in range(10):
                self.dQM[idx] = MF.exp_filter(self.dQM[idx], leg_info[idx][0][1], 0.90)

            self.Q = motor_to_joint_position(leg_info[0][0][0], leg_info[1][0][0], leg_info[2][0][0], leg_info[3][0][0], leg_info[4][0][0],
                                             leg_info[5][0][0], leg_info[6][0][0], leg_info[7][0][0], leg_info[8][0][0], leg_info[9][0][0])

            self.dQF = motor_to_joint_velocity(self.dQM[0], self.dQM[1], self.dQM[2], self.dQM[3], self.dQM[4],
                                               self.dQM[5], self.dQM[6], self.dQM[7], self.dQM[8], self.dQM[9])

            self.T = motor_to_joint_torque(leg_info[0][0][2], leg_info[1][0][2], leg_info[2][0][2], leg_info[3][0][2], leg_info[4][0][2],
                                           leg_info[5][0][2], leg_info[6][0][2], leg_info[7][0][2], leg_info[8][0][2], leg_info[9][0][2])
        except:
            print("Communication Fails!!!")
            return False  # communication fails

        # Write to shared memory
        data = {'joint_positions': self.Q,
                'joint_velocities': self.dQF,
                'joint_torques': self.T,
                'time_stamp': np.array([time.time()])}
        MM.LEG_STATE.set(data)

        return True  # communication succeeds

    def read_write_force(self, torques, positions, velocities):
        """
        Get present torque, position, velocity and send goal torque, position, velocity to the actuators
        """
        bear_goal_values = joint_to_motor_value(torques[0], torques[1], torques[2], torques[3], torques[4],
                                                torques[5], torques[6], torques[7], torques[8], torques[9],
                                                positions[0], positions[1], positions[2], positions[3], positions[4],
                                                positions[5], positions[6], positions[7], positions[8], positions[9],
                                                velocities[0], velocities[1], velocities[2], velocities[3], velocities[4],
                                                velocities[5], velocities[6], velocities[7], velocities[8], velocities[9])

        # check BEAR iq limit
        for idx, bear_id in enumerate(BEAR_LIST):
            if bear_goal_values[idx][0] > IQ_MAX:
                print("Setting to max torque for BEAR %02d!!!" % bear_id)
                bear_goal_values[idx][0] = IQ_MAX - 0.5
            elif bear_goal_values[idx][0] < IQ_MIN:
                print("Setting to min torque for BEAR %02d!!!" % bear_id)
                bear_goal_values[idx][0] = IQ_MIN + 0.5

        leg_info = None
        trial    = 0
        try:
            if DEMO:
                while leg_info is None and trial < self.trial_max:
                    leg_info = self.BEAR_controller.pbm.bulk_read_write(BEAR_LIST, ['present_position', 'present_velocity', 'present_iq'],
                                                                        ['goal_iq', 'goal_position', 'goal_velocity'], bear_goal_values)
                    trial += 1
                for idx in range(10):
                    self.IQ[idx] = MF.exp_filter(self.IQ[idx], leg_info[idx][0][2], 0.50)
                self.Tau = motor_to_joint_torque(self.IQ[0], self.IQ[1], self.IQ[2], self.IQ[3], self.IQ[4],
                                                 self.IQ[5], self.IQ[6], self.IQ[7], self.IQ[8], self.IQ[9])
            else:
                while leg_info is None and trial < self.trial_max:
                    leg_info = self.BEAR_controller.pbm.bulk_read_write(BEAR_LIST, ['present_position', 'present_velocity'],
                                                                        ['goal_iq', 'goal_position', 'goal_velocity'], bear_goal_values)
                    trial += 1

            for idx in range(10):
                self.dQM[idx] = MF.exp_filter(self.dQM[idx], leg_info[idx][0][1], 0.90)

            self.Q = motor_to_joint_position(leg_info[0][0][0], leg_info[1][0][0], leg_info[2][0][0], leg_info[3][0][0], leg_info[4][0][0],
                                             leg_info[5][0][0], leg_info[6][0][0], leg_info[7][0][0], leg_info[8][0][0], leg_info[9][0][0])

            self.dQF = motor_to_joint_velocity(self.dQM[0], self.dQM[1], self.dQM[2], self.dQM[3], self.dQM[4],
                                               self.dQM[5], self.dQM[6], self.dQM[7], self.dQM[8], self.dQM[9])
        except:
            print("Communication Fails!!!")
            return False  # communication fails

        # Write to shared memory
        data = {'joint_positions': self.Q,
                'joint_velocities': self.dQF,
                'joint_torques': self.Tau}
        MM.LEG_STATE.set(data)

        return True   # communication succeeds

    def read_write_position(self, positions):
        """
        Get present position, velocity, iq and send goal position to the actuators
        """
        bear_goal_position = joint_to_motor_position(positions[0], positions[1], positions[2], positions[3], positions[4],
                                                     positions[5], positions[6], positions[7], positions[8], positions[9])

        leg_info = None
        trial    = 0
        try:
            while leg_info is None and trial < self.trial_max:
                leg_info = self.BEAR_controller.pbm.bulk_read_write(BEAR_LIST, ['present_position', 'present_velocity'],
                                                                    ['goal_position'], bear_goal_position)
                trial += 1

            for idx in range(10):
                self.dQM[idx] = MF.exp_filter(self.dQM[idx], leg_info[idx][0][1], 0.90)

            self.Q = motor_to_joint_position(leg_info[0][0][0], leg_info[1][0][0], leg_info[2][0][0], leg_info[3][0][0], leg_info[4][0][0],
                                             leg_info[5][0][0], leg_info[6][0][0], leg_info[7][0][0], leg_info[8][0][0], leg_info[9][0][0])

            self.dQF = motor_to_joint_velocity(self.dQM[0], self.dQM[1], self.dQM[2], self.dQM[3], self.dQM[4],
                                               self.dQM[5], self.dQM[6], self.dQM[7], self.dQM[8], self.dQM[9])
        except:
            print("Communication Fails!!!")
            return False  # communication fails

        # Write to shared memory
        data = {'joint_positions': self.Q,
                'joint_velocities': self.dQF}
        MM.LEG_STATE.set(data)

        return True   # communication succeeds

    def read_write_torque(self, torques):
        """
        Get present position, velocity, iq and send goal torque to the actuators
        """
        bear_goal_iq = joint_to_motor_iq(torques[0], torques[1], torques[2], torques[3], torques[4],
                                         torques[5], torques[6], torques[7], torques[8], torques[9])

        # check BEAR iq limit
        for idx, bear_id in enumerate(BEAR_LIST):
            if bear_goal_iq[idx][0] > IQ_MAX:
                print("Setting to max torque for BEAR %02d!!!" % bear_id)
                bear_goal_iq[idx][0] = IQ_MAX - 0.5
            elif bear_goal_iq[idx][0] < IQ_MIN:
                print("Setting to min torque for BEAR %02d!!!" % bear_id)
                bear_goal_iq[idx][0] = IQ_MIN + 0.5

        leg_info = None
        trial    = 0
        try:
            while leg_info is None and trial < self.trial_max:
                leg_info = self.BEAR_controller.pbm.bulk_read_write(BEAR_LIST, ['present_position', 'present_velocity'],
                                                                    ['goal_iq'], bear_goal_iq)
                trial += 1
            for idx in range(10):
                self.dQM[idx] = MF.exp_filter(self.dQM[idx], leg_info[idx][0][1], 0.90)

            self.Q = motor_to_joint_position(leg_info[0][0][0], leg_info[1][0][0], leg_info[2][0][0], leg_info[3][0][0], leg_info[4][0][0],
                                             leg_info[5][0][0], leg_info[6][0][0], leg_info[7][0][0], leg_info[8][0][0], leg_info[9][0][0])

            self.dQF = motor_to_joint_velocity(self.dQM[0], self.dQM[1], self.dQM[2], self.dQM[3], self.dQM[4],
                                               self.dQM[5], self.dQM[6], self.dQM[7], self.dQM[8], self.dQM[9])
        except:
            print("Communication Fails!!!")
            return False  # communication fails

        # Write to shared memory
        data = {'joint_positions': self.Q,
                'joint_velocities': self.dQF}
        MM.LEG_STATE.set(data)

        return True  # communication succeeds


def main_loop():
    # Start motors
    for _ in range(6):
        bc.torque_enable_all(0)
        time.sleep(0.01)
    bc.start_drivers()
    bc.set_actuator_mode('position')
    comm_success = False
    change_count = 0

    commands = {'BEAR_enable':  np.array([0.]),
                'BEAR_mode':    np.array([-1.]),
                'goal_torques': np.zeros(10)}
    MM.LEG_COMMAND.set(commands)

    loop_freq            = 1000  # run at 1000 Hz
    damping_check_freq   = 1     # check damping at 1 Hz
    temp_vol_update_freq = 1     # update temperature and voltage at 1 Hz

    loop_duration            = 1. / loop_freq
    damping_check_duration   = 1. / damping_check_freq
    temp_vol_update_duration = 1. / temp_vol_update_freq

    print("====== The BEAR Actuator Communication Thread is running at", loop_freq, "Hz... ======")

    t0 = time.time()
    last_damping_check_time   = t0
    last_temp_vol_update_time = t0
    thread_run = False
    while True:
        loop_start_time = time.time()
        elapsed_time    = loop_start_time - t0

        if elapsed_time > 1:
            if not thread_run:
                MM.THREAD_STATE.set({'bear': np.array([1.0])}, opt='only')  # thread is running
                thread_run = True
            
            # check threading error
            if Bruce.thread_error():
                Bruce.stop_threading()
        
        # check BEAR damping
        if loop_start_time - last_damping_check_time > damping_check_duration:
            bc.get_torque_mode()
            if bc.is_damping_mode():
                raise BEAR_ESTOP_Exception
            last_damping_check_time = loop_start_time

        # update temperature & voltage
        if loop_start_time - last_temp_vol_update_time > temp_vol_update_duration:
            bc.update_present_temperature_and_voltage()
            last_temp_vol_update_time = loop_start_time

        commands = MM.LEG_COMMAND.get()
        if commands['BEAR_enable'][0] == 1.0:
            if commands['damping'][0] == 1.0:
                bc.set_damping_mode()

            # send desired commands to the actuators (MUST SEND COMMANDS BEFORE CHANGING MODE!!!)
            if commands['BEAR_mode'][0] == bc.BEAR_modes['force']:
                comm_success = bc.read_write_force(commands['goal_torques'], commands['goal_positions'], commands['goal_velocities'])
            elif commands['BEAR_mode'][0] == bc.BEAR_modes['position']:
                comm_success = bc.read_write_position(commands['goal_positions'])
            elif commands['BEAR_mode'][0] == bc.BEAR_modes['torque']:
                comm_success = bc.read_write_torque(commands['goal_torques'])

            if comm_success is True:
                # check to make sure we are in the correct mode
                if commands['BEAR_mode'][0] == bc.BEAR_mode:
                    # enable if not
                    if bc.torque_mode != bc.torque_modes['enable']:
                        bc.torque_enable_all(1)
                else:
                    change_count += 1
                    if change_count > 10:  # make sure the desired commands sent successfully before changing mode
                        bc.BEAR_mode = commands['BEAR_mode'][0]
                        bc.change_mode()
                        change_count = 0
        else:
            bc.update_present_status()

            # disable if not
            if bc.torque_mode != bc.torque_modes['disable']:
                bc.torque_enable_all(0)

        # check time to ensure that the motor controller stays at a consistent control loop.
        loop_end_time = loop_start_time + loop_duration
        present_time  = time.time()
        if present_time > loop_end_time:
            delay_time = 1000. * (present_time - loop_end_time)
            if delay_time > 1.:
                print(colored('Delayed ' + str(delay_time)[0:5] + ' ms at Te = ' + str(elapsed_time)[0:5] + ' s', 'yellow'))
        else:
            while time.time() < loop_end_time:
                pass


if __name__ == "__main__":
    # BRUCE SETUP
    Bruce = RDS.BRUCE()

    # BEAR Setup
    bc = bear_controller(port=BEAR_port, baudrate=BEAR_baudrate)

    try:
        main_loop()
    except BEAR_ERROR_Exception:
        print(colored('BEAR IN ERROR! Terminate Now!', 'red'))
        MM.THREAD_STATE.set({'bear': np.array([3.0])}, opt='only')  # BEAR in error
    except BEAR_ESTOP_Exception:
        print(colored('BEAR IN E-STOP! Terminate Now!', 'red'))
        MM.THREAD_STATE.set({'bear': np.array([4.0])}, opt='only')  # BEAR ESTOP
    except (NameError, KeyboardInterrupt) as error:
        MM.THREAD_STATE.set({'bear': np.array([0.0])}, opt='only')  # thread is stopped
    except Exception as error:
        print(error)
        MM.THREAD_STATE.set({'bear': np.array([2.0])}, opt='only')  # thread in error
    finally:
        bc.set_damping_mode()
        Bruce.stop_robot()
