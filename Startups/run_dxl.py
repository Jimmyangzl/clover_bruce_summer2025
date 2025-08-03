#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for communication with Dynamixel X series actuators on BRUCE
'''

import time
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
from termcolor import colored
from Settings.BRUCE_macros import *
from Library.ACTUATORS.DXL_controller import DynamixelController


class dxl_controller:
    def __init__(self):
        # motor managers
        self.DXL_controller = None

        # motor port and baudrates
        self.DXL_port = '/dev/ttyUSB1'
        self.DXL_baudarte = 2000000

        # motor modes
        self.DXL_mode = 3
        self.DXL_modes = {'position': 3, 'velocity': 1, 'extended position': 4, 'PWM': 16}

        # safety check
        self.th_mode = False

        # arm info
        self.num_motors = len(DXL_LIST)
        self.pos = None
        self.vel = None
        self.Q_arm   = np.zeros(6)  # raw arm joint position array
        self.dQ_arm  = np.zeros(6)  # raw arm joint velocity array
        self.dQM_arm = np.zeros(6)  # filtered arm motor velocity array
        self.dQF_arm = np.zeros(6)  # filtered arm joint velocity array

        self.trial_max = 2  # maximum DXL communication trial number before timeout

    def close(self):
        """
        Close Dynamixel serial port
        """
        self.DXL_controller.close_port()

    def start_drivers(self):
        """
        Start DXL.
        Ping all motors and then config their PIDs
        :return: None
        """
        self.DXL_controller = DynamixelController(self.DXL_port, self.DXL_baudarte)

        for i, joint_id in enumerate(DXL_LIST):
            trail = 0
            check = False
            print("Pinging DXL motor %d..." % joint_id)
            while not check:
                if self.DXL_controller.ping(joint_id):
                    check = True
                    print('Pinging DXL motor %d Succeed.' % joint_id)
                else:
                    trail += 1
                    if trail > 6:
                        print('ERROR: DXL motor %d offline.' % joint_id)
                        break
                    time.sleep(0.1)

            self.DXL_controller.set_p_gain_position(joint_id, DXL_POS_P[i])
            self.DXL_controller.set_i_gain_position(joint_id, DXL_POS_I[i])
            self.DXL_controller.set_d_gain_position(joint_id, DXL_POS_D[i])

        self.DXL_controller.setup_sync_read(DXL_LIST)
        self.DXL_controller.setup_sync_read_all(DXL_LIST)
        self.DXL_controller.setup_sync_write()
        print("DXL motor driver started.")

    def set_actuator_mode(self, mode_dxl='position'):
        """
        Set the actuator into a desired mode.
        mode_dxl for all dxl motors
        :param str mode_dxl: Desired mode. (velocity, position, extended position, PWM)
        """
        for joint_id in DXL_LIST:
            self.DXL_controller.set_mode(joint_id, mode_dxl)
            self.DXL_controller.set_mode(joint_id, mode_dxl)
        self.DXL_mode = self.DXL_modes[mode_dxl]

    def torque_enable_all(self, val):
        """
        Torque enable.
        :param int val: Enable/disable torque. (0: disable, 1: enable)
        """
        for joint_id in DXL_LIST:
            self.DXL_controller.torque_enable(joint_id, val)

            if self.DXL_controller.get_enable(joint_id):
                print('DXL motor %d enabled' % joint_id)
            else:
                print('DXL motor %d disabled' % joint_id)

    def torque_enable(self, ID, val):
        """
        Torque enable.
        :param int val: Enable/disable torque. (0: disable, 1: enable)
        :param int ID: The ID of the target actuator
        """
        self.DXL_controller.torque_enable(ID, val)
        if self.DXL_controller.get_enable(ID):
            print('DXL motor %d enabled' % ID)
        else:
            print('DXL motor %d disabled' % ID)

    def update_present_status(self):
        """
        Get present position and velocity.
        """
        self.pos, self.vel, error = self.DXL_controller.sync_read_posvel(DXL_LIST)

        if error == 0:
            self.Q_arm = self.motor2joint_position(np.array(self.pos))
            self.dQ_arm = self.motor2joint_velocity(np.array(self.vel))

            for idx in range(self.num_motors):
                self.dQF_arm[idx] = MF.exp_filter(self.dQF_arm[idx], self.dQ_arm[idx], 0.7)

            # Write to shared memory
            data_arm = {'joint_positions': self.Q_arm,
                        'joint_velocities': self.dQF_arm}
            MM.ARM_STATE.set(data_arm)

    def change_mode(self):
        """
        Change the mode of the actuators on the limb.
        """
        no_error = True
        dxl_mode = None
        if self.DXL_mode == 3:
            dxl_mode = 'position'
        elif self.DXL_mode == 1:
            dxl_mode = 'velocity'
        elif self.DXL_mode == 4:
            dxl_mode = 'extended position'
        elif self.DXL_mode == 16:
            dxl_mode = 'PWM'
        else:
            no_error = False

        if no_error:
            print('Changing DXL mode to %s!' % dxl_mode)
            for joint_id in DXL_LIST:
                self.DXL_controller.set_mode(joint_id, dxl_mode)
        else:
            print('Invalid DXL operation mode.')

    def set_command_position(self, commands):
        """
        Send commanded positions to the actuators.
        :param commands: num_motors x 1 numpy array of position commands in radians
        """
        self.DXL_controller.sync_write_position(DXL_LIST, self.joint2motor_position(commands))

    @staticmethod
    def joint2motor_position(cmd):
        q = np.copy(cmd)
        q[2] = -q[2]
        q[5] = -q[5]
        return q + np.array(DXL_OFFSET)

    @staticmethod
    def joint2motor_velocity(cmd):
        dq = np.copy(cmd)
        dq[2] = -dq[2]
        dq[5] = -dq[5]
        return dq

    @staticmethod
    def motor2joint_position(cmd):
        m = np.copy(cmd)
        m = m - np.array(DXL_OFFSET)
        m[2] = -m[2]
        m[5] = -m[5]
        return m

    @staticmethod
    def motor2joint_velocity(cmd):
        dm = np.copy(cmd)
        dm[2] = -dm[2]
        dm[5] = -dm[5]
        return dm


def main_loop():
    # Start motors
    dc.start_drivers()
    dc.torque_enable_all(0)
    dc.set_actuator_mode('position')
    enable_flag = False

    commands = {'DXL_enable': np.array([0.]),
                'DXL_mode': np.array([-1.])}
    MM.ARM_COMMAND.set(commands)

    loop_freq     = 50  # run at 50 Hz
    loop_duration = 1. / loop_freq

    print("======= The DXL Actuator Communication Thread is running at", loop_freq, "Hz... =======")

    t0 = time.time()
    thread_run = False
    while True:
        loop_start_time = time.time()
        elapsed_time = loop_start_time - t0

        if elapsed_time > 1:
            if not thread_run:
                MM.THREAD_STATE.set({'dxl': np.array([1.0])}, opt='only')  # thread is running
                thread_run = True

            # check threading error
            if Bruce.thread_error():
                Bruce.stop_threading()

        commands = MM.ARM_COMMAND.get()
        if commands['DXL_enable'][0] == 1.0:
            # send desired commands to the actuators
            if dc.DXL_mode == dc.DXL_modes['position']:
                dc.set_command_position(commands['goal_positions'])

            if not enable_flag:
                dc.torque_enable_all(1)
                enable_flag = True
        else:
            if enable_flag:
                dc.torque_enable_all(0)
                enable_flag = False

        dc.update_present_status()

        # check time to ensure that the motor controller stays at a consistent control loop.
        loop_end_time = loop_start_time + loop_duration
        present_time = time.time()
        if present_time > loop_end_time:
            delay_time = 1000 * (present_time - loop_end_time)
            if delay_time > 1.:
                print(colored('Delayed ' + str(delay_time)[0:5] + ' ms at Te = ' + str(elapsed_time)[0:5] + ' s', 'yellow'))
        else:
            while time.time() < loop_end_time:
                pass


if __name__ == "__main__":
    # BRUCE SETUP
    Bruce = RDS.BRUCE()
    
    # DXL SETUP
    dc = dxl_controller()
    dc.DXL_port = DXL_port
    dc.DXL_baudarte = DXL_baudrate

    try:
        main_loop()
    except (NameError, KeyboardInterrupt) as error:
        MM.THREAD_STATE.set({'dxl': np.array([0.0])}, opt='only')  # thread is stopped
    except Exception as error:
        print(error)
        MM.THREAD_STATE.set({'dxl': np.array([2.0])}, opt='only')  # thread in error
    finally:
        dc.torque_enable_all(0)
