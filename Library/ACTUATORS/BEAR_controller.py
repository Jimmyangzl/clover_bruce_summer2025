#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
This module is used to communicate with BEAR actuators on BRUCE
'''

from pybear import Manager
from Settings.BRUCE_macros import *


class BEARController(object):
    def __init__(self, port, baudrate):
        self.baudrate = baudrate
        self.port = port
        self.pbm = Manager.BEAR(port=self.port, baudrate=self.baudrate, bulk_timeout=0.002)

    def start_drivers(self):
        # Set all gains and limits
        # right leg
        self.pbm.set_limit_iq_max((BEAR_HIP1_R, IQ_MAX), (BEAR_HIP2_R, IQ_MAX), (BEAR_HIP3_R, IQ_MAX), (BEAR_KNEE_R, IQ_MAX), (BEAR_ANKLE_R, IQ_MAX))

        self.pbm.set_limit_position_max((BEAR_HIP1_R, BEAR_HIP1_MAX), (BEAR_HIP2_R, BEAR_HIP2_MAX), (BEAR_HIP3_R, BEAR_HIP3_MAX), (BEAR_KNEE_R, BEAR_KNEE_MAX), (BEAR_ANKLE_R, BEAR_ANKLE_MAX))
        self.pbm.set_limit_position_min((BEAR_HIP1_R, BEAR_HIP1_MIN), (BEAR_HIP2_R, BEAR_HIP2_MIN), (BEAR_HIP3_R, BEAR_HIP3_MIN), (BEAR_KNEE_R, BEAR_KNEE_MIN), (BEAR_ANKLE_R, BEAR_ANKLE_MIN))

        self.pbm.set_p_gain_position((BEAR_HIP1_R, BEAR_HIP1_R_POS_P), (BEAR_HIP2_R, BEAR_HIP2_R_POS_P), (BEAR_HIP3_R, BEAR_HIP3_R_POS_P), (BEAR_KNEE_R, BEAR_KNEE_R_POS_P), (BEAR_ANKLE_R, BEAR_ANKLE_R_POS_P))
        self.pbm.set_i_gain_position((BEAR_HIP1_R, BEAR_HIP1_R_POS_I), (BEAR_HIP2_R, BEAR_HIP2_R_POS_I), (BEAR_HIP3_R, BEAR_HIP3_R_POS_I), (BEAR_KNEE_R, BEAR_KNEE_R_POS_I), (BEAR_ANKLE_R, BEAR_ANKLE_R_POS_I))
        self.pbm.set_d_gain_position((BEAR_HIP1_R, BEAR_HIP1_R_POS_D), (BEAR_HIP2_R, BEAR_HIP2_R_POS_D), (BEAR_HIP3_R, BEAR_HIP3_R_POS_D), (BEAR_KNEE_R, BEAR_KNEE_R_POS_D), (BEAR_ANKLE_R, BEAR_ANKLE_R_POS_D))

        self.pbm.set_p_gain_velocity((BEAR_HIP1_R, BEAR_HIP1_R_VEL_P), (BEAR_HIP2_R, BEAR_HIP2_R_VEL_P), (BEAR_HIP3_R, BEAR_HIP3_R_VEL_P), (BEAR_KNEE_R, BEAR_KNEE_R_VEL_P), (BEAR_ANKLE_R, BEAR_ANKLE_R_VEL_P))
        self.pbm.set_i_gain_velocity((BEAR_HIP1_R, BEAR_HIP1_R_VEL_I), (BEAR_HIP2_R, BEAR_HIP2_R_VEL_I), (BEAR_HIP3_R, BEAR_HIP3_R_VEL_I), (BEAR_KNEE_R, BEAR_KNEE_R_VEL_I), (BEAR_ANKLE_R, BEAR_ANKLE_R_VEL_I))
        self.pbm.set_d_gain_velocity((BEAR_HIP1_R, BEAR_HIP1_R_VEL_D), (BEAR_HIP2_R, BEAR_HIP2_R_VEL_D), (BEAR_HIP3_R, BEAR_HIP3_R_VEL_D), (BEAR_KNEE_R, BEAR_KNEE_R_VEL_D), (BEAR_ANKLE_R, BEAR_ANKLE_R_VEL_D))

        self.pbm.set_p_gain_force((BEAR_HIP1_R, BEAR_HIP1_R_FOR_P), (BEAR_HIP2_R, BEAR_HIP2_R_FOR_P), (BEAR_HIP3_R, BEAR_HIP3_R_FOR_P), (BEAR_KNEE_R, BEAR_KNEE_R_FOR_P), (BEAR_ANKLE_R, BEAR_ANKLE_R_FOR_P))
        self.pbm.set_i_gain_force((BEAR_HIP1_R, BEAR_HIP1_R_FOR_I), (BEAR_HIP2_R, BEAR_HIP2_R_FOR_I), (BEAR_HIP3_R, BEAR_HIP3_R_FOR_I), (BEAR_KNEE_R, BEAR_KNEE_R_FOR_I), (BEAR_ANKLE_R, BEAR_ANKLE_R_FOR_I))
        self.pbm.set_d_gain_force((BEAR_HIP1_R, BEAR_HIP1_R_FOR_D), (BEAR_HIP2_R, BEAR_HIP2_R_FOR_D), (BEAR_HIP3_R, BEAR_HIP3_R_FOR_D), (BEAR_KNEE_R, BEAR_KNEE_R_FOR_D), (BEAR_ANKLE_R, BEAR_ANKLE_R_FOR_D))

        self.pbm.set_p_gain_iq((BEAR_HIP1_R, IQ_P), (BEAR_HIP2_R, IQ_P), (BEAR_HIP3_R, IQ_P), (BEAR_KNEE_R, IQ_P), (BEAR_ANKLE_R, IQ_P))
        self.pbm.set_i_gain_iq((BEAR_HIP1_R, IQ_I), (BEAR_HIP2_R, IQ_I), (BEAR_HIP3_R, IQ_I), (BEAR_KNEE_R, IQ_I), (BEAR_ANKLE_R, IQ_I))
        self.pbm.set_d_gain_iq((BEAR_HIP1_R, IQ_D), (BEAR_HIP2_R, IQ_D), (BEAR_HIP3_R, IQ_D), (BEAR_KNEE_R, IQ_D), (BEAR_ANKLE_R, IQ_D))

        self.pbm.set_p_gain_id((BEAR_HIP1_R, IQ_P), (BEAR_HIP2_R, IQ_P), (BEAR_HIP3_R, IQ_P), (BEAR_KNEE_R, IQ_P), (BEAR_ANKLE_R, IQ_P))
        self.pbm.set_i_gain_id((BEAR_HIP1_R, IQ_I), (BEAR_HIP2_R, IQ_I), (BEAR_HIP3_R, IQ_I), (BEAR_KNEE_R, IQ_I), (BEAR_ANKLE_R, IQ_I))
        self.pbm.set_d_gain_id((BEAR_HIP1_R, IQ_D), (BEAR_HIP2_R, IQ_D), (BEAR_HIP3_R, IQ_D), (BEAR_KNEE_R, IQ_D), (BEAR_ANKLE_R, IQ_D))

        # left leg
        self.pbm.set_limit_iq_max((BEAR_HIP1_L, IQ_MAX), (BEAR_HIP2_L, IQ_MAX), (BEAR_HIP3_L, IQ_MAX), (BEAR_KNEE_L, IQ_MAX), (BEAR_ANKLE_L, IQ_MAX))

        self.pbm.set_limit_position_max((BEAR_HIP1_L, BEAR_HIP1_MAX), (BEAR_HIP2_L, BEAR_HIP2_MAX), (BEAR_HIP3_L, BEAR_HIP3_MAX), (BEAR_KNEE_L, BEAR_KNEE_MAX), (BEAR_ANKLE_L, BEAR_ANKLE_MAX))
        self.pbm.set_limit_position_min((BEAR_HIP1_L, BEAR_HIP1_MIN), (BEAR_HIP2_L, BEAR_HIP2_MIN), (BEAR_HIP3_L, BEAR_HIP3_MIN), (BEAR_KNEE_L, BEAR_KNEE_MIN), (BEAR_ANKLE_L, BEAR_ANKLE_MIN))

        self.pbm.set_p_gain_position((BEAR_HIP1_L, BEAR_HIP1_L_POS_P), (BEAR_HIP2_L, BEAR_HIP2_L_POS_P), (BEAR_HIP3_L, BEAR_HIP3_L_POS_P), (BEAR_KNEE_L, BEAR_KNEE_L_POS_P), (BEAR_ANKLE_L, BEAR_ANKLE_L_POS_P))
        self.pbm.set_i_gain_position((BEAR_HIP1_L, BEAR_HIP1_L_POS_I), (BEAR_HIP2_L, BEAR_HIP2_L_POS_I), (BEAR_HIP3_L, BEAR_HIP3_L_POS_I), (BEAR_KNEE_L, BEAR_KNEE_L_POS_I), (BEAR_ANKLE_L, BEAR_ANKLE_L_POS_I))
        self.pbm.set_d_gain_position((BEAR_HIP1_L, BEAR_HIP1_L_POS_D), (BEAR_HIP2_L, BEAR_HIP2_L_POS_D), (BEAR_HIP3_L, BEAR_HIP3_L_POS_D), (BEAR_KNEE_L, BEAR_KNEE_L_POS_D), (BEAR_ANKLE_L, BEAR_ANKLE_L_POS_D))

        self.pbm.set_p_gain_velocity((BEAR_HIP1_L, BEAR_HIP1_L_VEL_P), (BEAR_HIP2_L, BEAR_HIP2_L_VEL_P), (BEAR_HIP3_L, BEAR_HIP3_L_VEL_P), (BEAR_KNEE_L, BEAR_KNEE_L_VEL_P), (BEAR_ANKLE_L, BEAR_ANKLE_L_VEL_P))
        self.pbm.set_i_gain_velocity((BEAR_HIP1_L, BEAR_HIP1_L_VEL_I), (BEAR_HIP2_L, BEAR_HIP2_L_VEL_I), (BEAR_HIP3_L, BEAR_HIP3_L_VEL_I), (BEAR_KNEE_L, BEAR_KNEE_L_VEL_I), (BEAR_ANKLE_L, BEAR_ANKLE_L_VEL_I))
        self.pbm.set_d_gain_velocity((BEAR_HIP1_L, BEAR_HIP1_L_VEL_D), (BEAR_HIP2_L, BEAR_HIP2_L_VEL_D), (BEAR_HIP3_L, BEAR_HIP3_L_VEL_D), (BEAR_KNEE_L, BEAR_KNEE_L_VEL_D), (BEAR_ANKLE_L, BEAR_ANKLE_L_VEL_D))

        self.pbm.set_p_gain_force((BEAR_HIP1_L, BEAR_HIP1_L_FOR_P), (BEAR_HIP2_L, BEAR_HIP2_L_FOR_P), (BEAR_HIP3_L, BEAR_HIP3_L_FOR_P), (BEAR_KNEE_L, BEAR_KNEE_L_FOR_P), (BEAR_ANKLE_L, BEAR_ANKLE_L_FOR_P))
        self.pbm.set_i_gain_force((BEAR_HIP1_L, BEAR_HIP1_L_FOR_I), (BEAR_HIP2_L, BEAR_HIP2_L_FOR_I), (BEAR_HIP3_L, BEAR_HIP3_L_FOR_I), (BEAR_KNEE_L, BEAR_KNEE_L_FOR_I), (BEAR_ANKLE_L, BEAR_ANKLE_L_FOR_I))
        self.pbm.set_d_gain_force((BEAR_HIP1_L, BEAR_HIP1_L_FOR_D), (BEAR_HIP2_L, BEAR_HIP2_L_FOR_D), (BEAR_HIP3_L, BEAR_HIP3_L_FOR_D), (BEAR_KNEE_L, BEAR_KNEE_L_FOR_D), (BEAR_ANKLE_L, BEAR_ANKLE_L_FOR_D))

        self.pbm.set_p_gain_iq((BEAR_HIP1_L, IQ_P), (BEAR_HIP2_L, IQ_P), (BEAR_HIP3_L, IQ_P), (BEAR_KNEE_L, IQ_P), (BEAR_ANKLE_L, IQ_P))
        self.pbm.set_i_gain_iq((BEAR_HIP1_L, IQ_I), (BEAR_HIP2_L, IQ_I), (BEAR_HIP3_L, IQ_I), (BEAR_KNEE_L, IQ_I), (BEAR_ANKLE_L, IQ_I))
        self.pbm.set_d_gain_iq((BEAR_HIP1_L, IQ_D), (BEAR_HIP2_L, IQ_D), (BEAR_HIP3_L, IQ_D), (BEAR_KNEE_L, IQ_D), (BEAR_ANKLE_L, IQ_D))

        self.pbm.set_p_gain_id((BEAR_HIP1_L, IQ_P), (BEAR_HIP2_L, IQ_P), (BEAR_HIP3_L, IQ_P), (BEAR_KNEE_L, IQ_P), (BEAR_ANKLE_L, IQ_P))
        self.pbm.set_i_gain_id((BEAR_HIP1_L, IQ_I), (BEAR_HIP2_L, IQ_I), (BEAR_HIP3_L, IQ_I), (BEAR_KNEE_L, IQ_I), (BEAR_ANKLE_L, IQ_I))
        self.pbm.set_d_gain_id((BEAR_HIP1_L, IQ_D), (BEAR_HIP2_L, IQ_D), (BEAR_HIP3_L, IQ_D), (BEAR_KNEE_L, IQ_D), (BEAR_ANKLE_L, IQ_D))

        print("BEAR driver started.")

    def set_mode(self, mode_bear):
        """
        Set the actuator into a desired mode.
        mode_bear for all bear actuators
        :param str mode_bear: Desired mode. (torque, velocity, position, force)
        """
        if mode_bear == 'position':
            m = 2
            print('Position mode for BEAR actuators.')
        elif mode_bear == 'velocity':
            m = 1
            print('Velocity mode for BEAR actuators.')
        elif mode_bear == 'torque':
            m = 0
            print('Torque mode for BEAR actuators.')
        elif mode_bear == 'force':
            m = 3
            print('Force mode for BEAR actuators.')
        else:
            m = -1
            print('Invalid operation mode for BEAR actuators.')

        if m != -1:
            self.pbm.set_mode((BEAR_HIP1_R, m), (BEAR_HIP2_R, m), (BEAR_HIP3_R, m), (BEAR_KNEE_R, m), (BEAR_ANKLE_R, m),
                              (BEAR_HIP1_L, m), (BEAR_HIP2_L, m), (BEAR_HIP3_L, m), (BEAR_KNEE_L, m), (BEAR_ANKLE_L, m))

    def torque_enable_all(self, val):
        """
        Torque enable all.
        :param int val: Enable/disable torque. (0: disable, 1: enable)
        """
        if val == 0 or val == 1:
            self.pbm.set_torque_enable((BEAR_HIP1_R, val), (BEAR_HIP2_R, val), (BEAR_HIP3_R, val), (BEAR_KNEE_R, val), (BEAR_ANKLE_R, val),
                                       (BEAR_HIP1_L, val), (BEAR_HIP2_L, val), (BEAR_HIP3_L, val), (BEAR_KNEE_L, val), (BEAR_ANKLE_L, val))
        else:
            print('Invalid torque enable command!!!')

        enable_info = self.pbm.get_torque_enable(BEAR_HIP1_R, BEAR_HIP2_R, BEAR_HIP3_R, BEAR_KNEE_R, BEAR_ANKLE_R,
                                                 BEAR_HIP1_L, BEAR_HIP2_L, BEAR_HIP3_L, BEAR_KNEE_L, BEAR_ANKLE_L)
        print('BEAR enable info:', enable_info)

    def torque_enable(self, bear_id, val):
        """
        Torque enable.
        :param int val: Enable/disable torque. (0: disable, 1: enable)
        """
        if val == 0 or val == 1:
            self.pbm.set_torque_enable((bear_id, val))
        else:
            print('Invalid torque enable command!!!')

        enable_info = self.pbm.get_torque_enable(bear_id)
        print('BEAR %02d enable info:' % bear_id, enable_info)

    def set_damping_mode(self):
        """
        Set the joints into damping mode.
        """
        val = 3
        self.pbm.set_torque_enable((BEAR_HIP1_R, val), (BEAR_HIP2_R, val), (BEAR_HIP3_R, val), (BEAR_KNEE_R, val), (BEAR_ANKLE_R, val),
                                   (BEAR_HIP1_L, val), (BEAR_HIP2_L, val), (BEAR_HIP3_L, val), (BEAR_KNEE_L, val), (BEAR_ANKLE_L, val))

    def get_torque_mode(self):
        """
        Get BEAR torque mode
        :return int: (0: disable, 1: enable, 2: error, 3: damping)
        """
        info = self.pbm.get_torque_enable(BEAR_KNEE_R)
        return info[0][0]

    def is_damping_mode(self):
        """
        Get the status of damping mode
        """
        val = self.get_torque_mode()
        if val == 3:
            return True
        else:
            return False
