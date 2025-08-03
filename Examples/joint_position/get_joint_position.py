#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Get joint positions (change 'q' to 'dq' for joint velocities)
'''

import time
import Settings.BRUCE_data as RDS
from Settings.BRUCE_macros import *


if __name__ == '__main__':
    # BRUCE setup
    Bruce = RDS.BRUCE()

    LINE_UP = '\033[1A'
    LINE_CLEAR = '\x1b[2K'
    while True:
        Bruce.update_arm_status()
        Bruce.update_leg_status()

        print("Right Arm: {:.2f} / {:.2f} / {:.2f}".format(Bruce.joint[SHOULDER_PITCH_R]['q'],
                                                           Bruce.joint[SHOULDER_ROLL_R]['q'],
                                                           Bruce.joint[ELBOW_YAW_R]['q']))
        print(" Left Arm: {:.2f} / {:.2f} / {:.2f}".format(Bruce.joint[SHOULDER_PITCH_L]['q'],
                                                           Bruce.joint[SHOULDER_ROLL_L]['q'],
                                                           Bruce.joint[ELBOW_YAW_L]['q']))
        print("Right Leg: {:.2f} / {:.2f} / {:.2f} / {:.2f} / {:.2f}".format(Bruce.joint[HIP_YAW_R]['q'],
                                                                             Bruce.joint[HIP_ROLL_R]['q'],
                                                                             Bruce.joint[HIP_PITCH_R]['q'],
                                                                             Bruce.joint[KNEE_PITCH_R]['q'],
                                                                             Bruce.joint[ANKLE_PITCH_R]['q']))
        print(" Left Leg: {:.2f} / {:.2f} / {:.2f} / {:.2f} / {:.2f}".format(Bruce.joint[HIP_YAW_L]['q'],
                                                                             Bruce.joint[HIP_ROLL_L]['q'],
                                                                             Bruce.joint[HIP_PITCH_L]['q'],
                                                                             Bruce.joint[KNEE_PITCH_L]['q'],
                                                                             Bruce.joint[ANKLE_PITCH_L]['q']))
        for _ in range(4):
            print(LINE_UP, end=LINE_CLEAR)

        time.sleep(0.01)
