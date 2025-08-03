#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for homing calibration of BEAR actuators on BRUCE
'''

import time
from pybear import Manager
from Settings.BRUCE_macros import *


def home_motor(bear_id):
    print('Homing BEAR ID: {}'.format(bear_id))

    if bear_id == BEAR_HIP1_R or bear_id == BEAR_HIP1_L:
        nominal = PI_2
    elif bear_id == BEAR_HIP2_R or bear_id == BEAR_HIP2_L:
        nominal = ag - PI_2
    elif bear_id == BEAR_HIP3_R or bear_id == BEAR_HIP3_L:
        nominal = PI_2 - ag
    elif bear_id == BEAR_KNEE_R or bear_id == BEAR_KNEE_L:
        nominal = PI - ag - bg
        if bear_id == BEAR_KNEE_R:
            nominal *= -1.
    else:
        nominal = 0.

    raw_encoder_reading = 0.0
    n_samples = 10
    for _ in range(n_samples):
        raw_encoder_reading += pbm.get_present_position(bear_id)[0][0][0] / n_samples
        time.sleep(0.01)

    current_offset = pbm.get_homing_offset(bear_id)[0][0][0]

    new_offset = nominal + current_offset - raw_encoder_reading
    if new_offset > 2 ** 17:
        new_offset -= 2 ** 18
    if new_offset < -2 ** 17:
        new_offset += 2 ** 18

    if new_offset > 2.0 * PI:
        new_offset -= 2.0 * PI
    elif new_offset < -2.0 * PI:
        new_offset += 2.0 * PI

    diff_offset = new_offset - current_offset

    print('Current Offset: {}'.format(current_offset))
    print('New Offset: {}'.format(new_offset))
    print('Difference: {}'.format(diff_offset))

    new_joint_position = raw_encoder_reading + diff_offset

    print('Joint Position w/ New Offset: {}'.format(new_joint_position))

    confirm = input('Set new offset? (y/n)')
    if confirm == 'y':
        pbm.set_homing_offset((bear_id, new_offset))
        time.sleep(0.2)
        data = pbm.get_homing_offset(bear_id)[0][0][0]
        if abs(data - new_offset) > 0.01:
            print('Failed to write homing offset!!!')
        else:
            pbm.save_config(bear_id)
            print('Homing success.')
    else:
        print('Homing canceled.')
    print("")


if __name__ == '__main__':
    # nominal joint angle offsets
    ag = np.arccos((32.5 ** 2 + (a3 * 1000) ** 2 - 24.5 ** 2 - 204.79 ** 2) / 2. / 32.5 / (a3 * 1000))
    bg = np.arctan2(193.40, 50.5)

    pbm = Manager.BEAR(port=BEAR_port, baudrate=BEAR_baudrate)

    print('====== BRUCE Homing Calibration ======')
    for i in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
        home_motor(i)
