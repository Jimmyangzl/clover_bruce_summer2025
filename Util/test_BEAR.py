#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Test communication with BEAR actuators
'''

import time
from pybear import Manager
from Settings.BRUCE_macros import *

if __name__ == '__main__':
    bear = Manager.BEAR(port=BEAR_port, baudrate=BEAR_baudrate)

    while True:
        loop_start_time = time.time()
        data = bear.bulk_read([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], ['present_position', 'present_velocity', 'present_iq'])  # read BEAR 1-10
        print(1. / (time.time() - loop_start_time))
    