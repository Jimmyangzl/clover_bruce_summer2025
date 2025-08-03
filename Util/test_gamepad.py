#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for monitoring gamepad data
'''

import time
import Settings.BRUCE_data as RDS


if __name__ == '__main__':
    Bruce = RDS.BRUCE()
    
    while True:
        Bruce.update_gamepad_status()

        for key in list(Bruce.gamepad.keys()):
            if key in ['LX', 'LY', 'RX', 'RY']:
                print(key + ': {:.2f}'.format(Bruce.gamepad[key]))
            else:
                print(key + ': {:.0f}'.format(Bruce.gamepad[key]))

        for _ in range(len(list(Bruce.gamepad))):
            print('\033[1A', end='\x1b[2K')

        time.sleep(0.01)
