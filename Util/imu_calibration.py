#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for IMU bias calibration
'''

import time
from Settings.BRUCE_macros import *
from Library.BRUCE_SENSE import Manager as sense_manager


def calibrate_IMU():
    sm.send_data(pico_mode='calibration')

    while sm.ser.in_waiting < sm.data4pico_length:
        pass
    sm.read_data()
    
    print("Accel Present Bias: {:.6f} {:.6f} {:.6f}".format(sm.accel[0], sm.accel[1], sm.accel[2]))
    print("Omega Present Bias: {:.6f} {:.6f} {:.6f}".format(sm.omega[0], sm.omega[1], sm.omega[2]))
    print()

    time.sleep(1)
    confirm = input('Calibrate IMU? (y/n) ')
    if confirm != 'y':
        exit()
    sm.send_data(pico_mode='calibration')


    i = 0
    while sm.ser.in_waiting < sm.data4pico_length:
        time.sleep(1)
        print('IMU in calibration ' +  '.' * i, end='\r')
        i += 1
    
    time.sleep(1)
    sm.read_data()
    print(end='\n')
    print("Accel New Bias: {:.6f} {:.6f} {:.6f}".format(sm.accel[0], sm.accel[1], sm.accel[2]))
    print("Omega New Bias: {:.6f} {:.6f} {:.6f}".format(sm.omega[0], sm.omega[1], sm.omega[2]))

    time.sleep(1)
    confirm = input('Save new bias? (y/n) ')
    if confirm != 'y':
        exit()
    
    sm.send_data(pico_mode='calibration')
    time.sleep(1)


if __name__ == '__main__':
    # Sense Setting
    sm = sense_manager.SenseManager(port=PICO_port, baudrate=PICO_baudrate)

    # clear buffer
    sm.send_data(pico_mode='idle')
    time.sleep(1)
    while (sm.ser.in_waiting):
        sm.ser.reset_input_buffer()
        time.sleep(1)

    try:
        calibrate_IMU()
    finally:
        sm.send_data(pico_mode='idle')