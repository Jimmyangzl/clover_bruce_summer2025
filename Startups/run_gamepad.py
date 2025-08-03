#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for communication with BRUCE gamepad
'''

import os
import time
import Startups.memory_manager as MM
from Play.config import *
from Settings.BRUCE_macros import *
from Library.BRUCE_GAMEPAD import Manager as gamepad_manager


def main_loop():
    booted       = False
    gamepad_data = MM.GAMEPAD_STATE.get()
    input_data   = MM.USER_COMMAND.get()
    leg_data     = MM.LEG_STATE.get()
    leg_command  = MM.LEG_COMMAND.get()
    thread_error = False

    send_data_frequency = 10
    send_data_dt        = 1. / send_data_frequency
    last_send_data_time = 0

    while True:
        gm.check_connection()
        if gm.is_connected:
            # set gamepad states to shared memory
            if gm.read_data():
                for key in list(gm.button.keys()):
                    gamepad_data[key] = np.array([gm.button[key]])
                for key in list(gm.axis.keys()):
                    gamepad_data[key] = np.array([gm.axis[key]])
                MM.GAMEPAD_STATE.set(gamepad_data)

            # bootup/terminate
            if booted:
                if (gm.button['LZ'] and gm.button['RZ']) or thread_error:
                    booted = False
                    os.system("gnome-terminal -- '/home/khadas/BRUCE/BRUCE-OP/Play/terminate.sh'")
                    print('BRUCE is terminated.')
                elif gm.button['ST'] and thread_data['top_level'][0] == 0.0:
                    os.system("kill -18 $(pgrep bootup)")
            else:
                if gm.button['LZ'] and gm.button['ST']:
                    booted = True
                    
                    # reset thread status
                    for key in list(thread_data.keys()):
                        thread_data[key][0] = 0.0
                    MM.THREAD_STATE.set(thread_data)
                    thread_error = False
                    os.system("gnome-terminal -- '/home/khadas/BRUCE/BRUCE-OP/Play/bootup_gamepad.sh'")
                    print('')
                    print('BRUCE is launched.')

            # send status to gamepad
            if time.time() - last_send_data_time >= send_data_dt:
                last_send_data_time = time.time()

                thread_data = MM.THREAD_STATE.get()
                for key in list(thread_data.keys()):
                    if thread_data[key][0] == 2.0:
                        thread_error = True
                        break

                if thread_data['bear'][0] == 3.0 or thread_data['bear'][0] == 4.0:
                    thread_error = True

                if booted:
                    if thread_data['bear'][0] == 1.0 and leg_command['BEAR_enable'][0] == 0.0:
                        leg_command = MM.LEG_COMMAND.get()
                    if thread_data['top_level'][0] == 1.0:
                        leg_data   = MM.LEG_STATE.get()
                        input_data = MM.USER_COMMAND.get()
                gm.send_data(            BOOT = booted,
                                          DXL = thread_data['dxl'][0],
                                         BEAR = thread_data['bear'][0],
                                   INITIALIZE = leg_command['BEAR_enable'][0],
                                   ESTIMATION = thread_data['estimation'][0] or thread_data['sense'][0],
                                    LOW_LEVEL = thread_data['low_level'][0],
                                   HIGH_LEVEL = thread_data['high_level'][0],
                                    TOP_LEVEL = thread_data['top_level'][0],
                                   BRUCE_MODE = input_data['mode'][0],
                             BEAR_TEMPERATURE = leg_data['temperature'][0],
                                 BEAR_VOLTAGE = leg_data['voltage'][0])
                
            time.sleep(0.01)
        else:
            gm.reconnect()


if __name__ == "__main__":
    if GAMEPAD:
        gm = gamepad_manager.GamepadManager(mac=GAMEPAD_mac)
        try:
            main_loop()
        except Exception as error:
            print(error)
            gm.disconnect()