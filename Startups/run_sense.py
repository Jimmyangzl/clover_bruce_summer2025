#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for communication with bruce sense
'''

import time
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
from Play.config import *
from termcolor import colored
from Settings.BRUCE_macros import *
from Play.Walking.walking_macros import *
from Library.BRUCE_SENSE import Manager as sense_manager


def IMU_check():
    import statistics

    print('Checking IMU ...')
    IMU_error = True
    while IMU_error:
        data  = []
        count = 0
        while count < 500:
            # get sensor data
            if sm.read_data():
                data.append(sm.accel[0])
                count += 1
        data_std = statistics.stdev(data)
        if data_std > 1.0:
            print(data_std)
            print(colored('IMU Error! Resetting ...', 'red'))
            sm.send_data(pico_mode='reset')
            time.sleep(0.5)
            sm.send_data(pico_mode='nominal')
            print('Checking IMU Again ...')
        else:
            print('IMU OK!')
            IMU_error = False


def contact_check():
    print('Checking Foot Contact ...')
    count = 0
    while count < 100:
        # get sensor data
        if sm.read_data():
            if np.any(sm.foot_contact[0:2]) or np.any(sm.foot_contact[2:4]):
                count += 1
    print('Foot On Ground!')


def main_loop():
    # BRUCE Setup
    Bruce = RDS.BRUCE()

    # Parameters
    loop_freq               = 500  # run at 500 Hz
    cooling_update_freq     = 1    # update cooling speed at 1 Hz
    loop_duration           = 1. / loop_freq
    cooling_update_duration = 1. / cooling_update_freq

    # IMU Measurements
    gravity_accel = 9.81                       # gravity  acceleration
    accel = np.array([0., 0., gravity_accel])  # filtered accelerometer reading
    omega = np.array([0., 0., 0.])             # filtered gyroscope     reading

    # Start Communication
    print("====== The Sense Communication Thread is running at", loop_freq, "Hz... ======")

    t0 = Bruce.get_time()
    last_cooling_update_time = t0
    thread_run = False
    while True:
        loop_start_time = Bruce.get_time()
        elapsed_time    = loop_start_time - t0

        if elapsed_time > 1:
            if not thread_run:
                MM.THREAD_STATE.set({'sense': np.array([1.0])}, opt='only')  # thread is running
                thread_run = True

            # check threading error
            if Bruce.thread_error():
                Bruce.stop_threading()

        # get sensor data
        sm.read_data()
        for idx in range(3):
            accel[idx] = MF.exp_filter(accel[idx], sm.accel[idx], 0.00)
            omega[idx] = MF.exp_filter(omega[idx], sm.omega[idx], 0.00)
        
        # save data
        sense_data = {'imu_acceleration': accel,
                      'imu_ang_rate':     omega,
                      'foot_contacts':    sm.foot_contact}
        MM.SENSE_STATE.set(sense_data)

        # send cooling speed info to pico
        if loop_start_time - last_cooling_update_time > cooling_update_duration:
            user_data = MM.USER_COMMAND.get()
            sm.send_data(cooling_speed=user_data['cooling_speed'][0])
            last_cooling_update_time = loop_start_time

        # check time to ensure that the thread stays at a consistent running loop
        loop_end_time = loop_start_time + loop_duration
        present_time  = Bruce.get_time()
        if present_time > loop_end_time:
            delay_time = 1000 * (present_time - loop_end_time)
            if delay_time > 1.:
                print(colored('Delayed ' + str(delay_time)[0:5] + ' ms at Te = ' + str(elapsed_time)[0:5] + ' s', 'yellow'))
        else:
            while Bruce.get_time() < loop_end_time:
                pass


if __name__ == '__main__':
    # PICO Setup
    sm = sense_manager.SenseManager(port=PICO_port, baudrate=PICO_baudrate)
    sm.send_data(pico_mode='nominal')  # run PICO

    IMU_check()
    if not DEMO:
        contact_check()
        
    try:
        main_loop()
    except (NameError, KeyboardInterrupt) as error:
        MM.THREAD_STATE.set({'sense': np.array([0.0])}, opt='only')  # thread is stopped
    except Exception as error:
        print(error)
        MM.THREAD_STATE.set({'sense': np.array([2.0])}, opt='only')  # thread in error
    finally:
        sm.send_data(pico_mode='idle')  # set PICO to idle mode