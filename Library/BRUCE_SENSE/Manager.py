#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Serial communication with PICO
'''

import time
import serial
import struct
import numpy as np
from termcolor import colored


class SenseManager(object):
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        # serial communication
        self.port         = port
        self.baudrate     = baudrate
        self.ser          = None
        self.read_timeout = 0.001  # [sec]
        self.open_port()

        # data from PICO
        # [0xFF(1), REST_LENGTH(1), ACCEL_X(4), ACCEL_Y(4), ACCEL_Z(4), OMEGA_X(4), OMEGA_Y(4), OMEGA_Z(4), FOOT_CONTACT(1), CHECKSUM(1), 0xFE(1)]
        self.data4pico = [0xFF, 27,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0b10000000,  228, 0xFE]
        self.data4pico_length   = len(self.data4pico)        # 29
        self.data4pico_length_1 = self.data4pico_length - 1  # 28
        self.data4pico_length_2 = self.data4pico_length - 2  # 27

        # data to PICO 
        # [0xFF(1), REST_LENGTH(1), 0b00{COOLING_SPEED[4]}{PICO_MODE[2]}(1), CHECKSUM(1), 0xFE(1)]
        self.data2pico = [0xFF, 3,  0,  252, 0xFE]

        self.accel = np.zeros(3)
        self.omega = np.zeros(3)
        self.foot_contact = np.zeros(4)  # [RIGHT_TOE, RIGHT_HEEL, LEFT_TOE, LEFT_HEEL]

    def open_port(self):
        """
        Open the serial port
        """
        self.ser = serial.Serial(self.port,
                                 self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0)

    def close_port(self):
        """
        Close the serial port
        """
        if self.ser:
            self.ser.close()

    def read_data(self):
        """
        Read data from PICO
        """
        t0 = time.time()
        while self.ser.in_waiting:
            if time.time() - t0 > self.read_timeout:
                self.print_warning(warning='read_timeout')
                return False
            if self.ser.read(1)[0] == 0xFF:  # find first byte
                t0 = time.time()
                while self.ser.in_waiting < self.data4pico_length_1:
                    if time.time() - t0 > self.read_timeout:
                        self.print_warning(warning='read_timeout')
                        self.ser.reset_input_buffer()
                        return False
                for idx in range(self.data4pico_length_1):
                    self.data4pico[idx + 1] = self.ser.read(1)[0]
                self.ser.reset_input_buffer()
                if self.data4pico[1] == self.data4pico_length_2 and self.data4pico[-2] == self.checksum(self.data4pico[1:-2]) and self.data4pico[-1] == 0xFE:  # check length byte, sum byte & last byte
                    self.accel[0] = self.hex_to_float32(self.data4pico[2:6])
                    self.accel[1] = self.hex_to_float32(self.data4pico[6:10])
                    self.accel[2] = self.hex_to_float32(self.data4pico[10:14])
                    self.omega[0] = self.hex_to_float32(self.data4pico[14:18])
                    self.omega[1] = self.hex_to_float32(self.data4pico[18:22])
                    self.omega[2] = self.hex_to_float32(self.data4pico[22:26])
                    for idx in range(4):
                        self.foot_contact[idx] = self.data4pico[-3] >> idx & 1
                    return True
                else:
                    self.print_warning(warning='bad_data')
                    return False

    def send_data(self, pico_mode='nominal', cooling_speed=0):
        """
        Send data to PICO
        """
        if pico_mode == 'idle':
            pico_mode = 0
        elif pico_mode == 'nominal':
            pico_mode = 1
        elif pico_mode == 'reset':
            pico_mode = 2
        elif pico_mode == 'calibration':
            pico_mode = 3
            
        cooling_speed = int(cooling_speed) if 0 <= cooling_speed <= 10 else 0

        self.data2pico[2] = int(cooling_speed) << 2 | int(pico_mode)
        self.data2pico[3] = self.checksum(self.data2pico[1:3])
        self.ser.write(self.data2pico)

    @staticmethod
    def checksum(packet):
        return 255 - sum(packet) % 256

    @staticmethod
    def hex_to_float32(byte):
        return struct.unpack('f', bytearray(byte))[0]

    @staticmethod
    def print_warning(warning):
        msg = '[BRUCE SENSE | WARNING] :: '
        if warning == 'read_timeout':
            msg += 'Read timed out. Just move on.'
            print(colored(msg, 'yellow'))
        elif warning == 'bad_data':
            msg += 'Data corrupted. Just move on.'
            print(colored(msg, 'red'))
