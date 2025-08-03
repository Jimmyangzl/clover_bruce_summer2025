#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Bluetooth communication with gamepad
'''

import os
import time
import socket
from termcolor import colored


class GamepadManager(object):
    def __init__(self, mac='E0:5A:1B:D1:01:76'):
        # bluetooth communication
        self.mac = mac  # gamepad MAC address
        self.s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.socket_timeout     = 0.5  # read/write timeout [second]
        self.connection_timeout = 5
        self.read_timeout_count = 0
        self.send_timeout_count = 0
        self.timeout_threshold  = 3
        self.is_connected       = False

        self.print_message(message='connecting')
        self.connect()

        # data from gamepad
        # [0xFF(1), REST_LENGTH(1), 0bUDLRABXY(1), 0b{LZ}{LS}{LSP}{LSM}{RZ}{RS}{RSP}{RSM}(1), 0b00{LS2}{RS2}{ST}{BK}{ALT}{FN}(1), LX(1), LY(1), RX(1), RY(1), CHECKSUM(1), 0xFE(1)]
        self.data4esp = [0xFF, 9,  0, 0, 0, 0, 0, 0, 0,  246, 0xFE]
        self.data4esp_length = len(self.data4esp)
        self.data4esp_length_1 = self.data4esp_length - 1
        self.data4esp_length_2 = self.data4esp_length - 2

        # data to gamepad 
        # [0xFF(1), REST_LENGTH(1), 0b0{BOOT}{DXL}{BEAR}{INITIALIZE}(1), 0b{ESTIMATION}{LOW_LEVEL}{HIGH_LEVEL}{TOP_LEVEL}(1), 0b000000{BRUCE_MODE}(1), BEAR_TEMPERATURE(1), BEAR_VOLTAGE(1), CHECKSUM(1), 0xFE(1)]
        self.data2esp = [0xFF, 7,  0, 0, 0, 0, 0,  248, 0xFE]

        # gamepad status
        self.button = {'U':   0,  # HAT_UP
                       'D':   0,  # HAT_DOWN
                       'L':   0,  # HAT_LEFT
                       'R':   0,  # HAT_RIGHT
                       'A':   0,
                       'B':   0,
                       'X':   0,
                       'Y':   0,

                       'LZ':  0,  # LEFT_AXIS_Z
                       'LS':  0,  # LEFT_SHOULDER
                       'LSP': 0,  # LEFT_SHOULDER_PLUS
                       'LSM': 0,  # LEFT_SHOULDER_MINUS
                       'LS2': 0,  # LEFT_SHOULDER_2
                       'RZ':  0,  # RIGHT_AXIS_Z
                       'RS':  0,  # RIGHT_SHOULDER
                       'RSP': 0,  # RIGHT_SHOULDER_PLUS
                       'RSM': 0,  # RIGHT_SHOULDER_MINUS
                       'RS2': 0,  # RIGHT_SHOULDER_2

                       'ST':  0,  # START
                       'BK':  0,  # BACK
                       'ALT': 0,
                       'FN':  0,
                       }
        self.axis = {'LX': 0,  # LEFT_AXIS_X
                     'LY': 0,  # LEFT_AXIS_Y
                     'RX': 0,  # RIGHT_AXIS_X
                     'RY': 0,  # RIGHT_AXIS_Y
                     }

    def connect(self):
        """
        Connect gamepad
        """
        os.system("rfkill unblock bluetooth")  # turn on bluetooth
        t0 = time.time()
        while 1:
            if (time.time() - t0 > self.connection_timeout):
                self.print_message(message='connection_timeout')
            try:
                self.s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.s.settimeout(self.socket_timeout)
                self.s.connect_ex((self.mac, 1))
                if len(self.s.recv(1024)) != 0:
                    self.is_connected = True
                    self.print_message(message='connected')
                    self.read_timeout_count = 0
                    self.send_timeout_count = 0
                    break
            except socket.error:
                pass

    def disconnect(self):
        """
        Disconnect gamepad
        """
        self.s.close()
        self.print_message(message='disconnected')
        os.system("rfkill block bluetooth")  # turn off bluetooth

    def reconnect(self):
        """
        Reconnect gamepad
        """
        self.disconnect()
        self.print_message(message='reconnecting')
        self.connect()

    def check_connection(self):
        if self.read_timeout_count > self.timeout_threshold or self.send_timeout_count > self.timeout_threshold:
            self.is_connected = False
            self.print_message(message='connection_lost')

    def send_data(self, BOOT=0, DXL=0, BEAR=0, INITIALIZE=0, ESTIMATION=0, LOW_LEVEL=0, HIGH_LEVEL=0, TOP_LEVEL=0, BRUCE_MODE=0, BEAR_TEMPERATURE=20, BEAR_VOLTAGE=16):
        """
        Send data to gamepad
        BOOT(1 bit), DXL(2 bit), BEAR(3 bit), INITIALIZE(1 bit), ESTIMATION(2 bit), LOW_LEVEL(2 bit), HIGH_LEVEL(2 bit), TOP_LEVEL(2 bit), BRUCE_MODE(2 bit)
        """
        self.data2esp[2] = int(BOOT)       << 6 | int(DXL)       << 4 | int(BEAR)       << 1 | int(INITIALIZE)  # 0 - off, 1 - on, 2 - error
        self.data2esp[3] = int(ESTIMATION) << 6 | int(LOW_LEVEL) << 4 | int(HIGH_LEVEL) << 2 | int(TOP_LEVEL)
        self.data2esp[4] = int(BRUCE_MODE)
        if -128 <= BEAR_TEMPERATURE <= 127:
            self.data2esp[5] = int(BEAR_TEMPERATURE + 256) if BEAR_TEMPERATURE < 0 else int(BEAR_TEMPERATURE)
        if 0 <= BEAR_VOLTAGE <= 25.5:
            self.data2esp[6] = int(BEAR_VOLTAGE * 10)
        self.data2esp[7] = self.checksum(self.data2esp[1:7])
        try:
            self.s.send(bytearray(self.data2esp))
            self.send_timeout_count = 0
            return True
        except socket.timeout:
            self.send_timeout_count += 1
            self.print_message(message='send_timeout_count')
            return False
        except:
            return False

    def read_data(self):
        """
        Read data from gamepad
        """
        try:
            packet = self.s.recv(32)
            packet_length = len(packet)
            if packet_length >= self.data4esp_length:
                for i in range(packet_length - self.data4esp_length_1):
                    if packet[i] == 0xFF:  # find first byte
                        self.data4esp[1:] = packet[i+1:i+self.data4esp_length]
                        if self.data4esp[1] == self.data4esp_length_2 and self.data4esp[-2] == self.checksum(self.data4esp[1:-2]) and self.data4esp[-1] == 0xFE:  # check length byte, sum byte & last byte
                            self.button['U'] = self.data4esp[2] >> 7 & 1
                            self.button['D'] = self.data4esp[2] >> 6 & 1
                            self.button['L'] = self.data4esp[2] >> 5 & 1
                            self.button['R'] = self.data4esp[2] >> 4 & 1
                            self.button['A'] = self.data4esp[2] >> 3 & 1
                            self.button['B'] = self.data4esp[2] >> 2 & 1
                            self.button['X'] = self.data4esp[2] >> 1 & 1
                            self.button['Y'] = self.data4esp[2] >> 0 & 1

                            self.button['LZ']  = self.data4esp[3] >> 7 & 1
                            self.button['LS']  = self.data4esp[3] >> 6 & 1
                            self.button['LSP'] = self.data4esp[3] >> 5 & 1
                            self.button['LSM'] = self.data4esp[3] >> 4 & 1
                            self.button['RZ']  = self.data4esp[3] >> 3 & 1
                            self.button['RS']  = self.data4esp[3] >> 2 & 1
                            self.button['RSP'] = self.data4esp[3] >> 1 & 1
                            self.button['RSM'] = self.data4esp[3] >> 0 & 1

                            self.button['LS2'] = self.data4esp[4] >> 5 & 1
                            self.button['RS2'] = self.data4esp[4] >> 4 & 1
                            self.button['ST']  = self.data4esp[4] >> 3 & 1
                            self.button['BK']  = self.data4esp[4] >> 2 & 1
                            self.button['ALT'] = self.data4esp[4] >> 1 & 1
                            self.button['FN']  = self.data4esp[4] >> 0 & 1

                            self.axis['LX'] = self.analog2axis(self.data4esp[5])
                            self.axis['LY'] = self.analog2axis(self.data4esp[6])
                            self.axis['RX'] = self.analog2axis(self.data4esp[7])
                            self.axis['RY'] = self.analog2axis(self.data4esp[8])

                            self.read_timeout_count = 0
                            return True
                        else:
                            self.print_message(message='bad_data')
                            return False
                self.print_message(message='bad_data')
                return False
            else:
                self.print_message(message='bad_data')
                return False
        except socket.timeout:
            self.read_timeout_count += 1
            self.print_message(message='read_timeout_count')
            return False
        except:
            return False
        
    @staticmethod
    def checksum(packet):
        """
        Calculate checksum
        """
        return 255 - sum(packet) % 256
    
    @staticmethod
    def analog2axis(val):
        """
        Convert [0, 255] to [-1, +1]
        """
        if val > 127:
            val = (128 - val) / 127
        elif val > 0:
            val = val / 127
        else:
            val = 0
        return val
    
    @staticmethod
    def print_message(message):
        """
        Print custom message
        """
        msg = '[BRUCE GAMEPAD | '

        # warning
        if message == 'connection_lost':
            msg += 'WARNING] :: Connection lost.'
            print(colored(msg, 'red'))
        elif message == 'connection_timeout':
            msg += 'WARNING] :: Connection timed out.'
            print(colored(msg, 'yellow'))
        elif message == 'send_timeout_count':
            msg += 'WARNING] :: Send timed out.'
            print(colored(msg, 'yellow'))
        elif message == 'read_timeout_count':
            msg += 'WARNING] :: Read timed out.'
            print(colored(msg, 'yellow'))
        elif message == 'bad_data':
            msg += 'WARNING] :: Data corrupted.'
            print(colored(msg, 'yellow'))

        # status
        elif message == 'connected':
            msg += 'STATUS]  :: Connected.'
            print(colored(msg, 'white'))
        elif message == 'disconnected':
            msg += 'STATUS]  :: Disconnected.'
            print(colored(msg, 'white'))
        elif message == 'connecting':
            msg += 'STATUS]  :: Connecting...'
            print(colored(msg, 'white'))
        elif message == 'reconnecting':
            msg += 'STATUS]  :: Reconnecting...'
            print(colored(msg, 'white'))
