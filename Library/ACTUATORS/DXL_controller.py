#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
This module is used to communicate with Dynamixel X series actuators on BRUCE
'''

import math
from dynamixel_sdk import *


# Unit
POSITION_UNIT = 0.00153398078  # radian/value
VELOCITY_UNIT = 0.02398082392  # (radian/s)/value - 0.229 rpm

# Control Table Address
# EEPROM
ADDR_X_ID                 = 7
ADDR_X_BAUDRATE           = 8
ADDR_X_DRIVE_MODE         = 10
ADDR_X_MODE               = 11
ADDR_X_HOMING_OFFSET      = 20
ADDR_X_VELOCITY_LIMIT     = 44
ADDR_X_POSITION_LIMIT_MAX = 48
ADDR_X_POSITION_LIMIT_MIN = 52

# RAM
ADDR_X_TORQUE_ENABLE        = 64
ADDR_X_P_GAIN_VELOCITY      = 76
ADDR_X_I_GAIN_VELOCITY      = 78
ADDR_X_P_GAIN_POSITION      = 84
ADDR_X_I_GAIN_POSITION      = 82
ADDR_X_D_GAIN_POSITION      = 80
ADDR_X_1ST_GAIN_FEEDFORWARD = 90
ADDR_X_2ND_GAIN_FEEDFORWARD = 88
ADDR_X_GOAL_POSITION        = 116
ADDR_X_GOAL_VELOCITY        = 104
ADDR_X_PRESENT_POSITION     = 132
ADDR_X_PRESENT_VELOCITY     = 128
ADDR_X_PROFILE_VELOCITY     = 112
ADDR_X_PROFILE_ACCELERATION = 108
ADDR_X_PRESENT_LOAD         = 126

# INDIRECT ADDRESS
ADDR_INDIRECTADDRESS_FOR_READ = 178
ADDR_INDIRECTDATA_FOR_READ_1  = 229
ADDR_INDIRECTDATA_FOR_READ_2  = ADDR_INDIRECTDATA_FOR_READ_1 + 4


class DynamixelController(object):
    def __init__(self, port, baudrate=2000000):

        # Protocol version
        self.PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.baudrate = baudrate  # Dynamixel default baudrate: 57600
        self.port = port

        self.port_handler = None
        self.packet_handler = None

        self.open_port()

    def open_port(self):

        # Initialize PortHandler
        self.port_handler = PortHandler(self.port)

        # Initialize PacketHandler
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.port_handler.openPort():
            # Set port baudrate
            if self.port_handler.setBaudRate(self.baudrate):
                return True
            else:
                pass
        else:
            print("Failed to open the Dynamixel port!")
            return False

    def close_port(self):
        # Close port
        self.port_handler.closePort()

    def ping(self, DXL_ID):
        # Ping Dynamixel and get Dynamixel model number
        model_number, comm_result, error = self.packet_handler.ping(self.port_handler, DXL_ID)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return model_number

    def set_mode(self, DXL_ID, mode):
        # Set the operation mode
        if mode == 'position':
            m = 3
        elif mode == 'velocity':
            m = 1
        elif mode == 'extended position':
            m = 4
        elif mode == 'PWM':
            m = 16
        else:
            print('Invalid operation mode.')
            return False

        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_X_MODE, m)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_baudrate(self, DXL_ID, val):
        # Set the communication baudrate
        if val == 0:
            print("Baudrate set to 9600.")
        elif val == 1:
            print("Baudrate set to 57600.")  # Dynamixel default
        elif val == 2:
            print("Baudrate set to 115200.")
        elif val == 3:
            print("Baudrate set to 1000000.")
        elif val == 4:
            print("Baudrate set to 2000000.")
        else:
            print('Invalid baudrate selection.')
            return False

        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_X_BAUDRATE, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_drive_mode(self, DXL_ID):
        # Set to velocity based profile, normal mode
        mode = 0
        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_X_DRIVE_MODE, mode)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_homing_offset(self, DXL_ID, homing_offset):
        # Write homing offset in radian between -pi/2 and pi/2 (only worked in joint mode/basic position mode)
        if abs(homing_offset) > math.pi / 2:
            print("Input out of range.")
            return False
        homing_offset = int(homing_offset / POSITION_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_HOMING_OFFSET, homing_offset)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_homing_offset(self, DXL_ID):
        # Read present homing offset in radian
        homing_offset, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_HOMING_OFFSET)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            if homing_offset > 0x7fffffff:  # two's complement (https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/264)
                homing_offset = homing_offset - 4294967296
            homing_offset *= POSITION_UNIT
            return homing_offset

    def torque_enable(self, DXL_ID, val):
        # Enable/dis-enable Dynamixel torque
        comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_X_TORQUE_ENABLE, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_enable(self, DXL_ID):
        # Get torque enable status of Dynamixel
        val, comm_result, error = self.packet_handler.read1ByteTxRx(self.port_handler, DXL_ID, ADDR_X_TORQUE_ENABLE)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return bool(val)

    def set_goal_position(self, DXL_ID, goal_position):
        # Write goal position in radian
        goal_position = int(goal_position / POSITION_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_GOAL_POSITION, goal_position)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_goal_position(self, DXL_ID):
        # Read present position in radian
        goal_position, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_GOAL_POSITION)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            goal_position *= POSITION_UNIT
            return goal_position

    def get_present_position(self, DXL_ID):
        # Read present position in radian
        present_position, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_PRESENT_POSITION)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            present_position *= POSITION_UNIT
            return present_position

    def set_goal_velocity(self, DXL_ID, goal_velocity):
        goal_velocity = int(goal_velocity / VELOCITY_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_GOAL_VELOCITY, goal_velocity)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_goal_velocity(self, DXL_ID):
        goal_velocity, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_GOAL_VELOCITY)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            if goal_velocity > 0x7fffffff:  # two's complement (https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/264)
                goal_velocity -= 4294967296
            goal_velocity *= VELOCITY_UNIT
            return goal_velocity

    def get_present_velocity(self, DXL_ID):
        present_velocity, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_PRESENT_VELOCITY)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            if present_velocity > 0x7fffffff:  # two's complement (https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/264)
                present_velocity -= 4294967296
            present_velocity *= VELOCITY_UNIT
            return present_velocity

    def set_profile_acceleration(self, DXL_ID, val):
        """
        Set the maximum acceleration of the profile
        Unit: 214.577 (rev/min^2)
        Range: 0 ~ 32767 (‘0’ stands for an infinite acceleration)

        :return:
        """
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_PROFILE_ACCELERATION, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_profile_velocity(self, DXL_ID, val):
        """
        Set the maximum velocity of the profile
        Unit: 0.229 (rev/min)
        Range: 0 ~ 32767 (‘0’ stands for an infinite velocity)

        :return:
        """
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_PROFILE_VELOCITY, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def set_p_gain_velocity(self, DXL_ID, val):
        # Set P gain for velocity loop
        if val > 16383 or val < 0:
            print("Input out of range.")
            return False
        comm_result, error = self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_P_GAIN_VELOCITY, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_p_gain_velocity(self, DXL_ID):
        # Read P gain for velocity loop
        val, comm_result, error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_P_GAIN_VELOCITY)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return val

    def set_i_gain_velocity(self, DXL_ID, val):
        # Set I gain for velocity loop
        if val > 16383 or val < 0:
            print("Input out of range.")
            return False
        comm_result, error = self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_I_GAIN_VELOCITY, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_i_gain_velocity(self, DXL_ID):
        # Read I gain for velocity loop
        val, comm_result, error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_I_GAIN_VELOCITY)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return val

    def set_p_gain_position(self, DXL_ID, val):
        # Set P gain for position loop
        if val > 16383 or val < 0:
            print("Input out of range.")
            return False
        comm_result, error = self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_P_GAIN_POSITION, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_p_gain_position(self, DXL_ID):
        # Read P gain for position loop
        val, comm_result, error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_P_GAIN_POSITION)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return val

    def set_i_gain_position(self, DXL_ID, val):
        # Set I gain for position loop
        if val > 16383 or val < 0:
            print("Input out of range.")
            return False
        comm_result, error = self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_I_GAIN_POSITION, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_i_gain_position(self, DXL_ID):
        # Read I gain for position loop
        val, comm_result, error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_I_GAIN_POSITION)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return val

    def set_d_gain_position(self, DXL_ID, val):
        # Set D gain for position loop
        if val > 16383 or val < 0:
            print("Input out of range.")
            return False
        comm_result, error = self.packet_handler.write2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_D_GAIN_POSITION, val)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_d_gain_position(self, DXL_ID):
        # Read D gain for position loop
        val, comm_result, error = self.packet_handler.read2ByteTxRx(self.port_handler, DXL_ID, ADDR_X_D_GAIN_POSITION)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return val

    def set_velocity_limit(self, DXL_ID, velocity_limit):
        velocity_limit = int(velocity_limit / VELOCITY_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_VELOCITY_LIMIT, velocity_limit)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_velocity_limit(self, DXL_ID):
        velocity_limit, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_VELOCITY_LIMIT)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            velocity_limit *= VELOCITY_UNIT
            return velocity_limit

    def set_position_limit_max(self, DXL_ID, position_limit_max):
        position_limit_max = int(position_limit_max / POSITION_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_POSITION_LIMIT_MAX, position_limit_max)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_position_limit_max(self, DXL_ID):
        position_limit_max, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_POSITION_LIMIT_MAX)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            position_limit_max *= POSITION_UNIT
            return position_limit_max

    def set_position_limit_min(self, DXL_ID, position_limit_min):
        position_limit_min = int(position_limit_min / POSITION_UNIT)
        comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_POSITION_LIMIT_MIN, position_limit_min)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            return True

    def get_position_limit_min(self, DXL_ID):
        position_limit_min, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_X_POSITION_LIMIT_MIN)
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return False
        elif 0 < error < 128:
            print("%s" % self.packet_handler.getRxPacketError(error))
            return False
        else:
            position_limit_min *= POSITION_UNIT
            return position_limit_min

    # ================================= Sync Read/Write Position/Velocity =================================
    def setup_sync_read(self, DXL_IDs):
        self.gsrPosition = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_X_PRESENT_POSITION, 4)
        self.gsrVelocity = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_X_PRESENT_VELOCITY, 4)

        # Add parameter storage for Dynamixel present position and velocity
        for id in DXL_IDs:
            dxl_addparam_result1 = self.gsrPosition.addParam(id)
            dxl_addparam_result2 = self.gsrVelocity.addParam(id)

            if dxl_addparam_result1 is not True or dxl_addparam_result2 is not True:
                print("[ID:%03d] groupSyncRead addparam failed" % id)

    def sync_read_position(self, DXL_IDs):
        num_ids = len(DXL_IDs)

        comm_result = self.gsrPosition.txRxPacket()
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return [False] * num_ids + [1]  # 1 means error

        present_position_info = []
        error = 0  # 0 means no error
        for id in DXL_IDs:
            dxl_getdata_result = self.gsrPosition.isAvailable(id, ADDR_X_PRESENT_POSITION, 4)

            if dxl_getdata_result is True:
                # Get Dynamixel present position value
                dxl_present_position = self.gsrPosition.getData(id, ADDR_X_PRESENT_POSITION, 4) * POSITION_UNIT
                present_position_info.append(dxl_present_position)
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                present_position_info.append(False)
                error = 1

        present_position_info.append(error)
        return present_position_info

    def sync_read_velocity(self, DXL_IDs):
        num_ids = len(DXL_IDs)

        comm_result = self.gsrVelocity.txRxPacket()
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(comm_result))
            return [False] * num_ids + [1]  # 1 means error

        present_velocity_info = []
        error = 0  # 0 means no error
        for id in DXL_IDs:
            dxl_getdata_result = self.gsrVelocity.isAvailable(id, ADDR_X_PRESENT_VELOCITY, 4)

            if dxl_getdata_result is True:
                # Get Dynamixel present velocity value
                dxl_present_velocity = self.gsrVelocity.getData(id, ADDR_X_PRESENT_VELOCITY, 4)
                if dxl_present_velocity > 0x7fffffff:  # two's complement (https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/264)
                    dxl_present_velocity -= 4294967296
                dxl_present_velocity *= VELOCITY_UNIT
                present_velocity_info.append(dxl_present_velocity)
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                present_velocity_info.append(False)
                error = 1

        present_velocity_info.append(error)
        return present_velocity_info

    def setup_sync_write(self):
        self.gswPosition = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_X_GOAL_POSITION, 4)
        self.gswVelocity = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_X_GOAL_VELOCITY, 4)

    def sync_write_position(self, DXL_IDs, goal_positions):
        num_ids = len(DXL_IDs)

        for i in range(num_ids):
            goal_position = int(goal_positions[i] / POSITION_UNIT)
            para1 = DXL_LOWORD(goal_position)
            para2 = DXL_HIWORD(goal_position)
            param_goal_position = [DXL_LOBYTE(para1), DXL_HIBYTE(para1), DXL_LOBYTE(para2), DXL_HIBYTE(para2)]
            dxl_addparam_result = self.gswPosition.addParam(DXL_IDs[i], param_goal_position)
            if dxl_addparam_result is not True:
                print("[ID:%03d] groupSyncWrite addparam failed" % id)

        # syncwrite goal position
        dxl_comm_result = self.gswPosition.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.gswPosition.clearParam()

    # sync read position and velocity together
    def setup_sync_read_all(self, DXL_IDs):
        self.gsrPosVel = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_INDIRECTDATA_FOR_READ_1, 4 + 4)

        for id in DXL_IDs:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ +  0, ADDR_X_PRESENT_POSITION + 0)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ +  2, ADDR_X_PRESENT_POSITION + 1)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ +  4, ADDR_X_PRESENT_POSITION + 2)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ +  6, ADDR_X_PRESENT_POSITION + 3)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ +  8, ADDR_X_PRESENT_VELOCITY + 0)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ + 10, ADDR_X_PRESENT_VELOCITY + 1)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ + 12, ADDR_X_PRESENT_VELOCITY + 2)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_INDIRECTADDRESS_FOR_READ + 14, ADDR_X_PRESENT_VELOCITY + 3)

            dxl_addparam_result = self.gsrPosVel.addParam(id)
            if dxl_addparam_result is not True:
                print("[ID:%03d] groupSyncRead addparam failed" % id)

    def sync_read_posvel(self, DXL_IDs):
        # num_ids = len(DXL_IDs)
        comm_result = self.gsrPosVel.txRxPacket()
        present_position_info = []
        present_velocity_info = []
        try:
            for id in DXL_IDs:
                # Get Dynamixel present position and velocity value
                dxl_present_position = self.gsrPosVel.getData(id, ADDR_INDIRECTDATA_FOR_READ_1, 4) * POSITION_UNIT
                dxl_present_velocity = self.gsrPosVel.getData(id, ADDR_INDIRECTDATA_FOR_READ_2, 4)

                if dxl_present_velocity > 0x7fffffff:  # two's complement (https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/264)
                    dxl_present_velocity -= 4294967296
                dxl_present_velocity *= VELOCITY_UNIT

                present_position_info.append(dxl_present_position)
                present_velocity_info.append(dxl_present_velocity)

            return present_position_info, present_velocity_info, 0  # 0 means no error
        except:
            # print("groupSyncRead position and velocity together failed")
            return False, False, 1  # 1 means error
