#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Interface between Gazebo and Python
'''

import time
import ctypes
import socket
import posix_ipc
import numpy as np
from Util import math_function as MF
from Library.SHARED_MEMORY import Manager as shmx


class ModelParameters(ctypes.Structure):
    _fields_ = [('operating_mode',    ctypes.c_int),
                ('state_update_rate', ctypes.c_double)]


class WorldParameters(ctypes.Structure):
    _fields_ = [('step_size',             ctypes.c_double),
                ('real_time_update_rate', ctypes.c_double)]


class GazeboInterface(object):
    def __init__(self, robot_name, num_joints, num_contact_sensors=None):
        """
        robot_name:          name of robot (as defined in the gazebo world file)
        num_joints:          number of joints of robot (as defined in the robot's sdf file)
        num_contact_sensors: number of contact sensors (as defined in the robot's sdf file)
        """
        self.robot_name = robot_name
        self.world_name = "world"
        self.dir = "/tmp/"

        self.num_joints = num_joints
        self.num_contact_sensors = num_contact_sensors or 1
        self.max_timeouts = 10

        self.initialize_shared_memory()
        self.initialize_clients()

    def initialize_shared_memory(self):
        self.WORLD_PARAMETER = shmx.SHMEMSEG(robot_name='GAZ', seg_name='WORLD_PARAMS', init=False)
        self.WORLD_PARAMETER.add_block(name='data', data=np.array(WorldParameters))

        self.MODEL_PARAMETER = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='MODEL_PARAMS', init=False)
        self.MODEL_PARAMETER.add_block(name='data', data=np.array(ModelParameters))

        self.JOINT_STATE = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='STATES', init=False)
        self.JOINT_STATE.add_block(name='time',     data=np.zeros(1))
        self.JOINT_STATE.add_block(name='position', data=np.zeros(self.num_joints))
        self.JOINT_STATE.add_block(name='velocity', data=np.zeros(self.num_joints))
        self.JOINT_STATE.add_block(name='force',    data=np.zeros(self.num_joints))

        self.JOINT_TORQUE_COMMAND = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='FORCE_COMMS', init=False)
        self.JOINT_TORQUE_COMMAND.add_block(name='data', data=np.zeros(self.num_joints))

        self.POSITION_PID_GAIN = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='PID_GAINS', init=False)
        self.POSITION_PID_GAIN.add_block(name='data', data=np.zeros(3 * self.num_joints))

        self.JOINT_POSITION_COMMAND = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='POS_COMMS', init=False)
        self.JOINT_POSITION_COMMAND.add_block(name='data', data=np.zeros(self.num_joints))

        self.JOINT_LIMIT = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='JOINT_LIMITS', init=False)
        self.JOINT_LIMIT.add_block(name='data', data=np.zeros(2 * self.num_joints))

        self.TORQUE_LIMIT = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='EFFORT_LIMITS', init=False)
        self.TORQUE_LIMIT.add_block(name='data', data=np.zeros(2 * self.num_joints))

        self.BODY_POSE = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='BODY_POSE', init=False)
        self.BODY_POSE.add_block(name='time',         data=np.zeros(1))
        self.BODY_POSE.add_block(name='position',     data=np.zeros(3))
        self.BODY_POSE.add_block(name='quaternion',   data=np.zeros(4))
        self.BODY_POSE.add_block(name='euler_angles', data=np.zeros(3))
        self.BODY_POSE.add_block(name='velocity',     data=np.zeros(3))

        self.IMU_STATE = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='IMU_STATES', init=False)
        self.IMU_STATE.add_block(name='time',     data=np.zeros(1))
        self.IMU_STATE.add_block(name='accel',    data=np.zeros(3))
        self.IMU_STATE.add_block(name='ang_rate', data=np.zeros(3))

        self.LIMB_CONTACT = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='LIMB_CONTACTS', init=False)
        self.LIMB_CONTACT.add_block(name='on', data=np.zeros(self.num_contact_sensors))

        self.BODY_FORCE = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='BODY_FORCE', init=False)
        self.BODY_FORCE.add_block(name='force', data=np.zeros(3))

        self.BODY_TORQUE = shmx.SHMEMSEG(robot_name=self.robot_name, seg_name='BODY_TORQUE', init=False)
        self.BODY_TORQUE.add_block(name='torque', data=np.zeros(3))

        try:
            self.WORLD_PARAMETER.connect_segment()
            self.MODEL_PARAMETER.connect_segment()
            self.JOINT_STATE.connect_segment()
            self.JOINT_TORQUE_COMMAND.connect_segment()
            self.POSITION_PID_GAIN.connect_segment()
            self.JOINT_POSITION_COMMAND.connect_segment()
            self.JOINT_LIMIT.connect_segment()
            self.TORQUE_LIMIT.connect_segment()
            self.BODY_POSE.connect_segment()
            self.IMU_STATE.connect_segment()
            self.LIMB_CONTACT.connect_segment()
            self.BODY_FORCE.connect_segment()
            self.BODY_TORQUE.connect_segment()
        except posix_ipc.ExistentialError:
            self.WORLD_PARAMETER.initialize        = True
            self.MODEL_PARAMETER.initialize        = True
            self.JOINT_STATE.initialize            = True
            self.JOINT_TORQUE_COMMAND.initialize   = True
            self.POSITION_PID_GAIN.initialize      = True
            self.JOINT_POSITION_COMMAND.initialize = True
            self.JOINT_LIMIT.initialize            = True
            self.TORQUE_LIMIT.initialize           = True
            self.BODY_POSE.initialize              = True
            self.IMU_STATE.initialize              = True
            self.LIMB_CONTACT.initialize           = True
            self.BODY_FORCE.initialize             = True
            self.BODY_TORQUE.initialize            = True

            self.WORLD_PARAMETER.connect_segment()
            self.MODEL_PARAMETER.connect_segment()
            self.JOINT_STATE.connect_segment()
            self.JOINT_TORQUE_COMMAND.connect_segment()
            self.POSITION_PID_GAIN.connect_segment()
            self.JOINT_POSITION_COMMAND.connect_segment()
            self.JOINT_LIMIT.connect_segment()
            self.TORQUE_LIMIT.connect_segment()
            self.BODY_POSE.connect_segment()
            self.IMU_STATE.connect_segment()
            self.LIMB_CONTACT.connect_segment()
            self.BODY_FORCE.connect_segment()
            self.BODY_TORQUE.connect_segment()

    def initialize_clients(self):
        connected = False
        timeouts = 0
        while not connected and timeouts < self.max_timeouts:
            try:
                self.world_address = self.dir + self.world_name
                self.world_socket  = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.world_socket.connect(self.world_address)

                self.model_address = self.dir + self.robot_name
                self.model_socket  = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.model_socket.connect(self.model_address)

                connected = True
            except socket.error:
                timeouts += 1
                time.sleep(1)
                print("GAZEBO NOT RUNNING! Timeouts remaining: ", self.max_timeouts - timeouts)

        if not connected:
            raise Exception("Please start Gazebo before running again!")

    def get_current_position(self):
        return self.JOINT_STATE.get()['position']

    def get_current_velocity(self):
        return self.JOINT_STATE.get()['velocity']

    def get_current_force(self):
        return self.JOINT_STATE.get()['force']

    def get_current_time(self):
        return self.JOINT_STATE.get()['time']

    def get_body_position(self):
        return self.BODY_POSE.get()['position']

    def get_body_quaternion(self):
        return self.BODY_POSE.get()['quaternion']

    def get_body_rot_mat(self):
        return MF.quat2rotm(self.get_body_quaternion())

    def get_body_euler_angles(self):
        return self.BODY_POSE.get()['euler_angles']

    def get_body_velocity(self):
        return self.BODY_POSE.get()['velocity']

    def get_imu_acceleration(self):
        return self.IMU_STATE.get()['accel']

    def get_imu_angular_rate(self):
        return self.IMU_STATE.get()['ang_rate']

    def get_foot_contacts(self):
        return self.LIMB_CONTACT.get()['on']

    def set_command_torque(self, force):
        data = {'data': np.array(force)}
        self.JOINT_TORQUE_COMMAND.set(data)

    def set_command_position(self, position):
        data = {'data': np.array(position)}
        self.JOINT_POSITION_COMMAND.set(data)

    def pause_physics(self):
        self.world_socket.send(b"pause_physics")
        self.world_socket.recv(1024)

    def unpause_physics(self):
        self.world_socket.send(b"unpause_physics")
        self.world_socket.recv(1024)

    def step_simulation(self):
        self.world_socket.send(b"step_simulation")
        self.world_socket.recv(1024)

    def reset_simulation(self, initial_pose=None):
        if initial_pose:
            self.set_command_position(initial_pose)
        else:
            self.set_command_position(np.zeros(self.num_joints))
        self.world_socket.send(b"reset_simulation")
        self.world_socket.recv(1024)

    def set_real_time_update_rate(self, rate):
        data = self.WORLD_PARAMETER.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(WorldParameters)).contents
        stru.real_time_update_rate = rate
        self.world_socket.send(b"update_world_parameters")
        self.world_socket.recv(1024)

    def set_step_size(self, step_size):
        data = self.WORLD_PARAMETER.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(WorldParameters)).contents
        stru.step_size = step_size
        self.world_socket.send(b"update_world_parameters")
        self.world_socket.recv(1024)

    def set_all_position_pid_gains(self, p_gains, i_gains, d_gains):
        data = {'data': np.zeros(3 * self.num_joints)}
        data['data'][np.arange(0, 3 * self.num_joints, 3)] = np.array(p_gains)
        data['data'][np.arange(1, 3 * self.num_joints, 3)] = np.array(i_gains)
        data['data'][np.arange(2, 3 * self.num_joints, 3)] = np.array(d_gains)
        self.POSITION_PID_GAIN.set(data)
        self.model_socket.send(b"set_position_pid_gains")
        self.model_socket.recv(1024)

    def set_joint_position_pid_gains(self, joint_idx, p_gain, i_gain, d_gain):
        data = self.POSITION_PID_GAIN.get()
        data['data'][3 * joint_idx] = p_gain
        data['data'][3 * joint_idx + 1] = i_gain
        data['data'][3 * joint_idx + 2] = d_gain
        self.POSITION_PID_GAIN.set(data)
        self.model_socket.send(b"set_position_pid_gains")
        self.model_socket.recv(1024)

    def set_operating_mode(self, mode):
        data = self.MODEL_PARAMETER.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(ModelParameters)).contents
        stru.operating_mode = int(mode)
        self.model_socket.send(b"update_model_parameters")
        self.model_socket.recv(1024)

    def set_joint_limits(self, lower_limits, upper_limits):
        data = {'data': np.zeros(2 * self.num_joints)}
        data['data'][np.arange(0, 2 * self.num_joints, 2)] = np.array(lower_limits)
        data['data'][np.arange(1, 2 * self.num_joints, 2)] = np.array(upper_limits)
        self.JOINT_LIMIT.set(data)
        self.model_socket.send(b"set_joint_limits")
        self.model_socket.recv(1024)

    def set_torque_limits(self, torque_limits):
        data = {'data': np.array(torque_limits)}
        self.TORQUE_LIMIT.set(data)
        self.model_socket.send(b"set_effort_limits")
        self.model_socket.recv(1024)

    def set_body_force(self, force):
        data = {'force': force}
        self.BODY_FORCE.set(data)
        self.model_socket.send(b"set_body_force")
        self.model_socket.recv(1024)

    def set_body_torque(self, torque):
        data = {'torque': torque}
        self.BODY_FORCE.set(data)
        self.model_socket.send(b"set_body_torque")
        self.model_socket.recv(1024)
