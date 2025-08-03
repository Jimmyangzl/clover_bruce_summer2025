#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Pre-generate the shared memory segments before using them in the rest of the scripts
'''

import numpy as np
from Library.SHARED_MEMORY import Manager as shmx


# Create Shared Memory Segments
# Thread State
THREAD_STATE = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='THREAD_STATE', init=False)
THREAD_STATE.add_block(name='simulation', data=np.zeros(1))
THREAD_STATE.add_block(name='bear',       data=np.zeros(1))
THREAD_STATE.add_block(name='dxl',        data=np.zeros(1))
THREAD_STATE.add_block(name='sense',      data=np.zeros(1))
THREAD_STATE.add_block(name='estimation', data=np.zeros(1))
THREAD_STATE.add_block(name='low_level',  data=np.zeros(1))
THREAD_STATE.add_block(name='high_level', data=np.zeros(1))
THREAD_STATE.add_block(name='top_level',  data=np.zeros(1))

# Simulator State
SIMULATOR_STATE = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='SIMULATOR_STATE', init=False)
SIMULATOR_STATE.add_block(name='time_stamp', data=np.zeros(1))

# Sense State
SENSE_STATE = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='SENSE_STATE', init=False)
SENSE_STATE.add_block(name='time_stamp',       data=np.zeros(1))
SENSE_STATE.add_block(name='imu_acceleration', data=np.zeros(3))
SENSE_STATE.add_block(name='imu_ang_rate',     data=np.zeros(3))
SENSE_STATE.add_block(name='foot_contacts',    data=np.zeros(4))

# Gamepad State
GAMEPAD_STATE = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='GAMEPAD_STATE', init=False)
GAMEPAD_STATE.add_block(name='U',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='D',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='L',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='R',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='A',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='B',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='X',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='Y',   data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LZ',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LS',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LS2', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LSP', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LSM', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RZ',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RS',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RS2', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RSP', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RSM', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='ST',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='BK',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='ALT', data=np.zeros(1))
GAMEPAD_STATE.add_block(name='FN',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LX',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='LY',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RX',  data=np.zeros(1))
GAMEPAD_STATE.add_block(name='RY',  data=np.zeros(1))

# Leg State
LEG_STATE = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='LEG_STATE', init=False)
LEG_STATE.add_block(name='time_stamp',       data=np.zeros(1))
LEG_STATE.add_block(name='joint_positions',  data=np.zeros(10))
LEG_STATE.add_block(name='joint_velocities', data=np.zeros(10))
LEG_STATE.add_block(name='joint_torques',    data=np.zeros(10))
LEG_STATE.add_block(name='temperature',      data=np.zeros(1))
LEG_STATE.add_block(name='voltage',          data=np.zeros(1))

# Leg Command
LEG_COMMAND = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='LEG_COMMAND', init=False)
LEG_COMMAND.add_block(name='time_stamp',      data=np.zeros(1))
LEG_COMMAND.add_block(name='goal_torques',    data=np.zeros(10))
LEG_COMMAND.add_block(name='goal_positions',  data=np.zeros(10))
LEG_COMMAND.add_block(name='goal_velocities', data=np.zeros(10))
LEG_COMMAND.add_block(name='BEAR_mode',       data=np.zeros(1))
LEG_COMMAND.add_block(name='BEAR_enable',     data=np.zeros(1))
LEG_COMMAND.add_block(name='damping',         data=np.zeros(1))

# Arm State
ARM_STATE = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='ARM_STATE', init=False)
ARM_STATE.add_block(name='time_stamp',       data=np.zeros(1))
ARM_STATE.add_block(name='joint_positions',  data=np.zeros(6))
ARM_STATE.add_block(name='joint_velocities', data=np.zeros(6))

# Arm Command
ARM_COMMAND = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='ARM_COMMAND', init=False)
ARM_COMMAND.add_block(name='time_stamp',      data=np.zeros(1))
ARM_COMMAND.add_block(name='goal_positions',  data=np.zeros(6))
ARM_COMMAND.add_block(name='goal_velocities', data=np.zeros(6))
ARM_COMMAND.add_block(name='DXL_mode',        data=np.zeros(1))
ARM_COMMAND.add_block(name='DXL_enable',      data=np.zeros(1))

# Estimator State
ESTIMATOR_STATE = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='ESTIMATOR_STATE', init=False)
ESTIMATOR_STATE.add_block(name='time_stamp',        data=np.zeros(1))
ESTIMATOR_STATE.add_block(name='body_position',     data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='body_velocity',     data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='body_acceleration', data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='body_rot_matrix',   data=np.eye(3))
ESTIMATOR_STATE.add_block(name='body_euler_ang',    data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='body_yaw_ang',      data=np.zeros(1))
ESTIMATOR_STATE.add_block(name='body_ang_rate',     data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='com_position',      data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='com_velocity',      data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='ang_momentum',      data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='H_matrix',          data=np.zeros((16, 16)))
ESTIMATOR_STATE.add_block(name='CG_vector',         data=np.zeros(16))
ESTIMATOR_STATE.add_block(name='AG_matrix',         data=np.zeros((6, 16)))
ESTIMATOR_STATE.add_block(name='dAGdq_vector',      data=np.zeros(6))
ESTIMATOR_STATE.add_block(name='foot_contacts',     data=np.zeros(4))

ESTIMATOR_STATE.add_block(name='right_foot_rot_matrix', data=np.zeros((3, 3)))
ESTIMATOR_STATE.add_block(name='right_foot_ang_rate',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_foot_Jw',         data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='right_foot_dJwdq',      data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_foot_position',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_foot_velocity',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_toe_position',    data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_toe_velocity',    data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_toe_Jv',          data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='right_toe_dJvdq',       data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_heel_position',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_heel_velocity',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_heel_Jv',         data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='right_heel_dJvdq',      data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_ankle_position',  data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_ankle_velocity',  data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='right_ankle_Jv',        data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='right_ankle_dJvdq',     data=np.zeros(3))

ESTIMATOR_STATE.add_block(name='left_foot_rot_matrix', data=np.zeros((3, 3)))
ESTIMATOR_STATE.add_block(name='left_foot_ang_rate',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_foot_Jw',         data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='left_foot_dJwdq',      data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_foot_position',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_foot_velocity',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_toe_position',    data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_toe_velocity',    data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_toe_Jv',          data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='left_toe_dJvdq',       data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_heel_position',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_heel_velocity',   data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_heel_Jv',         data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='left_heel_dJvdq',      data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_ankle_position',  data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_ankle_velocity',  data=np.zeros(3))
ESTIMATOR_STATE.add_block(name='left_ankle_Jv',        data=np.zeros((3, 16)))
ESTIMATOR_STATE.add_block(name='left_ankle_dJvdq',     data=np.zeros(3))

# Estimator Command
ESTIMATOR_COMMAND = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='ESTIMATOR_COMMAND', init=False)
ESTIMATOR_COMMAND.add_block(name='restart', data=np.zeros(1))

# Planner Command
PLANNER_COMMAND = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='PLANNER_COMMAND', init=False)
PLANNER_COMMAND.add_block(name='time_stamp',      data=np.zeros(1))
PLANNER_COMMAND.add_block(name='mode',            data=np.zeros(1))
PLANNER_COMMAND.add_block(name='phase',           data=np.zeros(1))
PLANNER_COMMAND.add_block(name='body_position',   data=np.zeros(3))
PLANNER_COMMAND.add_block(name='body_velocity',   data=np.zeros(3))
PLANNER_COMMAND.add_block(name='body_rot_matrix', data=np.eye(3))
PLANNER_COMMAND.add_block(name='body_ang_rate',   data=np.zeros(3))
PLANNER_COMMAND.add_block(name='com_position',    data=np.zeros(3))
PLANNER_COMMAND.add_block(name='com_velocity',    data=np.zeros(3))

PLANNER_COMMAND.add_block(name='right_foot_phase',      data=np.zeros(1))
PLANNER_COMMAND.add_block(name='right_foot_position',   data=np.zeros(3))
PLANNER_COMMAND.add_block(name='right_foot_velocity',   data=np.zeros(3))
PLANNER_COMMAND.add_block(name='right_foot_rot_matrix', data=np.zeros((3, 3)))
PLANNER_COMMAND.add_block(name='right_foot_ang_rate',   data=np.zeros(3))

PLANNER_COMMAND.add_block(name='left_foot_phase',       data=np.zeros(1))
PLANNER_COMMAND.add_block(name='left_foot_position',    data=np.zeros(3))
PLANNER_COMMAND.add_block(name='left_foot_velocity',    data=np.zeros(3))
PLANNER_COMMAND.add_block(name='left_foot_rot_matrix',  data=np.zeros((3, 3)))
PLANNER_COMMAND.add_block(name='left_foot_ang_rate',    data=np.zeros(3))

# User Command
USER_COMMAND = shmx.SHMEMSEG(robot_name='BRUCE', seg_name='USER_COMMAND', init=False)
USER_COMMAND.add_block(name='time_stamp',                  data=np.zeros(1))
USER_COMMAND.add_block(name='mode',                        data=np.zeros(1))
USER_COMMAND.add_block(name='com_xy_velocity',             data=np.zeros(2))
USER_COMMAND.add_block(name='yaw_rate',                    data=np.zeros(1))
USER_COMMAND.add_block(name='com_position_change_scaled',  data=np.zeros(3))
USER_COMMAND.add_block(name='body_euler_angle_change',     data=np.zeros(3))
USER_COMMAND.add_block(name='right_foot_yaw_angle_change', data=np.zeros(1))
USER_COMMAND.add_block(name='left_foot_yaw_angle_change',  data=np.zeros(1))
USER_COMMAND.add_block(name='foot_clearance',              data=np.zeros(1))
USER_COMMAND.add_block(name='cooling_speed',               data=np.zeros(1))
USER_COMMAND.add_block(name='dcm_offset_compensation',     data=np.zeros(2))
USER_COMMAND.add_block(name='com_offset_compensation',     data=np.zeros(1))


def init():
    """Init if main"""
    THREAD_STATE.initialize      = True
    SIMULATOR_STATE.initialize   = True
    SENSE_STATE.initialize       = True
    GAMEPAD_STATE.initialize     = True
    LEG_STATE.initialize         = True
    LEG_COMMAND.initialize       = True
    ARM_STATE.initialize         = True
    ARM_COMMAND.initialize       = True
    ESTIMATOR_STATE.initialize   = True
    ESTIMATOR_COMMAND.initialize = True
    PLANNER_COMMAND.initialize   = True
    USER_COMMAND.initialize      = True


def connect():
    """Connect and create segment"""
    THREAD_STATE.connect_segment()
    SIMULATOR_STATE.connect_segment()
    SENSE_STATE.connect_segment()
    GAMEPAD_STATE.connect_segment()
    LEG_STATE.connect_segment()
    LEG_COMMAND.connect_segment()
    ARM_STATE.connect_segment()
    ARM_COMMAND.connect_segment()
    ESTIMATOR_STATE.connect_segment()
    ESTIMATOR_COMMAND.connect_segment()
    PLANNER_COMMAND.connect_segment()
    USER_COMMAND.connect_segment()


if __name__ == '__main__':
    init()
    connect()
else:
    connect()
