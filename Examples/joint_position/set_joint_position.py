#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Set joint positions (using joint PD control)
'''

import time
import Settings.BRUCE_data as RDS
from Settings.BRUCE_macros import *


def set_joint_positions(npt, dt,
                        arm_move=False, arm_goal_positions=np.zeros(6),
                        leg_move=False, leg_goal_positions=np.zeros(10)):

    if arm_move:
        Bruce.update_arm_status()

        traj_jnt_shoulder_pitch_r = np.linspace(Bruce.joint[SHOULDER_PITCH_R]['q'], arm_goal_positions[0], npt, endpoint=True)
        traj_jnt_shoulder_roll_r  = np.linspace(Bruce.joint[SHOULDER_ROLL_R]['q'],  arm_goal_positions[1], npt, endpoint=True)
        traj_jnt_elbow_yaw_r      = np.linspace(Bruce.joint[ELBOW_YAW_R]['q'],      arm_goal_positions[2], npt, endpoint=True)

        traj_jnt_shoulder_pitch_l = np.linspace(Bruce.joint[SHOULDER_PITCH_L]['q'], arm_goal_positions[3], npt, endpoint=True)
        traj_jnt_shoulder_roll_l  = np.linspace(Bruce.joint[SHOULDER_ROLL_L]['q'],  arm_goal_positions[4], npt, endpoint=True)
        traj_jnt_elbow_yaw_l      = np.linspace(Bruce.joint[ELBOW_YAW_L]['q'],      arm_goal_positions[5], npt, endpoint=True)

    if leg_move:
        Bruce.update_leg_status()

        traj_jnt_hip_yaw_r     = np.linspace(Bruce.joint[HIP_YAW_R]['q'],     leg_goal_positions[0], npt, endpoint=True)
        traj_jnt_hip_roll_r    = np.linspace(Bruce.joint[HIP_ROLL_R]['q'],    leg_goal_positions[1], npt, endpoint=True)
        traj_jnt_hip_pitch_r   = np.linspace(Bruce.joint[HIP_PITCH_R]['q'],   leg_goal_positions[2], npt, endpoint=True)
        traj_jnt_knee_pitch_r  = np.linspace(Bruce.joint[KNEE_PITCH_R]['q'],  leg_goal_positions[3], npt, endpoint=True)
        traj_jnt_ankle_pitch_r = np.linspace(Bruce.joint[ANKLE_PITCH_R]['q'], leg_goal_positions[4], npt, endpoint=True)

        traj_jnt_hip_yaw_l     = np.linspace(Bruce.joint[HIP_YAW_L]['q'],     leg_goal_positions[5], npt, endpoint=True)
        traj_jnt_hip_roll_l    = np.linspace(Bruce.joint[HIP_ROLL_L]['q'],    leg_goal_positions[6], npt, endpoint=True)
        traj_jnt_hip_pitch_l   = np.linspace(Bruce.joint[HIP_PITCH_L]['q'],   leg_goal_positions[7], npt, endpoint=True)
        traj_jnt_knee_pitch_l  = np.linspace(Bruce.joint[KNEE_PITCH_L]['q'],  leg_goal_positions[8], npt, endpoint=True)
        traj_jnt_ankle_pitch_l = np.linspace(Bruce.joint[ANKLE_PITCH_L]['q'], leg_goal_positions[9], npt, endpoint=True)

    for tdx in range(npt):
        if arm_move:
            Bruce.joint[SHOULDER_PITCH_R]['q_goal'] = traj_jnt_shoulder_pitch_r[tdx]
            Bruce.joint[SHOULDER_ROLL_R]['q_goal']  = traj_jnt_shoulder_roll_r[tdx]
            Bruce.joint[ELBOW_YAW_R]['q_goal']      = traj_jnt_elbow_yaw_r[tdx]

            Bruce.joint[SHOULDER_PITCH_L]['q_goal'] = traj_jnt_shoulder_pitch_l[tdx]
            Bruce.joint[SHOULDER_ROLL_L]['q_goal']  = traj_jnt_shoulder_roll_l[tdx]
            Bruce.joint[ELBOW_YAW_L]['q_goal']      = traj_jnt_elbow_yaw_l[tdx]

            Bruce.set_command_arm_positions()

        if leg_move:
            Bruce.joint[HIP_YAW_R]['q_goal']     = traj_jnt_hip_yaw_r[tdx]
            Bruce.joint[HIP_ROLL_R]['q_goal']    = traj_jnt_hip_roll_r[tdx]
            Bruce.joint[HIP_PITCH_R]['q_goal']   = traj_jnt_hip_pitch_r[tdx]
            Bruce.joint[KNEE_PITCH_R]['q_goal']  = traj_jnt_knee_pitch_r[tdx]
            Bruce.joint[ANKLE_PITCH_R]['q_goal'] = traj_jnt_ankle_pitch_r[tdx]

            Bruce.joint[HIP_YAW_L]['q_goal']     = traj_jnt_hip_yaw_l[tdx]
            Bruce.joint[HIP_ROLL_L]['q_goal']    = traj_jnt_hip_roll_l[tdx]
            Bruce.joint[HIP_PITCH_L]['q_goal']   = traj_jnt_hip_pitch_l[tdx]
            Bruce.joint[KNEE_PITCH_L]['q_goal']  = traj_jnt_knee_pitch_l[tdx]
            Bruce.joint[ANKLE_PITCH_L]['q_goal'] = traj_jnt_ankle_pitch_l[tdx]

            Bruce.set_command_leg_positions()

        time.sleep(dt)


if __name__ == '__main__':
    # BRUCE setup
    Bruce = RDS.BRUCE()

    # Arm goal pose (shoulder pitch, shoulder roll, elbow yaw)
    ar1, ar2, ar3 = -0.7,  1.3,  2.0  # right arm
    al1, al2, al3 =  0.7, -1.3, -2.0  # left arm

    # Leg goal pose (hip yaw, hip roll, hip pitch, knee pitch, ankle pitch)
    lr1, lr2, lr3, lr4, lr5 = -PI_2, PI_2, 0.6, -1.0, 0.3  # right leg
    ll1, ll2, ll3, ll4, ll5 = -PI_2, PI_2, 0.6, -1.0, 0.3  # left leg

    # Go to goal pose from present pose (here we divide the whole process into Ns pieces evenly in joint space)
    Ns = 100   # number of pieces
    Td = 0.01  # time delay [sec] between adjacent pieces

    # Start moving
    confirm = input('Go to goal pose? (y/n) ')
    if confirm != 'y':
        exit()
    set_joint_positions(Ns, Td,
                        arm_move=True, arm_goal_positions=np.array([ar1, ar2, ar3,
                                                                    al1, al2, al3]),
                        leg_move=True, leg_goal_positions=np.array([lr1, lr2, lr3, lr4, lr5,
                                                                    ll1, ll2, ll3, ll4, ll5]))
    
    time.sleep(1)
    Bruce.stop_robot()  # disable the actuators in the end
