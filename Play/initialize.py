#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for setting initial pose of BRUCE
'''

import time
import Settings.BRUCE_data as RDS
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
from Settings.BRUCE_macros import *


def set_joint_positions(robot, npt, dt,
                        arm_move=False, arm_goal_positions=np.zeros(6),
                        leg_move=False, leg_goal_positions=np.zeros(10)):

    if arm_move:
        robot.update_arm_status()

        traj_jnt_shoulder_pitch_r = np.linspace(robot.joint[SHOULDER_PITCH_R]['q'], arm_goal_positions[0], npt, endpoint=True)
        traj_jnt_shoulder_roll_r  = np.linspace(robot.joint[SHOULDER_ROLL_R]['q'],  arm_goal_positions[1], npt, endpoint=True)
        traj_jnt_elbow_yaw_r      = np.linspace(robot.joint[ELBOW_YAW_R]['q'],      arm_goal_positions[2], npt, endpoint=True)

        traj_jnt_shoulder_pitch_l = np.linspace(robot.joint[SHOULDER_PITCH_L]['q'], arm_goal_positions[3], npt, endpoint=True)
        traj_jnt_shoulder_roll_l  = np.linspace(robot.joint[SHOULDER_ROLL_L]['q'],  arm_goal_positions[4], npt, endpoint=True)
        traj_jnt_elbow_yaw_l      = np.linspace(robot.joint[ELBOW_YAW_L]['q'],      arm_goal_positions[5], npt, endpoint=True)

    if leg_move:
        robot.update_leg_status()

        traj_jnt_hip_yaw_r     = np.linspace(robot.joint[HIP_YAW_R]['q'],     leg_goal_positions[0], npt, endpoint=True)
        traj_jnt_hip_roll_r    = np.linspace(robot.joint[HIP_ROLL_R]['q'],    leg_goal_positions[1], npt, endpoint=True)
        traj_jnt_hip_pitch_r   = np.linspace(robot.joint[HIP_PITCH_R]['q'],   leg_goal_positions[2], npt, endpoint=True)
        traj_jnt_knee_pitch_r  = np.linspace(robot.joint[KNEE_PITCH_R]['q'],  leg_goal_positions[3], npt, endpoint=True)
        traj_jnt_ankle_pitch_r = np.linspace(robot.joint[ANKLE_PITCH_R]['q'], leg_goal_positions[4], npt, endpoint=True)

        traj_jnt_hip_yaw_l     = np.linspace(robot.joint[HIP_YAW_L]['q'],     leg_goal_positions[5], npt, endpoint=True)
        traj_jnt_hip_roll_l    = np.linspace(robot.joint[HIP_ROLL_L]['q'],    leg_goal_positions[6], npt, endpoint=True)
        traj_jnt_hip_pitch_l   = np.linspace(robot.joint[HIP_PITCH_L]['q'],   leg_goal_positions[7], npt, endpoint=True)
        traj_jnt_knee_pitch_l  = np.linspace(robot.joint[KNEE_PITCH_L]['q'],  leg_goal_positions[8], npt, endpoint=True)
        traj_jnt_ankle_pitch_l = np.linspace(robot.joint[ANKLE_PITCH_L]['q'], leg_goal_positions[9], npt, endpoint=True)

    for tdx in range(npt):
        if arm_move:
            robot.joint[SHOULDER_PITCH_R]['q_goal'] = traj_jnt_shoulder_pitch_r[tdx]
            robot.joint[SHOULDER_ROLL_R]['q_goal']  = traj_jnt_shoulder_roll_r[tdx]
            robot.joint[ELBOW_YAW_R]['q_goal']      = traj_jnt_elbow_yaw_r[tdx]

            robot.joint[SHOULDER_PITCH_L]['q_goal'] = traj_jnt_shoulder_pitch_l[tdx]
            robot.joint[SHOULDER_ROLL_L]['q_goal']  = traj_jnt_shoulder_roll_l[tdx]
            robot.joint[ELBOW_YAW_L]['q_goal']      = traj_jnt_elbow_yaw_l[tdx]

            robot.set_command_arm_positions()

        if leg_move:
            robot.joint[HIP_YAW_R]['q_goal']     = traj_jnt_hip_yaw_r[tdx]
            robot.joint[HIP_ROLL_R]['q_goal']    = traj_jnt_hip_roll_r[tdx]
            robot.joint[HIP_PITCH_R]['q_goal']   = traj_jnt_hip_pitch_r[tdx]
            robot.joint[KNEE_PITCH_R]['q_goal']  = traj_jnt_knee_pitch_r[tdx]
            robot.joint[ANKLE_PITCH_R]['q_goal'] = traj_jnt_ankle_pitch_r[tdx]

            robot.joint[HIP_YAW_L]['q_goal']     = traj_jnt_hip_yaw_l[tdx]
            robot.joint[HIP_ROLL_L]['q_goal']    = traj_jnt_hip_roll_l[tdx]
            robot.joint[HIP_PITCH_L]['q_goal']   = traj_jnt_hip_pitch_l[tdx]
            robot.joint[KNEE_PITCH_L]['q_goal']  = traj_jnt_knee_pitch_l[tdx]
            robot.joint[ANKLE_PITCH_L]['q_goal'] = traj_jnt_ankle_pitch_l[tdx]

            robot.set_command_leg_positions()

        time.sleep(dt)


if __name__ == '__main__':
    Bruce = RDS.BRUCE()

    mode = input('Go to standing (s) or nominal (n) posture? ')
    if mode == 'n':
        # arm pose
        ar1, ar2, ar3 = 0., 0., 0.
        al1, al2, al3 = 0., 0., 0.

        # leg pose
        lr1, lr2, lr3, lr4, lr5 = -PI_2, PI_2, 0., 0., 0.
        ll1, ll2, ll3, ll4, ll5 = -PI_2, PI_2, 0., 0., 0.
    else:
        # arm pose
        ar1, ar2, ar3 = -0.7,  1.3,  2.0
        al1, al2, al3 =  0.7, -1.3, -2.0

        # leg pose
        bpr = np.array([0.0740, -0.070, -0.44])  # right foot position  in body frame
        bpl = np.array([0.0740, +0.070, -0.44])  # left  foot position  in body frame
        bxr = np.array([1., 0., 0.])             # right foot direction in body frame
        bxl = np.array([1., 0., 0.])             # left  foot direction in body frame
        lr1, lr2, lr3, lr4, lr5 = kin.legIK_foot(bpr, bxr, +1.)
        ll1, ll2, ll3, ll4, ll5 = kin.legIK_foot(bpl, bxl, -1.)

        lr5 += 0.00
        ll5 += 0.00

        print(ll1, ll2, ll3, ll4, ll5)

    Ns   = 100
    freq = 100  # [Hz]
    set_joint_positions(Bruce, Ns, 1. / freq,
                        arm_move=True, arm_goal_positions=np.array([ar1, ar2, ar3,
                                                                    al1, al2, al3]),
                        leg_move=True, leg_goal_positions=np.array([lr1, lr2, lr3, lr4, lr5,
                                                                    ll1, ll2, ll3, ll4, ll5]))
    time.sleep(1)
