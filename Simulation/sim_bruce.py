#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for communication with Gazebo
'''

import time
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
import Library.ROBOT_MODEL.BRUCE_dynamics as dyn
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
from Play.config import *
from Settings.BRUCE_macros import *
from Play.Walking.walking_macros import *
from Library.BRUCE_GYM.GAZEBO_INTERFACE import Manager as gazint


class GazeboSimulator:
    def __init__(self):
        # robot info
        self.num_legs = 2
        self.num_joints_per_leg = 5
        self.num_arms = 2
        self.num_joints_per_arms = 3
        self.num_joints = self.num_legs * self.num_joints_per_leg + self.num_arms * self.num_joints_per_arms
        self.num_contact_sensors = 4
        
        self.leg_p_gains = [265, 150,  80,  80,    30]
        self.leg_i_gains = [  0,   0,   0,   0,     0]
        self.leg_d_gains = [ 1., 2.3, 0.8, 0.8, 0.003]

        self.arm_p_gains = [ 1.6,  1.6,  1.6]
        self.arm_i_gains = [   0,    0,    0]
        self.arm_d_gains = [0.03, 0.03, 0.03]

        self.p_gains = self.leg_p_gains * 2 + self.arm_p_gains * 2  # the joint order matches the robot's sdf file
        self.i_gains = self.leg_i_gains * 2 + self.arm_i_gains * 2
        self.d_gains = self.leg_d_gains * 2 + self.arm_d_gains * 2
        
        # simulator info
        self.simulator = None
        self.simulation_frequency = 1000  # Hz
        self.simulation_modes = {'torque': 0, 'position': 2}
        self.simulation_mode = self.simulation_modes['position']
        
    def initialize_simulator(self):
        self.simulator = gazint.GazeboInterface(robot_name='bruce', num_joints=self.num_joints, num_contact_sensors=self.num_contact_sensors)
        self.simulator.set_step_size(1. / self.simulation_frequency)
        self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_all_position_pid_gains(self.p_gains, self.i_gains, self.d_gains)

        # arm pose
        ar1, ar2, ar3 = -0.7,  1.3,  2.0
        al1, al2, al3 =  0.7, -1.3, -2.0

        # leg pose
        bpr = np.array([0.04, -0.07, -0.42])  # right foot position  in body frame
        bpl = np.array([0.04, +0.07, -0.42])  # left  foot position  in body frame
        bxr = np.array([1., 0., 0.])          # right foot direction in body frame
        bxl = np.array([1., 0., 0.])          # left  foot direction in body frame
        lr1, lr2, lr3, lr4, lr5 = kin.legIK_foot(bpr, bxr, +1.)
        ll1, ll2, ll3, ll4, ll5 = kin.legIK_foot(bpl, bxl, -1.)
        initial_pose = [lr1+PI_2, lr2-PI_2, lr3, lr4, lr5,
                        ll1+PI_2, ll2-PI_2, ll3, ll4, ll5,
                        ar1, ar2, ar3,
                        al1, al2, al3]
        self.simulator.reset_simulation(initial_pose=initial_pose)
        
        print('Gazebo Initialization Completed!')
        
    def write_position(self, leg_positions, arm_positions):
        """
        Send goal positions to the simulator.
        """
        goal_position = [leg_positions[0]+PI_2, leg_positions[1]-PI_2, leg_positions[2], leg_positions[3], leg_positions[4],
                         leg_positions[5]+PI_2, leg_positions[6]-PI_2, leg_positions[7], leg_positions[8], leg_positions[9],
                         arm_positions[0], arm_positions[1], arm_positions[2],
                         arm_positions[3], arm_positions[4], arm_positions[5]]
        if self.simulation_mode != self.simulation_modes['position']:
            self.simulation_mode = self.simulation_modes['position']
            self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_command_position(goal_position)

    def write_torque(self, leg_torques, arm_torques):
        """
        Send goal torques to the simulator.
        """
        goal_torque = [leg_torques[0], leg_torques[1], leg_torques[2], leg_torques[3], leg_torques[4],
                       leg_torques[5], leg_torques[6], leg_torques[7], leg_torques[8], leg_torques[9],
                       arm_torques[0], arm_torques[1], arm_torques[2],
                       arm_torques[3], arm_torques[4], arm_torques[5]]
        if self.simulation_mode != self.simulation_modes['torque']:
            self.simulation_mode = self.simulation_modes['torque']
            self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_command_torque(goal_torque)

    def get_arm_goal_torques(self, arm_positions, arm_velocities):
        """
        Calculate arm goal torques.
        """
        arm_goal_torque = np.zeros(6)
        for i in range(6):
            arm_goal_torque[i] = self.arm_p_gains[i % self.num_joints_per_arms] * (arm_positions[i] - self.q_arm[i]) + self.arm_d_gains[i % self.num_joints_per_arms] * (arm_velocities[i] - self.dq_arm[i])
        return arm_goal_torque
        
    def update_sensor_info(self):
        """
        Get sensor info and write it to shared memory.
        """
        # get sim time
        MM.SIMULATOR_STATE.set({'time_stamp': np.array([self.simulator.get_current_time()])})

        # get joint states
        q = self.simulator.get_current_position()
        dq = self.simulator.get_current_velocity()
        tau = self.simulator.get_current_force()
        
        self.q_leg  = np.array([q[0]-PI_2, q[1]+PI_2, q[2], q[3], q[4],
                                q[5]-PI_2, q[6]+PI_2, q[7], q[8], q[9]])
        self.q_arm  = q[10:16]
        self.dq_leg = dq[0:10]
        self.dq_arm = dq[10:16]

        leg_data = {'joint_positions':  self.q_leg,
                    'joint_velocities': self.dq_leg,
                    'joint_torques':    tau[0:10]}
        arm_data = {'joint_positions':  self.q_arm,
                    'joint_velocities': self.dq_arm}
        MM.LEG_STATE.set(leg_data)
        MM.ARM_STATE.set(arm_data)
        
        # get imu states
        self.rot_mat = self.simulator.get_body_rot_mat()
        self.accel   = self.simulator.get_imu_acceleration()
        self.omega   = self.rot_mat.T @ self.simulator.get_imu_angular_rate()
        self.foot_contacts = self.simulator.get_foot_contacts()
        
        sense_data = {'imu_acceleration': self.accel,
                      'imu_ang_rate':     self.omega,
                      'foot_contacts':    self.foot_contacts}
        MM.SENSE_STATE.set(sense_data)
        
    def calculate_robot_model(self):
        """
        Calculate kinematics & dynamics and write it to shared memory.
        """
        r1, r2, r3, r4, r5 = self.q_leg[0], self.q_leg[1], self.q_leg[2], self.q_leg[3], self.q_leg[4]
        l1, l2, l3, l4, l5 = self.q_leg[5], self.q_leg[6], self.q_leg[7], self.q_leg[8], self.q_leg[9]
        dr1, dr2, dr3, dr4, dr5 = self.dq_leg[0], self.dq_leg[1], self.dq_leg[2], self.dq_leg[3], self.dq_leg[4]
        dl1, dl2, dl3, dl4, dl5 = self.dq_leg[5], self.dq_leg[6], self.dq_leg[7], self.dq_leg[8], self.dq_leg[9]

        R_wb = self.rot_mat
        w_bb = self.omega
        p_wb = self.simulator.get_body_position()
        v_wb = self.simulator.get_body_velocity()
        a_wb = R_wb @ self.accel
        v_bb = R_wb.T @ v_wb
        yaw_angle = np.arctan2(R_wb[1, 0], R_wb[0, 0])

        # compute leg forward kinematics
        p_bt_r, v_bt_r, Jv_bt_r, dJv_bt_r, \
        p_bh_r, v_bh_r, Jv_bh_r, dJv_bh_r, \
        p_ba_r, v_ba_r, Jv_ba_r, dJv_ba_r, \
        p_bf_r, v_bf_r,  R_bf_r,  Jw_bf_r, dJw_bf_r, \
        p_bt_l, v_bt_l, Jv_bt_l, dJv_bt_l, \
        p_bh_l, v_bh_l, Jv_bh_l, dJv_bh_l, \
        p_ba_l, v_ba_l, Jv_ba_l, dJv_ba_l, \
        p_bf_l, v_bf_l,  R_bf_l,  Jw_bf_l, dJw_bf_l = kin.legFK(r1, r2, r3, r4, r5,
                                                                l1, l2, l3, l4, l5,
                                                                dr1, dr2, dr3, dr4, dr5,
                                                                dl1, dl2, dl3, dl4, dl5)

        # compute robot forward kinematics
        p_wt_r, v_wt_r, Jv_wt_r, dJvdq_wt_r, \
        p_wh_r, v_wh_r, Jv_wh_r, dJvdq_wh_r, \
        p_wa_r, v_wa_r, Jv_wa_r, dJvdq_wa_r, \
        p_wf_r, v_wf_r,  \
        R_wf_r, w_ff_r, Jw_ff_r, dJwdq_ff_r, \
        p_wt_l, v_wt_l, Jv_wt_l, dJvdq_wt_l, \
        p_wh_l, v_wh_l, Jv_wh_l, dJvdq_wh_l, \
        p_wa_l, v_wa_l, Jv_wa_l, dJvdq_wa_l, \
        p_wf_l, v_wf_l,  \
        R_wf_l, w_ff_l, Jw_ff_l, dJwdq_ff_l = kin.robotFK(R_wb, p_wb, w_bb, v_bb,
                                                          p_bt_r, Jv_bt_r, dJv_bt_r,
                                                          p_bh_r, Jv_bh_r, dJv_bh_r,
                                                          p_ba_r, Jv_ba_r, dJv_ba_r, R_bf_r, Jw_bf_r, dJw_bf_r,
                                                          p_bt_l, Jv_bt_l, dJv_bt_l,
                                                          p_bh_l, Jv_bh_l, dJv_bh_l,
                                                          p_ba_l, Jv_ba_l, dJv_ba_l, R_bf_l, Jw_bf_l, dJw_bf_l,
                                                          dr1, dr2, dr3, dr4, dr5,
                                                          dl1, dl2, dl3, dl4, dl5)

        # calculate robot dynamics
        H, CG, AG, dAGdq, p_wg, v_wg, k_wg = dyn.robotID(R_wb, p_wb, w_bb, v_bb,
                                                         r1, r2, r3, r4, r5,
                                                         l1, l2, l3, l4, l5,
                                                         dr1, dr2, dr3, dr4, dr5,
                                                         dl1, dl2, dl3, dl4, dl5)

        # save as estimation data
        estimation_data = {}
        estimation_data['time_stamp']        = np.array([self.simulator.get_current_time()])
        estimation_data['body_position']     = p_wb
        estimation_data['body_velocity']     = v_wb
        estimation_data['body_acceleration'] = a_wb
        estimation_data['body_rot_matrix']   = R_wb
        estimation_data['body_ang_rate']     = w_bb
        estimation_data['body_yaw_ang']      = np.array([yaw_angle])
        estimation_data['com_position']      = p_wg
        estimation_data['com_velocity']      = v_wg
        estimation_data['ang_momentum']      = k_wg
        estimation_data['H_matrix']          = H
        estimation_data['CG_vector']         = CG
        estimation_data['AG_matrix']         = AG
        estimation_data['dAGdq_vector']      = dAGdq
        estimation_data['foot_contacts']     = self.foot_contacts

        estimation_data['right_foot_rot_matrix'] = R_wf_r
        estimation_data['right_foot_ang_rate']   = w_ff_r
        estimation_data['right_foot_Jw']         = Jw_ff_r
        estimation_data['right_foot_dJwdq']      = dJwdq_ff_r
        estimation_data['right_foot_position']   = p_wf_r
        estimation_data['right_foot_velocity']   = v_wf_r
        estimation_data['right_toe_position']    = p_wt_r
        estimation_data['right_toe_velocity']    = v_wt_r
        estimation_data['right_toe_Jv']          = Jv_wt_r
        estimation_data['right_toe_dJvdq']       = dJvdq_wt_r
        estimation_data['right_heel_position']   = p_wh_r
        estimation_data['right_heel_velocity']   = v_wh_r
        estimation_data['right_heel_Jv']         = Jv_wh_r
        estimation_data['right_heel_dJvdq']      = dJvdq_wh_r
        estimation_data['right_ankle_position']  = p_wa_r
        estimation_data['right_ankle_velocity']  = v_wa_r
        estimation_data['right_ankle_Jv']        = Jv_wa_r
        estimation_data['right_ankle_dJvdq']     = dJvdq_wa_r

        estimation_data['left_foot_rot_matrix']  = R_wf_l
        estimation_data['left_foot_ang_rate']    = w_ff_l
        estimation_data['left_foot_Jw']          = Jw_ff_l
        estimation_data['left_foot_dJwdq']       = dJwdq_ff_l
        estimation_data['left_foot_position']    = p_wf_l
        estimation_data['left_foot_velocity']    = v_wf_l
        estimation_data['left_toe_position']     = p_wt_l
        estimation_data['left_toe_velocity']     = v_wt_l
        estimation_data['left_toe_Jv']           = Jv_wt_l
        estimation_data['left_toe_dJvdq']        = dJvdq_wt_l
        estimation_data['left_heel_position']    = p_wh_l
        estimation_data['left_heel_velocity']    = v_wh_l
        estimation_data['left_heel_Jv']          = Jv_wh_l
        estimation_data['left_heel_dJvdq']       = dJvdq_wh_l
        estimation_data['left_ankle_position']   = p_wa_l
        estimation_data['left_ankle_velocity']   = v_wa_l
        estimation_data['left_ankle_Jv']         = Jv_wa_l
        estimation_data['left_ankle_dJvdq']      = dJvdq_wa_l

        MM.ESTIMATOR_STATE.set(estimation_data)


def main_loop():
    # When restart this thread, reset the shared memory (so the robot is in idle)
    MM.init()
    MM.connect()

    # BRUCE SETUP
    Bruce = RDS.BRUCE()

    gs = GazeboSimulator()
    gs.initialize_simulator()

    MM.THREAD_STATE.set({'simulation': np.array([1.0])}, opt='only')  # thread is running

    while True:
        if Bruce.thread_error():
            Bruce.stop_threading()

        gs.update_sensor_info()
        if not ESTIMATION:
            gs.calculate_robot_model()

        leg_command = MM.LEG_COMMAND.get()
        arm_command = MM.ARM_COMMAND.get()

        if leg_command['BEAR_enable'][0] == 1.0:
            if leg_command['BEAR_mode'][0] == 2:  # position control
                gs.write_position(leg_command['goal_positions'], arm_command['goal_positions'])
            elif leg_command['BEAR_mode'][0] == 0 or 3:  # torque control
                gs.write_torque(leg_command['goal_torques'], gs.get_arm_goal_torques(arm_command['goal_positions'], arm_command['goal_velocities']))

        gs.simulator.step_simulation()
        time.sleep(0.000)  # delay if needed


if __name__ == "__main__":
    try:
        MM.THREAD_STATE.set({'simulation': np.array([1.0])}, opt='only')  # thread is running
        main_loop()
    except:
        time.sleep(0.1)
        MM.THREAD_STATE.set({'simulation': np.array([2.0])}, opt='only')  # thread is stopped
