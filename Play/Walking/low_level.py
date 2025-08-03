#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
QP-Based Weighted Whole-Body Control
'''

import osqp
import Startups.memory_manager as MM
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
from scipy import linalg
from scipy import sparse
from Play.config import *
from Play.initialize import *
from termcolor import colored
from Settings.BRUCE_macros import *
from Play.Walking.walking_macros import *


def check_simulation_or_estimation_thread(robot):
    error = True
    while error:
        thread_data = MM.THREAD_STATE.get()
        if thread_data['simulation'][0] == 1.0 or thread_data['estimation'][0] == 1.0:
            error = False

            # move shoulder twice if good
            num = 2
            for _ in range(num):
                set_joint_positions(robot, 30, 0.01, arm_move=True, arm_goal_positions=np.array([-0.7, 1.1, 2.0, 0.7, -1.1, -2.0]))
                set_joint_positions(robot, 30, 0.01, arm_move=True, arm_goal_positions=np.array([-0.7, 1.3, 2.0, 0.7, -1.3, -2.0]))
        time.sleep(0.1)


def cost_update(P0, q0,
                Jac0, dJac0dq, w0,      Kd0,          kt,
                Jac1, dJac1dq, W1, Kp1, Kd1, pg, pgd, vg, vgd,
                               w2, Kp2, Kd2, Rb, Rbd, wb, wbd,
                Jac3, dJac3dq, w3,
                Jac4, dJac4dq, w4, Kp4, Kd4, pr, prd, vr, vrd,
                Jac5, dJac5dq, w5, Kp5, Kd5, Rr, Rrd, wr, wrd,
                Jac6, dJac6dq, w6,
                Jac7, dJac7dq, w7, Kp7, Kd7, pl, pld, vl, vld,
                Jac8, dJac8dq, w8, Kp8, Kd8, Rl, Rld, wl, wld):

    P = np.copy(P0)
    q = np.copy(q0)

    # Task 0 - Angular Momentum
    ac0            = -Kd0 @ kt
    JacTW0         = Jac0.T * w0
    P[0:16, 0:16] += JacTW0 @ Jac0
    q[0:16]       += JacTW0 @ (dJac0dq - ac0)

    # Task 1 - Linear Momentum
    ac1            = (Kp1 @ (pgd - pg) + Kd1 @ (vgd - vg)) * MASS_TOT
    JacTW1         = Jac1.T @ W1
    P[0:16, 0:16] += JacTW1 @ Jac1
    q[0:16]       += JacTW1 @ (dJac1dq - ac1)

    # Task 2 - Base Orientation
    ac2          = Kp2 @ MF.logvee(Rb.T @ Rbd) + Kd2 @ (wbd - wb)
    P[0:3, 0:3] += w2 * np.eye(3)
    q[0:3]      -= w2 * ac2

    # Task 3 - Right Stance Contact
    JacTW3         = Jac3.T * w3
    P[0:16, 0:16] += JacTW3 @ Jac3
    q[0:16]       += JacTW3 @ dJac3dq

    # Task 4 - Right Swing Position
    ac4            = Kp4 @ (prd - pr) + Kd4 @ (vrd - vr)
    JacTW4         = Jac4.T * w4
    P[0:16, 0:16] += JacTW4 @ Jac4
    q[0:16]       += JacTW4 @ (dJac4dq - ac4)

    # Task 5 - Right Swing Orientation
    ac5            = Kp5 @ MF.logvee(Rr.T @ Rrd) + Kd5 @ (wrd - wr)
    JacTW5         = Jac5.T * w5
    P[0:16, 0:16] += JacTW5 @ Jac5
    q[0:16]       += JacTW5 @ (dJac5dq - ac5[1:3])

    # Task 6 - Left Stance Contact
    JacTW6         = Jac6.T * w6
    P[0:16, 0:16] += JacTW6 @ Jac6
    q[0:16]       += JacTW6 @ dJac6dq

    # Task 7 - Left Swing Position
    ac7            = Kp7 @ (pld - pl) + Kd7 @ (vld - vl)
    JacTW7         = Jac7.T * w7
    P[0:16, 0:16] += JacTW7 @ Jac7
    q[0:16]       += JacTW7 @ (dJac7dq - ac7)

    # Task 8 - Left Swing Orientation
    ac8            = Kp8 @ MF.logvee(Rl.T @ Rld) + Kd8 @ (wld - wl)
    JacTW8         = Jac8.T * w8
    P[0:16, 0:16] += JacTW8 @ Jac8
    q[0:16]       += JacTW8 @ (dJac8dq - ac8[1:3])

    return P, q


def constraint_update(A0, l0, u0,
                      Hb, CGb,
                      Jrtb, Jrhb, Jltb, Jlhb,
                      fz_max_r, fz_min_r,
                      fz_max_l, fz_min_l):

    A = np.copy(A0)
    l = np.copy(l0)
    u = np.copy(u0)

    # Constraint 1 - Dynamics
    A[0:6, 0:16] = Hb

    l[0:6] = -CGb
    u[0:6] = l[0:6]

    A[0:6, 16:19] = -Jrtb.T
    A[0:6, 19:22] = -Jrhb.T
    A[0:6, 22:25] = -Jltb.T
    A[0:6, 25:28] = -Jlhb.T

    # Constraint 2 - Contact Force
    u[10], l[10] = fz_max_r, fz_min_r
    u[15], l[15] = fz_max_r, fz_min_r
    u[20], l[20] = fz_max_l, fz_min_l
    u[25], l[25] = fz_max_l, fz_min_l

    return A, l, u


def get_tau(Hj, CGj,
            Jrtj, Jrhj, Jltj, Jlhj,
            Frt, Frh, Flt, Flh,
            ddq):
    tau = Hj @ ddq + CGj - Jrtj.T @ Frt - Jrhj.T @ Frh - Jltj.T @ Flt - Jlhj.T @ Flh
    return tau


def main_loop():
    # BRUCE SETUP
    Bruce = RDS.BRUCE()

    # Check if estimation is running
    check_simulation_or_estimation_thread(Bruce)

    # CONTROL FREQUENCY
    loop_freq     = 500  # run at 500 Hz
    loop_duration = 1. / loop_freq

    # FEEDBACK GAINS
    # swing foot
    if SIMULATION:
        Kp_p = np.array([100., 100., 100.]) * 10.
        Kd_p = np.array([ 10.,  10.,  10.]) * 10.
    else:
        Kp_p = np.array([500., 500., 100.])
        Kd_p = np.array([ 10.,  10.,  10.])
    Kp_R = np.array([0., 100., 300.])
    Kd_R = np.array([0.,  10.,  30.])

    # [ang mom, lin mom, body rot, sw pos r, sw rot r, sw pos l, sw rot l]
    # balancing (b), right stance (r), left stance (l), single stance (s)
    Kp_all_b = np.array([0., 0., 0.,  50, 50, 150,  1000, 1000, 200,        0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,      0])
    Kp_all_r = np.array([0., 0., 0.,   0,  0, 150,  1000, 1000, 200,        0,       0,       0,       0,       0,       0, Kp_p[0], Kp_p[1], Kp_p[2], Kp_R[0], Kp_R[1], Kp_R[2]])
    Kp_all_l = np.array([0., 0., 0.,   0,  0, 150,  1000, 1000, 200,  Kp_p[0], Kp_p[1], Kp_p[2], Kp_R[0], Kp_R[1], Kp_R[2],       0,       0,       0,       0,       0,      0])

    Kd_all_b = np.array([10, 10, 1.,  10, 10,  15,   100,  100,  20,        0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,      0])
    Kd_all_r = np.array([10, 10, 1.,   5,  5,  15,   100,  100,  20,        0,       0,       0,       0,       0,       0, Kd_p[0], Kd_p[1], Kd_p[2], Kd_R[0], Kd_R[1], Kd_R[2]])
    Kd_all_l = np.array([10, 10, 1.,   5,  5,  15,   100,  100,  20,  Kd_p[0], Kd_p[1], Kd_p[2], Kd_R[0], Kd_R[1], Kd_R[2],       0,       0,       0,       0,       0,      0])

    Kp_all = np.vstack((Kp_all_b, Kp_all_r, Kp_all_l))
    Kd_all = np.vstack((Kd_all_b, Kd_all_r, Kd_all_l))

    # COST WEIGHTS
    # angular momentum
    wk = 1.

    # linear momentum xy
    wl_b = 50.
    wl_s = 1.

    # linear momentum z
    wz = 100.

    # body orientation
    wR_b = 10.
    wR_s = 10.

    # stance foot contact
    wc_b = np.array([ 1e3,  1e3])
    wc_r = np.array([ 1e3, 1e-4])
    wc_l = np.array([1e-4,  1e3])

    # swing foot position
    wsp_b = np.array([1e-4, 1e-4])
    wsp_r = np.array([1e-4, 100.]) if HARDWARE else np.array([1e-4, 10.])
    wsp_l = np.array([100., 1e-4]) if HARDWARE else np.array([10., 1e-4])

    # swing foot orientation
    wsR_b = np.array([1e-4, 1e-4])
    wsR_r = np.array([1e-4,   1.])
    wsR_l = np.array([  1., 1e-4])

    # [ang mom, lin mom z, body rot, contact r, sw pos r, sw rot r, contact l, sw pos l, sw rot l]
    W_all_b = np.array([wk, wl_b, wR_b, wc_b[0], wsp_b[0], wsR_b[0], wc_b[1], wsp_b[1], wsR_b[1]])
    W_all_r = np.array([wk, wl_s, wR_s, wc_r[0], wsp_r[0], wsR_r[0], wc_r[1], wsp_r[1], wsR_r[1]])
    W_all_l = np.array([wk, wl_s, wR_s, wc_l[0], wsp_l[0], wsR_l[0], wc_l[1], wsp_l[1], wsR_l[1]])
    W_all   = np.vstack((W_all_b, W_all_r, W_all_l))

    # regularization terms
    W_F   = np.eye(12) * 1e-3                                       # contact force
    W_ddq = linalg.block_diag(1e-4 * np.eye(6), 1e-4 * np.eye(10))  # joint acceleration

    # QP SETUP
    # Decision Variables
    # x = [ddq, F] -     16 + 12 = 28
    # ddq          -   6 + 2 * 5 = 16 (6 dofs for floating base and 5 dofs for each leg)
    # F            - 2 * (3 + 3) = 12 (3 dofs for each toe/heel)

    # Constraints
    # Con1 - dynamics constraint: H_b*ddq - J_b'*F = -CG_b (6 cons)
    Aeq = np.zeros((6, 28))
    beq = np.zeros(6)

    # Con2 - contact force constraint:
    # -2*mu*fz_max <= -fx-mu*fz <= 0
    # -2*mu*fz_max <=  fx-mu*fz <= 0
    # -2*mu*fz_max <= -fy-mu*fz <= 0
    # -2*mu*fz_max <=  fy-mu*fz <= 0
    #                        fz <= fz_max (2 * 2 * 5 = 20 cons)
    mu       = 0.5  # friction coefficient
    fz_max   = 100.
    fz_min   = 1.
    fz_max_all = np.array([[fz_max, fz_max],
                           [fz_max,     0.],
                           [    0., fz_max]])
    fz_min_all = np.array([[fz_min, fz_min],
                           [fz_min,     0.],
                           [    0., fz_min]])
    Aineq    = np.zeros((20, 28))
    bineq_ls = np.array([-2 * mu * fz_max, -2 * mu * fz_max, -2 * mu * fz_max, -2 * mu * fz_max,     0.])
    bineq_us = np.array([              0.,               0.,               0.,               0., fz_max])
    bineq_l  = np.kron(np.ones(4), bineq_ls)
    bineq_u  = np.kron(np.ones(4), bineq_us)
    Aineq[0:20, 16:28] = np.kron(np.eye(4), np.array([[-1.,  0., -mu],
                                                      [ 1.,  0., -mu],
                                                      [ 0., -1., -mu],
                                                      [ 0.,  1., -mu],
                                                      [ 0.,  0.,  1.]]))

    # Overall
    A0 = np.vstack((Aeq, Aineq))
    l0 = np.hstack((beq, bineq_l))
    u0 = np.hstack((beq, bineq_u))

    P0 = np.zeros((28, 28))
    q0 = np.zeros(28)

    P0[ 0:16,  0:16] = 1e-12 * np.ones((16, 16)) + W_ddq
    P0[16:28, 16:28] = W_F

    # START CONTROL
    confirm = input('Start whole body control? (y/n) ')
    if confirm != 'y':
        exit()

    Bruce.update_robot_status()
    plan_data = {'robot_state':     np.zeros(1),
                 'body_rot_matrix': MF.Rz(Bruce.yaw),
                 'body_ang_rate':   np.zeros(3),
                 'com_position':    np.copy(Bruce.p_wg),
                 'com_velocity':    np.zeros(3)}
    MM.PLANNER_COMMAND.set(plan_data)

    qp_setup = False  # is qp set up?

    tau_sol = np.zeros(10)
    ddq_sol = np.zeros(16)

    q_des   = np.zeros(10)
    dq_des  = np.zeros(10)
    ddq_des = np.zeros(10)
    tau_des = np.zeros(10)

    q_knee_max = -0.1

    danger_duration   = [0.3, 0.3]  # stop robot if in dangerous zone over 0.2 seconds, e.g., large tilt angle/lose foot contact
    danger_start_time = [0.0, 0.0]
    t0 = Bruce.get_time()
    thread_run = False
    while True:
        loop_start_time = Bruce.get_time()

        # elapsed time
        elapsed_time = loop_start_time - t0

        # robot state update
        Bruce.update_robot_status()
        Bruce.update_leg_status()

        # planner update
        Bruce.update_plan_status()
        Bs = int(Bruce.phase)

        # QP update
        PP, qp_q = cost_update(P0, q0,
                               Bruce.AG[0:3, :], Bruce.dAGdq[0:3], W_all[Bs, 0], np.diag(Kd_all[Bs, 0:3]), Bruce.k_wg,
                               Bruce.AG[3:6, :], Bruce.dAGdq[3:6], np.diag([W_all[Bs, 1], W_all[Bs, 1], wz]), np.diag(Kp_all[Bs, 3:6]), np.diag(Kd_all[Bs, 3:6]), Bruce.p_wg, Bruce.des_p_wg, Bruce.v_wg, Bruce.des_v_wg,
                               W_all[Bs, 2], np.diag(Kp_all[Bs, 6:9]), np.diag(Kd_all[Bs, 6:9]), Bruce.R_wb, Bruce.des_R_wb, Bruce.w_bb, Bruce.des_w_bb,
                               np.vstack((Bruce.Jv_wt_r, Bruce.Jv_wh_r)), np.hstack((Bruce.dJvdq_wt_r, Bruce.dJvdq_wh_r)), W_all[Bs, 3],
                               Bruce.Jv_wa_r, Bruce.dJvdq_wa_r, W_all[Bs, 4], np.diag(Kp_all[Bs,  9:12]), np.diag(Kd_all[Bs,  9:12]), Bruce.p_wa_r, Bruce.des_p_wf_r, Bruce.v_wa_r, Bruce.des_v_wf_r,
                               Bruce.Jw_ff_r[1:3, :], Bruce.dJwdq_ff_r[1:3], W_all[Bs, 5], np.diag(Kp_all[Bs, 12:15]), np.diag(Kd_all[Bs, 12:15]), Bruce.R_wf_r, Bruce.des_R_wf_r, Bruce.w_ff_r, Bruce.des_w_ff_r,
                               np.vstack((Bruce.Jv_wt_l, Bruce.Jv_wh_l)), np.hstack((Bruce.dJvdq_wt_l, Bruce.dJvdq_wh_l)), W_all[Bs, 6],
                               Bruce.Jv_wa_l, Bruce.dJvdq_wa_l, W_all[Bs, 7], np.diag(Kp_all[Bs, 15:18]), np.diag(Kd_all[Bs, 15:18]), Bruce.p_wa_l, Bruce.des_p_wf_l, Bruce.v_wa_l, Bruce.des_v_wf_l,
                               Bruce.Jw_ff_l[1:3, :], Bruce.dJwdq_ff_l[1:3], W_all[Bs, 8], np.diag(Kp_all[Bs, 18:21]), np.diag(Kd_all[Bs, 18:21]), Bruce.R_wf_l, Bruce.des_R_wf_l, Bruce.w_ff_l, Bruce.des_w_ff_l)
        AA, qp_l, qp_u = constraint_update(A0, l0, u0,
                                           Bruce.H[0:6, :], Bruce.CG[0:6],
                                           Bruce.Jv_wt_r[:, 0:6], Bruce.Jv_wh_r[:, 0:6],
                                           Bruce.Jv_wt_l[:, 0:6], Bruce.Jv_wh_l[:, 0:6],
                                           fz_max_all[Bs, 0], fz_min_all[Bs, 0],
                                           fz_max_all[Bs, 1], fz_min_all[Bs, 1])
        qp_P = sparse.csc_matrix(PP)
        qp_A = sparse.csc_matrix(AA)

        if not qp_setup:
            # QP initialize
            prob = osqp.OSQP()
            prob.setup(P=sparse.triu(qp_P, format='csc'), q=qp_q, A=qp_A, l=qp_l, u=qp_u, verbose=False, warm_start=True,
                       eps_abs=1e-3, eps_rel=1e-3, max_iter=1000, check_termination=1, adaptive_rho_interval=50, scaling=10)
            qp_setup = True
        else:
            # QP solving
            prob.update(Px=sparse.triu(qp_P).data, Ax=qp_A.data, q=qp_q, l=qp_l, u=qp_u)
            sol = prob.solve()

            if sol.info.status != 'solved':
                # QP infeasible
                print(colored('OSQP did not solve the problem!!! ' + sol.info.status + '!!! Te = ' + str(elapsed_time)[0:5] + ' s', 'red'))
            else:
                # QP solved
                ddq_sol = sol.x[0:16]
                F_sol   = sol.x[16:28]

                tau_sol = get_tau(Bruce.H[6:16, :], Bruce.CG[6:16],
                                  Bruce.Jv_wt_r[:, 6:16], Bruce.Jv_wh_r[:, 6:16], Bruce.Jv_wt_l[:, 6:16], Bruce.Jv_wh_l[:, 6:16],
                                  F_sol[0:3], F_sol[3:6], F_sol[6:9], F_sol[9:12],
                                  ddq_sol)

            for idx, joint_id in enumerate(LEG_JOINT_LIST):
                tau_des[idx] = MF.exp_filter(tau_des[idx], tau_sol[idx],   0.0)
                ddq_des[idx] = MF.exp_filter(ddq_des[idx], ddq_sol[idx+6], 0.0)

                # Decayed joint value integration
                dq_des[idx] = MF.exp_filter(dq_des[idx],                         0., 0.0) + ddq_des[idx] * loop_duration
                q_des[idx]  = MF.exp_filter( q_des[idx], Bruce.joint[joint_id]['q'], 0.0) +  dq_des[idx] * loop_duration

            # IK compensation
            if Bs == 1:
                R_bw   = Bruce.R_wb.T
                p_bf_l = R_bw @ (Bruce.des_p_wf_l - Bruce.p_wb)
                R_bf_l = R_bw @ Bruce.des_R_wf_l
                v_wf_l = Bruce.des_v_wf_l - Bruce.v_wb - Bruce.R_wb @ MF.hat(Bruce.w_bb) @ p_bf_l
                ql1, ql2, ql3, ql4, ql5 = kin.legIK_ankle(p_bf_l, R_bf_l[:, 0], -1)
                dql = np.linalg.solve(np.vstack((Bruce.Jv_wa_l[0:3, 11:15], Bruce.Jw_ff_l[2, 11:15])), np.hstack((v_wf_l, 0.)))
                q_des[5], q_des[6], q_des[7], q_des[8], q_des[9] = ql1, ql2, ql3, ql4, ql5
                dq_des[5], dq_des[6], dq_des[7], dq_des[8], dq_des[9] = dql[0], dql[1], dql[2], dql[3], 0.
            elif Bs == 2:
                R_bw   = Bruce.R_wb.T
                p_bf_r = R_bw @ (Bruce.des_p_wf_r - Bruce.p_wb)
                R_bf_r = R_bw @ Bruce.des_R_wf_r
                v_wf_r = Bruce.des_v_wf_r - Bruce.v_wb - Bruce.R_wb @ MF.hat(Bruce.w_bb) @ p_bf_r
                qr1, qr2, qr3, qr4, qr5 = kin.legIK_ankle(p_bf_r, R_bf_r[:, 0], +1)
                dqr = np.linalg.solve(np.vstack((Bruce.Jv_wa_r[0:3, 6:10], Bruce.Jw_ff_r[2, 6:10])), np.hstack((v_wf_r, 0.)))
                q_des[0], q_des[1], q_des[2], q_des[3], q_des[4] = qr1, qr2, qr3, qr4, qr5
                dq_des[0], dq_des[1], dq_des[2], dq_des[3], dq_des[4] = dqr[0], dqr[1], dqr[2], dqr[3], 0.

            if q_des[3] > q_knee_max:
                q_des[3] = q_knee_max

            if q_des[8] > q_knee_max:
                q_des[8] = q_knee_max

            for idx, joint_id in enumerate(LEG_JOINT_LIST):
                Bruce.joint[joint_id]['tau_goal'] = tau_des[idx]
                Bruce.joint[joint_id]['dq_goal']  =  dq_des[idx]
                Bruce.joint[joint_id]['q_goal']   =   q_des[idx]

            if elapsed_time > 0.5:
                # safety check, i.e., large tilt angle or lose contact -> robot is falling
                if np.arccos(Bruce.R_wb[2, 2]) > PI_4:
                    if elapsed_time - danger_start_time[0] > danger_duration[0]:
                        print(colored('Robot Large Tilt Angle! Terminate Now!', 'red'))
                        Bruce.damping_robot()
                        Bruce.stop_threading()
                else:
                    danger_start_time[0] = elapsed_time

                if Bruce.phase == 1 and not np.any(Bruce.foot_contacts[0:2]):
                    danger_duration[1] = 0.5
                    if elapsed_time - danger_start_time[1] > danger_duration[1]:
                        print(colored('Robot Losing Right Contact! Terminate Now!', 'red'))
                        Bruce.damping_robot()
                        Bruce.stop_threading()
                elif Bruce.phase == 2 and not np.any(Bruce.foot_contacts[2:4]):
                    danger_duration[1] = 0.5
                    if elapsed_time - danger_start_time[1] > danger_duration[1]:
                        print(colored('Robot Losing Left Contact! Terminate Now!', 'red'))
                        Bruce.damping_robot()
                        Bruce.stop_threading()
                else:
                    danger_start_time[1] = elapsed_time

                if elapsed_time > 1:
                    if not thread_run:
                        MM.THREAD_STATE.set({'low_level': np.array([1.0])}, opt='only')  # thread is running
                        thread_run = True

                    # check threading error
                    if Bruce.thread_error():
                        Bruce.stop_threading()

                # send command
                if SIMULATION:
                    Bruce.set_command_leg_torques()
                else:
                    Bruce.set_command_leg_values()

            # check time to ensure that the whole-body controller stays at a consistent running loop.
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
    try:
        main_loop()
    except (NameError, KeyboardInterrupt) as error:
        MM.THREAD_STATE.set({'low_level': np.array([0.0])}, opt='only')  # thread is stopped
    except Exception as error:
        print(error)
        MM.THREAD_STATE.set({'low_level': np.array([2.0])}, opt='only')  # thread in error
