#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
DCM-Based Footstep Planner
'''

import time
import osqp
import numpy as np
import Util.math_function as MF
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
from scipy import linalg
from scipy import sparse
from Play.config import *
from termcolor import colored
from Play.Walking.walking_macros import *


def cost_update(q0c, W_L, W_b, w_s,
                leg, Rd, cop, eTs, dy, N,
                Lox, Woy, box, boy1, boy2):
    q = np.copy(q0c)

    q[0:2] = -W_L @ (Rd @ np.array([Lox[0], Woy[0] + dy * leg]) + cop)
    q[2:4] = -W_b @ Rd @ np.array([box[0], boy1[0] - boy2[0] * leg])

    q[-1]  = -w_s * eTs

    for idx in range(N - 1):
        idx1 = idx + 1

        id1 = 4 * idx
        id2 = id1 + 2
        id3 = id1 + 4
        id4 = id1 + 6
        id5 = id1 + 8

        Avar = W_L @ Rd @ np.array([Lox[idx1], Woy[idx1] - dy * leg * (-1.) ** idx])
        q[id1:id2] += Avar
        q[id3:id4] -= Avar
        q[id4:id5]  = -W_b @ Rd @ np.array([box[idx1], boy1[idx1] + boy2[idx1] * leg * (-1.) ** idx])

    return q


def constraint_update(A0c, l0c, u0c,
                      leg, Rd, tau, cop, b_tau,
                      omg, eTs, N,
                      L_min_1, L_min_2, L_max_1, L_max_2):
    A = np.copy(A0c)
    l = np.copy(l0c)
    u = np.copy(u0c)

    RdT = np.copy(Rd.T)
    cop = np.copy(cop)

    # Equality Constraints
    # eq1 - DCM dynamics constraint - 2 * N cons
    A[0:2, -1] = b_tau * np.exp(-omg * tau)
    l[0:2] = -cop
    u[0:2] = l[0:2]

    I2 = np.eye(2)
    for idx in range(N - 1):
        A[2*idx+2:2*idx+4, 4*idx:4*idx+8] = np.kron(np.array([1., eTs, -1., -1.]), I2)

    # Inequality Constraints
    # ineq1 - kinematic reachability: L_min <= L_k <= L_max - 2 * N cons
    i1 = 2 * N
    i2 = i1 + 2
    A[i1:i2, 0:2] = RdT
    l[i1:i2] = RdT @ cop + L_min_1 + L_min_2 * leg
    u[i1:i2] = RdT @ cop + L_max_1 + L_max_2 * leg
    for idx in range(N - 1):
        id1 = 2 * idx + i2
        id2 = id1 + 2

        id3 = 4 * idx
        id4 = id3 + 2
        id5 = id4 + 2
        id6 = id5 + 2
        A[id1:id2, id3:id4] = -RdT
        A[id1:id2, id5:id6] =  RdT

        l[id1:id2] = L_min_1 - L_min_2 * leg * (-1.) ** idx
        u[id1:id2] = L_max_1 - L_max_2 * leg * (-1.) ** idx

    return A, l, u


def get_swing_traj(t, tp, pp, vp, ap,
                   T, p1, pz0, pzf, pn, pzm,
                   Txn, Txf,
                   Tyn, Tyf,
                   Tzm, Tzf):
    tp2 = tp * tp
    tp3 = tp * tp2
    tp4 = tp * tp3
    tp5 = tp * tp4
    t_mat = np.array([[1.,  tp,    tp2,    tp3,      tp4,      tp5],
                      [0.,  1, 2 * tp, 3 * tp2,  4 * tp3,  5 * tp4],
                      [0.,  0,      2,  6 * tp, 12 * tp2, 20 * tp3]])

    # x
    Tx1 = T - Txf
    Tx2 = Tx1 * Tx1
    Tx3 = Tx1 * Tx2
    Tx4 = Tx1 * Tx3
    Tx5 = Tx1 * Tx4
    Tx_mat = np.array([[1., Tx1,     Tx2,      Tx3,      Tx4,      Tx5],
                       [0.,   1, 2 * Tx1,  3 * Tx2,  4 * Tx3,  5 * Tx4],
                       [0.,   0,       2,  6 * Tx1, 12 * Tx2, 20 * Tx3]])
    Txf_mat = np.vstack((t_mat, Tx_mat))

    if t < Txn:
        cxo = np.linalg.solve(Txf_mat, np.array([pp[0], vp[0], ap[0], pn[0], 0., 0.]))
    else:
        cxo = np.linalg.solve(Txf_mat, np.array([pp[0], vp[0], ap[0], p1[0], 0., 0.]))

    # y
    Ty1 = T - Tyf
    Ty2 = Ty1 * Ty1
    Ty3 = Ty1 * Ty2
    Ty4 = Ty1 * Ty3
    Ty5 = Ty1 * Ty4
    Ty_mat = np.array([[1., Ty1,     Ty2,      Ty3,      Ty4,      Ty5],
                       [0.,   1, 2 * Ty1,  3 * Ty2,  4 * Ty3,  5 * Ty4],
                       [0.,   0,       2,  6 * Ty1, 12 * Ty2, 20 * Ty3]])
    Tyf_mat = np.vstack((t_mat, Ty_mat))

    if t < Tyn:
        cyo = np.linalg.solve(Tyf_mat, np.array([pp[1], vp[1], ap[1], pn[1], 0., 0.]))
    else:
        cyo = np.linalg.solve(Tyf_mat, np.array([pp[1], vp[1], ap[1], p1[1], 0., 0.]))

    # z
    czo = np.zeros((2, 6))
    Tzm2 = Tzm * Tzm
    Tzm3 = Tzm * Tzm2
    Tzm4 = Tzm * Tzm3
    Tzm5 = Tzm * Tzm4
    Tzm_mat = np.array([[1.,   0,       0,        0,         0,         0],
                        [0.,   1,       0,        0,         0,         0],
                        [0.,   0,       2,        0,         0,         0],
                        [1., Tzm,    Tzm2,     Tzm3,      Tzm4,      Tzm5],
                        [0.,   1, 2 * Tzm, 3 * Tzm2,  4 * Tzm3,  5 * Tzm4],
                        [0.,   0,       2,  6 * Tzm, 12 * Tzm2, 20 * Tzm3]])

    czo[0, :] = np.linalg.solve(Tzm_mat, np.array([pz0, 0., 0., pzm, 0., 0.]))

    Tz1 = T - Tzf
    Tz2 = Tz1 * Tz1
    Tz3 = Tz1 * Tz2
    Tz4 = Tz1 * Tz3
    Tz5 = Tz1 * Tz4
    Tz_mat = np.array([[1., Tz1,     Tz2,      Tz3,      Tz4,      Tz5],
                       [0.,   1, 2 * Tz1,  3 * Tz2,  4 * Tz3,  5 * Tz4],
                       [0.,   0,       2,  6 * Tz1, 12 * Tz2, 20 * Tz3]])

    if tp < Tzm:
        Tzf_mat = np.vstack((Tzm_mat[3:6, :], Tz_mat))
        czo[1, :] = np.linalg.solve(Tzf_mat, np.array([pzm, 0., 0., pzf, 0., 0.]))
    else:
        Tzf_mat = np.vstack((t_mat, Tz_mat))
        czo[1, :] = np.linalg.solve(Tzf_mat, np.array([pp[2], vp[2], ap[2], pzf, 0., 0.]))

    return cxo, cyo, czo


def get_swing_ref(cxo, cyo, czo,
                  p0, pzf, p1, T, t,
                  Txi, Txf,
                  Tyi, Tyf,
                  Tzm, Tzf):
    if t < T:
        t2  = t * t
        t3  = t * t2
        t4  = t * t3
        t5  = t * t4
        t_p = np.array([1., t,    t2,     t3,      t4,      t5])
        t_v = np.array([0., 1, 2 * t, 3 * t2,  4 * t3,  5 * t4])
        t_a = np.array([0., 0,     2,  6 * t, 12 * t2, 20 * t3])

        pt, vt, at = np.zeros(3), np.zeros(3), np.zeros(3)

        # x
        if t <= Txi:
            pt[0] = p0[0]
        elif Txi < t < T - Txf:
            pt[0] = cxo @ t_p
            vt[0] = cxo @ t_v
            at[0] = cxo @ t_a
        else:
            pt[0] = p1[0]

        # y
        if t <= Tyi:
            pt[1] = p0[1]
        elif Tyi < t < T - Tyf:
            pt[1] = cyo @ t_p
            vt[1] = cyo @ t_v
            at[1] = cyo @ t_a
        else:
            pt[1] = p1[1]

        # z
        if t <= Tzm:
            pt[2] = czo[0, :] @ t_p
            vt[2] = czo[0, :] @ t_v
            at[2] = czo[0, :] @ t_a
        elif Tzm < t < T - Tzf:
            pt[2] = czo[1, :] @ t_p
            vt[2] = czo[1, :] @ t_v
            at[2] = czo[1, :] @ t_a
        else:
            at[2] = 0.
            vt[2] = 0.
            pt[2] = pzf
    else:
        vzf = -0.5
        at  = np.array([0., 0., 0.0])
        vt  = np.array([0., 0., vzf])
        pt  = np.array([p1[0], p1[1], pzf + vzf * (t - T)])

    return pt, vt, at


def get_DCM_nom(vxd, vyd, dy, Ts, eTs):
    Lox = vxd * Ts  # nominal step difference [m]
    Woy = vyd * Ts  # nominal pelvis movement [m]

    # nominal initial DCM offset
    box  = Lox / (eTs - 1.)
    boy1 = Woy / (eTs - 1.)
    boy2 =  dy / (eTs + 1.)

    return Lox, Woy, box, boy1, boy2


def main_loop():
    # BRUCE SETUP
    Bruce = RDS.BRUCE()

    # CONTROL FREQUENCY
    loop_freq   = 1000  # run at 1000 Hz
    update_freq =  500  # DCM replanning frequency

    loop_duration   = 1. / loop_freq
    update_duration = 1. / update_freq

    # NOMINAL GAIT PATTERN
    omg = np.sqrt(9.81 / hz)  # LIPM natural frequency
    wTs = omg * Ts
    eTs = np.exp(wTs)

    # COST WEIGHTS
    W_L = np.diag([ 10.,  10.])
    W_b = np.diag([100., 100.])
    w_s = 0.1

    # QP SETUP
    # Decision Variables
    # x = [p1, b1, p2, b2, ... , pN, bN, es] - 2 * 2 * N + 1 = 4N + 1
    # pk - 2 (foothold location)
    # bk - 2 (initial DCM offset)
    # es - 1 (stance phase duration)
    N = 3           # number of steps
    Nv = 4 * N + 1  # number of decision variables

    # kinematic reachability [m]
    L_max_l = np.array([ lx, -lyi])
    L_min_l = np.array([-lx, -lyo])
    L_max_r = np.array([ lx,  lyo])
    L_min_r = np.array([-lx,  lyi])

    # Costs
    P0 = linalg.block_diag(W_L, W_b, np.zeros((Nv - 5, Nv - 5)), w_s)
    for idx in range(N - 1):
        id1 = 4 * idx
        id2 = id1 + 6
        P0[id1:id2, id1:id2] += np.kron(np.array([[ 1., 0., -1.],
                                                  [ 0., 0.,  0.],
                                                  [-1., 0.,  1.]]), W_L)
        id3 = id1 + 6
        id4 = id1 + 8
        P0[id3:id4, id3:id4] = W_b

    q0     = np.zeros(Nv)
    q0[-1] = -w_s * eTs

    # Equality Constraints
    # eq1 - DCM dynamics constraint: xi_k+1 = p_k + b_k * exp(w * Ts) - 2 * N cons
    Aeq           =  np.zeros((2 * N, Nv))
    Aeq[0:2, 0:2] = -np.eye(2)
    Aeq[0:2, 2:4] = -np.eye(2)
    Aeq[0:2, -1]  =  np.array([1., 1.])
    for idx in range(N - 1):
        Aeq[2*idx+2:2*idx+4, 4*idx:4*idx+8] = np.kron(np.array([1., eTs, -1., -1.]), np.eye(2))

    beq = np.zeros(2 * N)

    # Inequality Constraints
    # ineq1 - kinematic reachability: L_min <= L_k <= L_max - 2 * N cons
    L_max_1 = (L_max_r + L_max_l) / 2.
    L_max_2 = (L_max_r - L_max_l) / 2.
    L_min_1 = (L_min_r + L_min_l) / 2.
    L_min_2 = (L_min_r - L_min_l) / 2.

    Aineq1 = np.zeros((2 * N, Nv))
    Aineq1[0:2, 0:2] = np.ones((2, 2))

    for idx in range(N - 1):
        Aineq1[2*idx+2:2*idx+4, 4*idx:4*idx+6] = np.kron(np.array([-1., 0., 1.]), np.ones((2, 2)))

    bineq1 = np.zeros(2 * N)

    # ineq2 - phase duration limit: exp(w * Ts_min) <= es <= exp(w * Ts_max) - 1 con
    Aineq2 = np.zeros(Nv)
    Aineq2[-1] = 1.

    bineq2_l = np.exp(omg * Ts_min)
    bineq2_u = np.exp(omg * Ts_max)

    # Overall
    A0 = np.vstack((Aeq, Aineq1, Aineq2))
    l0 = np.hstack((beq, bineq1, bineq2_l))
    u0 = np.hstack((beq, bineq1, bineq2_u))

    qp_P = sparse.csc_matrix(P0)
    qp_A = sparse.csc_matrix(A0)

    # OSQP setup
    prob = osqp.OSQP()
    prob.setup(P=sparse.triu(qp_P, format='csc'), q=q0, A=qp_A, l=l0, u=u0, verbose=False, warm_start=True,
               eps_abs=1e-3, eps_rel=1e-3, max_iter=1000, check_termination=1, adaptive_rho_interval=50, scaling=10)

    # START CONTROL
    confirm = input('Start DCM Walking? (y/n) ')
    if confirm != 'y':
        exit()

    Bruce.update_robot_DCM_status()

    input_data = {'mode': np.zeros(1),
                  'com_xy_velocity': np.zeros(2),
                  'yaw_rate': np.zeros(1)}
    MM.USER_COMMAND.set(input_data)

    plan_data = {'mode':  np.zeros(1),
                 'phase': np.zeros(1),
                 'body_rot_matrix': np.copy(Bruce.R_yaw)}
    # robot mode - balance 0
    #              walking 1
    # robot phase - double stance 0
    #             - right  stance 1
    #             - left   stance 2

    leg_st = -1  # right stance +1
                 # left  stance -1

    T0 = Bruce.get_time()
    thread_run = False
    while True:
        # user input update
        Bruce.update_input_status()

        # robot state update
        Bruce.update_robot_DCM_status()

        # STOP WALKING AND KEEP BALANCING
        # check if command stop walking and actual CoM velocities are small
        if Bruce.cmd_mode == 0 and np.sqrt(Bruce.v_wg[0] ** 2 + Bruce.v_wg[1] ** 2) <= 0.20:
            # stop walking and move CoM to center
            plan_data['mode']  = np.zeros(1)
            plan_data['phase'] = np.zeros(1)

            p_wg_0 = np.copy(Bruce.p_wg)
            foot_mid_center = 0.5 * (Bruce.p_wf_r + Bruce.p_wf_l)
            x_pos_dir = 0.5 * (Bruce.p_wt_r + Bruce.p_wt_l) - foot_mid_center  # forward direction
            x_neg_dir = 0.5 * (Bruce.p_wh_r + Bruce.p_wh_l) - foot_mid_center  # backward direction
            p_wg_1    = foot_mid_center + ka * x_pos_dir
            p_wg_1[2] += hz

            pitch_angle = 0.0
            R_wb_0 = np.copy(Bruce.R_wb)
            phi_0  = MF.logvee(R_wb_0)
            R_wb_1 = MF.Rz(Bruce.yaw) @ MF.Ry(pitch_angle)
            phi_1  = MF.logvee(R_wb_1)

            t1 = 0.05
            te = 0.
            t0 = Bruce.get_time()
            while te <= t1:
                te = Bruce.get_time() - t0
                plan_data['com_position']    = p_wg_0 + (p_wg_1 - p_wg_0) / t1 * te
                plan_data['com_velocity']    = np.zeros(3)
                plan_data['body_rot_matrix'] = MF.hatexp(phi_0 + (phi_1 - phi_0) / t1 * te)

                MM.PLANNER_COMMAND.set(plan_data)
                time.sleep(0.001)

            # keep balancing until command walking
            p_wg_0 = np.copy(p_wg_1)
            R_wb_0 = np.copy(R_wb_1)
            while Bruce.cmd_mode == 0.:
                Te = Bruce.get_time() - T0

                if Te > 1:
                    if not thread_run:
                        MM.THREAD_STATE.set({'high_level': np.array([1.0])}, opt='only')  # thread is running
                        thread_run = True

                    # check threading error
                    if Bruce.thread_error():
                        Bruce.stop_threading()

                # user input update
                Bruce.update_input_status()

                # manipulating CoM
                # lateral
                y_pos_dir = Bruce.p_wf_l - foot_mid_center   # left  direction
                y_neg_dir = Bruce.p_wf_r - foot_mid_center   # right direction
                if Bruce.cmd_p_wg_change[1] >= 0.:
                    dp_xy = y_pos_dir[0:2] * +Bruce.cmd_p_wg_change[1]
                else:
                    dp_xy = y_neg_dir[0:2] * -Bruce.cmd_p_wg_change[1]

                # longitudinal
                if Bruce.cmd_p_wg_change[0] >= 0.:
                    dp_xy += x_pos_dir[0:2] * +Bruce.cmd_p_wg_change[0]
                else:
                    dp_xy += x_neg_dir[0:2] * -Bruce.cmd_p_wg_change[0]

                # vertical
                dp_z = Bruce.cmd_p_wg_change[2]

                plan_data['com_position'] = p_wg_0 + np.array([dp_xy[0],
                                                               dp_xy[1],
                                                               dp_z])

                # manipulating body orientation
                plan_data['body_rot_matrix'] = R_wb_0 @ Bruce.cmd_R_change

                MM.PLANNER_COMMAND.set(plan_data)
                time.sleep(0.001)

            # shift CoM for walking again
            Bruce.update_robot_DCM_status()
            p_wg_1 = np.copy(p_wg_0)
            if leg_st == +1.:
                p_st_xy = Bruce.p_wf_r[0:2]
            elif leg_st == -1.:
                p_st_xy = Bruce.p_wf_l[0:2]
            p_wg_1[0:2] += (p_st_xy - p_wg_1[0:2]) * 0.70 + ka * x_pos_dir[0:2]
            t1 = 0.8
            te = 0.
            t0 = Bruce.get_time()
            while te <= t1:
                te = Bruce.get_time() - t0
                plan_data['com_position'] = p_wg_0 + (p_wg_1 - p_wg_0) / t1 * te
                plan_data['com_velocity'] = np.zeros(3)

                MM.PLANNER_COMMAND.set(plan_data)
                time.sleep(0.001)

            # initialize walking parameters
            # each step can have different velocities
            vxd  = np.zeros(N)
            vyd  = np.zeros(N)
            Lox  = np.zeros(N)
            Woy  = np.zeros(N)
            box  = np.zeros(N)
            boy1 = np.zeros(N)
            boy2 = np.zeros(N)

            pxyd  = np.copy(Bruce.p_wg[0:2])
            yawd  = Bruce.yaw
            dyawd = 0.

            step_num = 0

        # initialize current cycle
        leg_st *= -1  # desired stance leg
        Ts_sol  = Ts  # optimal Ts [s]
        te      = 0.  # phase elapsed time [s]

        step_num += 1

        if step_num > 1:
            plan_data['mode']  = np.ones(1)
            plan_data['phase'] = np.array([1.5 - 0.5 * leg_st])

        # nominal pattern
        sf = 0.5  # smoothing factor
        vxd[0:N-1] = vxd[1:N]
        vxd[N-1]   = MF.exp_filter(vxd[N-1], Bruce.cmd_vxy_wg[0], sf)

        vyd[0:N-1] = vyd[1:N]
        vyd[N-1]   = MF.exp_filter(vyd[N-1], Bruce.cmd_vxy_wg[1], sf)

        pxyd += Bruce.R_yaw[0:2, 0:2] @ np.array([vxd[0], vyd[0]]) * Ts

        dyawd = MF.exp_filter(dyawd, Bruce.cmd_yaw_rate, sf)
        yawd += dyawd * Ts
        Rd    = MF.Rz(yawd) @ MF.Ry(pitch_angle) @ Bruce.cmd_R_change

        dyt = Bruce.cmd_dy

        if leg_st == +1.:
            p_sw_0 = np.copy(Bruce.p_wa_l)
            R_yawd_f = MF.Rz(yawd + Bruce.cmd_yaw_l_change + yaw_f_offset)
        elif leg_st == -1.:
            p_sw_0 = np.copy(Bruce.p_wa_r)
            R_yawd_f = MF.Rz(yawd + Bruce.cmd_yaw_r_change - yaw_f_offset)

        p1_sol = np.copy(p_sw_0)
        p1_nom = np.copy(p2_sol) if step_num > 1 else np.copy(p_sw_0)  # current nominal position is the previous solution

        des_t_sw, des_p_sw, des_v_sw, des_a_sw = te, p_sw_0, np.zeros(3), np.zeros(3)
        tu = 0.  # swing update timer

        dcm = Bruce.p_wg[0:2] + Bruce.v_wg[0:2] / omg

        t0 = Bruce.get_time()
        while True:  # continue the current cycle
            loop_start_time = Bruce.get_time()

            Te = loop_start_time - T0
            te = loop_start_time - t0
            tr = Ts_sol - te

            # check threading error
            if Bruce.thread_error():
                Bruce.stop_threading()

            # robot state update
            Bruce.update_robot_DCM_status()
            Rt = Bruce.R_yaw[0:2, 0:2]

            # stop current cycle
            contact_con = (te >= Ts_min and (Bruce.foot_contacts[int(leg_st+1)] or Bruce.foot_contacts[int(leg_st+2)])) or (te >= Ts_max and step_num == 1)
            if contact_con:  # stop current cycle if touchdown
                break

            if leg_st == +1.:
                p_st_m = Bruce.p_wf_r
                p_st_a = Bruce.p_wa_r
            elif leg_st == -1.:
                p_st_m = Bruce.p_wf_l
                p_st_a = Bruce.p_wa_l

            if leg_st == -1.:
                zm = zm_r
                zf = zf_r
            elif leg_st == +1.:
                zm = zm_l
                zf = zf_l

            pz0 = p_sw_0[2]
            if SIMULATION:
                pzm = p_st_a[2] + zm
                pzf = p_st_a[2] + zf
                # pzm = pz0 + zm
                # pzf = pz0 + zf
            else:
                pzm = p_st_a[2] + zm
                pzf = p_st_a[2] + zf
                # pzm = pz0 + zm
                # pzf = pz0 + zf
                
            for i in range(N):
                Lox[i], Woy[i], box[i], boy1[i], boy2[i] = get_DCM_nom(vxd[i], vyd[i], dyt, Ts, eTs)
                box[i]  += bx_offset
                boy1[i] += by_offset
                boy2[i] += by_offset
                
            alp = 0.95 if SIMULATION else 0.0
            for i in range(2):
                dcm[i] = MF.exp_filter(dcm[i], Bruce.p_wg[i] + Bruce.v_wg[i] / omg, alp)
            p_tau = p_st_a[0:2]
            b_tau = dcm - p_tau

            if tr >= T_buff and te >= tu:  # constantly planning (at update_freq Hz) if tr larger than T_buff
                # qp update
                qp_q = cost_update(q0, W_L, W_b, w_s,
                                   leg_st, Rt, p_tau, eTs, dyt, N,
                                   Lox, Woy, box, boy1, boy2)
                AA, qp_l, qp_u = constraint_update(A0, l0, u0, leg_st, Rt, te, p_tau, b_tau,
                                                   omg, eTs, N,
                                                   L_min_1, L_min_2, L_max_1, L_max_2)
                qp_A = sparse.csc_matrix(AA)

                # qp solve
                prob.update(Ax=qp_A.data, q=qp_q, l=qp_l, u=qp_u)
                sol = prob.solve()

                if sol.info.status != 'solved':
                    # qp infeasible
                    print(colored('OSQP did not solve the problem!!!', 'red'))
                else:
                    # qp solved
                    tu += update_duration

                    p1_sol = sol.x[0:2]                # current step location
                    p2_sol = sol.x[4:6]                # next    step location
                    Ts_sol = np.log(sol.x[-1]) / omg   # current step timing

                    if Ts_sol < Ts_min:
                        Ts_sol = Ts_min

                cx, cy, cz = get_swing_traj(te, des_t_sw, des_p_sw, des_v_sw, des_a_sw,
                                            Ts_sol, p1_sol, pz0, pzf, p1_nom, pzm,
                                            Txn, Txf,
                                            Tyn, Tyf,
                                            Tzm, Tzf)

            # compute references for swing foot
            des_p_sw, des_v_sw, des_a_sw = get_swing_ref(cx, cy, cz,
                                                         p_sw_0, pzf, p1_sol, Ts_sol, te,
                                                         Txi, Txf,
                                                         Tyi, Tyf,
                                                         Tzm, Tzf)
            des_t_sw = te

            # set planner command
            if step_num > 1:
                plan_data['body_rot_matrix'] = Rd
                plan_data['body_ang_rate']   = np.array([0., 0., dyawd])

                plan_data['com_position'] = np.array([pxyd[0], pxyd[1], p_st_m[2] + hz + Bruce.cmd_p_wg_change[2]])
                plan_data['com_velocity'] = Bruce.R_yaw @ np.array([vxd[0], vyd[0], 0.])

                if leg_st == +1.:
                    plan_data['left_foot_position']   = des_p_sw
                    plan_data['left_foot_velocity']   = des_v_sw
                    plan_data['left_foot_rot_matrix'] = R_yawd_f
                    plan_data['left_foot_ang_rate']   = np.array([0., 0., dyawd])
                elif leg_st == -1.:
                    plan_data['right_foot_position']   = des_p_sw
                    plan_data['right_foot_velocity']   = des_v_sw
                    plan_data['right_foot_rot_matrix'] = R_yawd_f
                    plan_data['right_foot_ang_rate']   = np.array([0., 0., dyawd])

                MM.PLANNER_COMMAND.set(plan_data)

            # loop time
            loop_target_time = loop_start_time + loop_duration
            present_time = Bruce.get_time()
            if present_time > loop_target_time:
                delay_time = 1000. * (present_time - loop_target_time)
                if delay_time > 1.:
                    print(colored('Delayed ' + str(delay_time)[0:5] + ' ms at Te = ' + str(Te)[0:5] + ' s', 'yellow'))
            else:
                while Bruce.get_time() < loop_target_time:
                    pass


if __name__ == '__main__':
    try:
        main_loop()
    except (NameError, KeyboardInterrupt) as error:
        MM.THREAD_STATE.set({'high_level': np.array([0.0])}, opt='only')  # thread is stopped
    except Exception as error:
        print(error)
        MM.THREAD_STATE.set({'high_level': np.array([2.0])}, opt='only')  # thread in error
