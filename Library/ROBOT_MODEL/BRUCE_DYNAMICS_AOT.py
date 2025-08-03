#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Compile BRUCE full-body dynamics ahead of time (AOT) using Numba
'''

from numba.pycc import CC
from Settings.BRUCE_macros import *


cc = CC('BRUCE_dynamics')


r_01_hat = MF.hat(COORDINATE_VECTOR[HIP_YAW_R])
r_12_hat = MF.hat(COORDINATE_VECTOR[HIP_ROLL_R])
r_23_hat = MF.hat(COORDINATE_VECTOR[HIP_PITCH_R])
r_34_hat = MF.hat(COORDINATE_VECTOR[KNEE_PITCH_R])
r_45_hat = MF.hat(COORDINATE_VECTOR[ANKLE_PITCH_R])

r_06_hat  = MF.hat(COORDINATE_VECTOR[HIP_YAW_L])
r_67_hat  = MF.hat(COORDINATE_VECTOR[HIP_ROLL_L])
r_78_hat  = MF.hat(COORDINATE_VECTOR[HIP_PITCH_L])
r_89_hat  = MF.hat(COORDINATE_VECTOR[KNEE_PITCH_L])
r_910_hat = MF.hat(COORDINATE_VECTOR[ANKLE_PITCH_L])

Is_0  = SPATIAL_INERTIA[BODY]

Is_1  = SPATIAL_INERTIA[HIP_YAW_R]
Is_2  = SPATIAL_INERTIA[HIP_ROLL_R]
Is_3  = SPATIAL_INERTIA[HIP_PITCH_R]
Is_4  = SPATIAL_INERTIA[KNEE_PITCH_R]
Is_5  = SPATIAL_INERTIA[ANKLE_PITCH_R]

Is_6  = SPATIAL_INERTIA[HIP_YAW_L]
Is_7  = SPATIAL_INERTIA[HIP_ROLL_L]
Is_8  = SPATIAL_INERTIA[HIP_PITCH_L]
Is_9  = SPATIAL_INERTIA[KNEE_PITCH_L]
Is_10 = SPATIAL_INERTIA[ANKLE_PITCH_L]

e3 = np.array([0., 0., 1., 0., 0., 0.])
S6 = np.hstack((np.eye(6), np.zeros((6, 10))))


@cc.export('robotID', '(f8[:,:], f8[:], f8[:], f8[:],'
                      'f8, f8, f8, f8, f8, f8, f8, f8, f8, f8,'
                      'f8, f8, f8, f8, f8, f8, f8, f8, f8, f8)')
def robotID(R, p, w, bv,
            q1, q2, q3, q4, q5, q6, q7, q8, q9, q10,
            dq1, dq2, dq3, dq4, dq5, dq6, dq7, dq8, dq9, dq10):
    R   = np.copy(R)
    v_0 = np.hstack((w, bv))

    X0 = np.zeros((6, 6))

    X1 = np.zeros((6, 6))
    X2 = np.zeros((6, 6))
    X3 = np.zeros((6, 6))
    X4 = np.zeros((6, 6))
    X5 = np.zeros((6, 6))

    X6  = np.zeros((6, 6))
    X7  = np.zeros((6, 6))
    X8  = np.zeros((6, 6))
    X9  = np.zeros((6, 6))
    X10 = np.zeros((6, 6))

    X0[0:3, 0:3] = R
    X0[3:6, 3:6] = R

    c1 = np.cos(q1)
    s1 = np.sin(q1)
    R1 = np.array([[c1, -s1, 0.], [s1, c1, 0.], [0., 0., 1.]])
    X1[0:3, 0:3] = R1
    X1[3:6, 3:6] = R1
    X1[0:3, 3:6] = r_01_hat @ R1

    c2 = np.cos(q2)
    s2 = np.sin(q2)
    R2 = np.array([[c2, -s2, 0.], [0., 0., 1.], [-s2, -c2, 0.]])
    X2[0:3, 0:3] = R2
    X2[3:6, 3:6] = R2
    X2[0:3, 3:6] = r_12_hat @ R2

    c3 = np.cos(q3)
    s3 = np.sin(q3)
    R3 = np.array([[c3, -s3, 0.], [0., 0., -1.], [s3, c3, 0.]])
    X3[0:3, 0:3] = R3
    X3[3:6, 3:6] = R3
    X3[0:3, 3:6] = r_23_hat @ R3

    c4 = np.cos(q4)
    s4 = np.sin(q4)
    R4 = np.array([[c4, -s4, 0.], [s4, c4, 0.], [0., 0., 1.]])
    X4[0:3, 0:3] = R4
    X4[3:6, 3:6] = R4
    X4[0:3, 3:6] = r_34_hat @ R4

    c5 = np.cos(q5)
    s5 = np.sin(q5)
    R5 = np.array([[c5, -s5, 0.], [s5, c5, 0.], [0., 0., 1.]])
    X5[0:3, 0:3] = R5
    X5[3:6, 3:6] = R5
    X5[0:3, 3:6] = r_45_hat @ R5

    c6 = np.cos(q6)
    s6 = np.sin(q6)
    R6 = np.array([[c6, -s6, 0.], [s6, c6, 0.], [0., 0., 1.]])
    X6[0:3, 0:3] = R6
    X6[3:6, 3:6] = R6
    X6[0:3, 3:6] = r_06_hat @ R6

    c7 = np.cos(q7)
    s7 = np.sin(q7)
    R7 = np.array([[c7, -s7, 0.], [0., 0., 1.], [-s7, -c7, 0.]])
    X7[0:3, 0:3] = R7
    X7[3:6, 3:6] = R7
    X7[0:3, 3:6] = r_67_hat @ R7

    c8 = np.cos(q8)
    s8 = np.sin(q8)
    R8 = np.array([[c8, -s8, 0.], [0., 0., -1.], [s8, c8, 0.]])
    X8[0:3, 0:3] = R8
    X8[3:6, 3:6] = R8
    X8[0:3, 3:6] = r_78_hat @ R8

    c9 = np.cos(q9)
    s9 = np.sin(q9)
    R9 = np.array([[c9, -s9, 0.], [s9, c9, 0.], [0., 0., 1.]])
    X9[0:3, 0:3] = R9
    X9[3:6, 3:6] = R9
    X9[0:3, 3:6] = r_89_hat @ R9

    c10 = np.cos(q10)
    s10 = np.sin(q10)
    R10 = np.array([[c10, -s10, 0.], [s10, c10, 0.], [0., 0., 1.]])
    X10[0:3, 0:3] = R10
    X10[3:6, 3:6] = R10
    X10[0:3, 3:6] = r_910_hat @ R10

    X0T = X0.T

    X1T = X1.T
    X2T = X2.T
    X3T = X3.T
    X4T = X4.T
    X5T = X5.T

    X6T  = X6.T
    X7T  = X7.T
    X8T  = X8.T
    X9T  = X9.T
    X10T = X10.T

    # inverse dynamics
    # calculate C
    # body
    vHatstar_0 = MF.Hatstar(v_0)
    fv_0 = vHatstar_0 @ Is_0 @ v_0
    f_0  = fv_0

    # right
    vJ_1 = dq1 * e3
    v_1  = X1T @ v_0 + vJ_1
    vHat_1, vHatstar_1 = MF.HatStar(v_1)
    av_1 = vHat_1 @ vJ_1
    a_1  = av_1
    fv_1 = vHatstar_1 @ Is_1 @ v_1
    f_1  = Is_1 @ a_1 + fv_1

    vJ_2 = dq2 * e3
    v_2  = X2T @ v_1 + vJ_2
    vHat_2, vHatstar_2 = MF.HatStar(v_2)
    av_2 = vHat_2 @ vJ_2
    a_2  = X2T @ a_1 + av_2
    fv_2 = vHatstar_2 @ Is_2 @ v_2
    f_2  = Is_2 @ a_2 + fv_2

    vJ_3 = dq3 * e3
    v_3  = X3T @ v_2 + vJ_3
    vHat_3, vHatstar_3 = MF.HatStar(v_3)
    av_3 = vHat_3 @ vJ_3
    a_3  = X3T @ a_2 + av_3
    fv_3 = vHatstar_3 @ Is_3 @ v_3
    f_3  = Is_3 @ a_3 + fv_3

    vJ_4 = dq4 * e3
    v_4  = X4T @ v_3 + vJ_4
    vHat_4, vHatstar_4 = MF.HatStar(v_4)
    av_4 = vHat_4 @ vJ_4
    a_4  = X4T @ a_3 + av_4
    fv_4 = vHatstar_4 @ Is_4 @ v_4
    f_4  = Is_4 @ a_4 + fv_4

    vJ_5 = dq5 * e3
    v_5  = X5T @ v_4 + vJ_5
    vHat_5, vHatstar_5 = MF.HatStar(v_5)
    av_5 = vHat_5 @ vJ_5
    a_5  = X5T @ a_4 + av_5
    fv_5 = vHatstar_5 @ Is_5 @ v_5
    f_5  = Is_5 @ a_5 + fv_5

    # left
    vJ_6 = dq6 * e3
    v_6  = X6T @ v_0 + vJ_6
    vHat_6, vHatstar_6 = MF.HatStar(v_6)
    av_6 = vHat_6 @ vJ_6
    a_6  = av_6
    fv_6 = vHatstar_6 @ Is_6 @ v_6
    f_6  = Is_6 @ a_6 + fv_6

    vJ_7 = dq7 * e3
    v_7  = X7T @ v_6 + vJ_7
    vHat_7, vHatstar_7 = MF.HatStar(v_7)
    av_7 = vHat_7 @ vJ_7
    a_7  = X7T @ a_6 + av_7
    fv_7 = vHatstar_7 @ Is_7 @ v_7
    f_7  = Is_7 @ a_7 + fv_7

    vJ_8 = dq8 * e3
    v_8  = X8T @ v_7 + vJ_8
    vHat_8, vHatstar_8 = MF.HatStar(v_8)
    av_8 = vHat_8 @ vJ_8
    a_8  = X8T @ a_7 + av_8
    fv_8 = vHatstar_8 @ Is_8 @ v_8
    f_8  = Is_8 @ a_8 + fv_8

    vJ_9 = dq9 * e3
    v_9  = X9T @ v_8 + vJ_9
    vHat_9, vHatstar_9 = MF.HatStar(v_9)
    av_9 = vHat_9 @ vJ_9
    a_9  = X9T @ a_8 + av_9
    fv_9 = vHatstar_9 @ Is_9 @ v_9
    f_9  = Is_9 @ a_9 + fv_9

    vJ_10 = dq10 * e3
    v_10  = X10T @ v_9 + vJ_10
    vHat_10, vHatstar_10 = MF.HatStar(v_10)
    av_10 = vHat_10 @ vJ_10
    a_10  = X10T @ a_9 + av_10
    fv_10 = vHatstar_10 @ Is_10 @ v_10
    f_10  = Is_10 @ a_10 + fv_10

    # calculate C
    C = np.zeros(16)

    C[15] = f_10[2]
    f_9 += X10 @ f_10

    C[14] = f_9[2]
    f_8 += X9 @ f_9

    C[13] = f_8[2]
    f_7 += X8 @ f_8

    C[12] = f_7[2]
    f_6 += X7 @ f_7

    C[11] = f_6[2]
    f_0 += X6 @ f_6

    C[10] = f_5[2]
    f_4 += X5 @ f_5

    C[9] = f_4[2]
    f_3 += X4 @ f_4

    C[8] = f_3[2]
    f_2 += X3 @ f_3

    C[7] = f_2[2]
    f_1 += X2 @ f_2

    C[6] = f_1[2]
    f_0 += X1 @ f_1

    C[0:6] = f_0

    # calculate C+G
    spatial_a_w = np.array([0., 0., 0., 0., 0., 9.81])

    # body
    a_0 =  X0T @ spatial_a_w
    f_0 = Is_0 @ a_0 + fv_0

    # right
    a_1 =  X1T @ a_0 + av_1
    f_1 = Is_1 @ a_1 + fv_1

    a_2 =  X2T @ a_1 + av_2
    f_2 = Is_2 @ a_2 + fv_2

    a_3 =  X3T @ a_2 + av_3
    f_3 = Is_3 @ a_3 + fv_3

    a_4 =  X4T @ a_3 + av_4
    f_4 = Is_4 @ a_4 + fv_4

    a_5 =  X5T @ a_4 + av_5
    f_5 = Is_5 @ a_5 + fv_5

    # left
    a_6 =  X6T @ a_0 + av_6
    f_6 = Is_6 @ a_6 + fv_6

    a_7 =  X7T @ a_6 + av_7
    f_7 = Is_7 @ a_7 + fv_7

    a_8 =  X8T @ a_7 + av_8
    f_8 = Is_8 @ a_8 + fv_8

    a_9 =  X9T @ a_8 + av_9
    f_9 = Is_9 @ a_9 + fv_9

    a_10 =  X10T @  a_9 + av_10
    f_10 = Is_10 @ a_10 + fv_10

    CG = np.zeros(16)

    CG[15] = f_10[2]
    f_9 += X10 @ f_10

    CG[14] = f_9[2]
    f_8 += X9 @ f_9

    CG[13] = f_8[2]
    f_7 += X8 @ f_8

    CG[12] = f_7[2]
    f_6 += X7 @ f_7

    CG[11] = f_6[2]
    f_0 += X6 @ f_6

    CG[10] = f_5[2]
    f_4 += X5 @ f_5

    CG[9] = f_4[2]
    f_3 += X4 @ f_4

    CG[8] = f_3[2]
    f_2 += X3 @ f_3

    CG[7] = f_2[2]
    f_1 += X2 @ f_2

    CG[6] = f_1[2]
    f_0 += X1 @ f_1

    CG[0:6] = f_0

    # CRBA
    H = np.zeros((16, 16))

    Ic_0  = np.copy(Is_0)
    Ic_1  = np.copy(Is_1)
    Ic_2  = np.copy(Is_2)
    Ic_3  = np.copy(Is_3)
    Ic_4  = np.copy(Is_4)
    Ic_5  = np.copy(Is_5)
    Ic_6  = np.copy(Is_6)
    Ic_7  = np.copy(Is_7)
    Ic_8  = np.copy(Is_8)
    Ic_9  = np.copy(Is_9)
    Ic_10 = np.copy(Is_10)

    # i = 10
    Ic_9 += X10 @ Ic_10 @ X10T
    F = Ic_10 @ e3
    H[15, 15] = Ic_10[2, 2]
    # k = 10
    F = X10 @ F
    H[15, 14] = F[2]
    H[14, 15] = F[2]
    # k = 9
    F = X9 @ F
    H[15, 13] = F[2]
    H[13, 15] = F[2]
    # k = 8
    F = X8 @ F
    H[15, 12] = F[2]
    H[12, 15] = F[2]
    # k = 7
    F = X7 @ F
    H[15, 11] = F[2]
    H[11, 15] = F[2]
    # k = 6
    F = X6 @ F
    H[15, 0:6] = F
    H[0:6, 15] = F.T

    # i = 9
    Ic_8 += X9 @ Ic_9 @ X9T
    F = Ic_9 @ e3
    H[14, 14] = Ic_9[2, 2]
    # k = 9
    F = X9 @ F
    H[14, 13] = F[2]
    H[13, 14] = F[2]
    # k = 8
    F = X8 @ F
    H[14, 12] = F[2]
    H[12, 14] = F[2]
    # k = 7
    F = X7 @ F
    H[14, 11] = F[2]
    H[11, 14] = F[2]
    # k = 6
    F = X6 @ F
    H[14, 0:6] = F
    H[0:6, 14] = F.T

    # i = 8
    Ic_7 += X8 @ Ic_8 @ X8T
    F = Ic_8 @ e3
    H[13, 13] = Ic_8[2, 2]
    # k = 8
    F = X8 @ F
    H[13, 12] = F[2]
    H[12, 13] = F[2]
    # k = 7
    F = X7 @ F
    H[13, 11] = F[2]
    H[11, 13] = F[2]
    # k = 6
    F = X6 @ F
    H[13, 0:6] = F
    H[0:6, 13] = F.T

    # i = 7
    Ic_6 += X7 @ Ic_7 @ X7T
    F = Ic_7 @ e3
    H[12, 12] = Ic_7[2, 2]
    # k = 7
    F = X7 @ F
    H[12, 11] = F[2]
    H[11, 12] = F[2]
    # k = 6
    F = X6 @ F
    H[12, 0:6] = F
    H[0:6, 12] = F.T

    # i = 6
    Ic_0 += X6 @ Ic_6 @ X6T
    F = Ic_6 @ e3
    H[11, 11] = Ic_6[2, 2]
    # k = 6
    F = X6 @ F
    H[11, 0:6] = F
    H[0:6, 11] = F.T

    # i = 5
    Ic_4 += X5 @ Ic_5 @ X5T
    F = Ic_5 @ e3
    H[10, 10] = Ic_5[2, 2]
    # k = 5
    F = X5 @ F
    H[10, 9] = F[2]
    H[9, 10] = F[2]
    # k = 4
    F = X4 @ F
    H[10, 8] = F[2]
    H[8, 10] = F[2]
    # k = 3
    F = X3 @ F
    H[10, 7] = F[2]
    H[7, 10] = F[2]
    # k = 2
    F = X2 @ F
    H[10, 6] = F[2]
    H[6, 10] = F[2]
    # k = 1
    F = X1 @ F
    H[10, 0:6] = F
    H[0:6, 10] = F.T

    # i = 4
    Ic_3 += X4 @ Ic_4 @ X4T
    F = Ic_4 @ e3
    H[9, 9] = Ic_4[2, 2]
    # k = 4
    F = X4 @ F
    H[9, 8] = F[2]
    H[8, 9] = F[2]
    # k = 3
    F = X3 @ F
    H[9, 7] = F[2]
    H[7, 9] = F[2]
    # k = 2
    F = X2 @ F
    H[9, 6] = F[2]
    H[6, 9] = F[2]
    # k = 1
    F = X1 @ F
    H[9, 0:6] = F
    H[0:6, 9] = F.T

    # i = 3
    Ic_2 += X3 @ Ic_3 @ X3T
    F = Ic_3 @ e3
    H[8, 8] = Ic_3[2, 2]
    # k = 3
    F = X3 @ F
    H[8, 7] = F[2]
    H[7, 8] = F[2]
    # k = 2
    F = X2 @ F
    H[8, 6] = F[2]
    H[6, 8] = F[2]
    # k = 1
    F = X1 @ F
    H[8, 0:6] = F
    H[0:6, 8] = F.T

    # i = 2
    Ic_1 += X2 @ Ic_2 @ X2T
    F = Ic_2 @ e3
    H[7, 7] = Ic_2[2, 2]
    # k = 2
    F = X2 @ F
    H[7, 6] = F[2]
    H[6, 7] = F[2]
    # k = 1
    F = X1 @ F
    H[7, 0:6] = F
    H[0:6, 7] = F.T

    # i = 1
    Ic_0 += X1 @ Ic_1 @ X1T
    F = Ic_1 @ e3
    H[6, 6] = Ic_1[2, 2]
    # k = 1
    F = X1 @ F
    H[6, 0:6] = F
    H[0:6, 6] = F.T

    # i = 0
    H[0:6, 0:6] = Ic_0

    # adjust for sparsity
    H[0:6, :] += 1e-12 * np.ones((6, 16))

    bp_G = np.array([H[2, 4], H[0, 5], H[1, 3]]) / MASS_TOT  # CoM in body frame

    X0[0:3, 3:6] = -R @ MF.hat(bp_G)
    AG    = X0 @ S6 @ H
    dAGdq = X0 @ C[0:6]

    pcom = p + R @ bp_G
    h    = AG @ np.hstack((v_0, np.array([dq1, dq2, dq3, dq4, dq5, dq6, dq7, dq8, dq9, dq10])))
    vcom = h[3:6] / MASS_TOT

    return H, CG, AG, dAGdq, pcom, vcom, h[0:3]


if __name__ == '__main__':
    cc.compile()
