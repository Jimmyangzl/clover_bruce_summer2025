#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Compile BRUCE state estimation (Kalman filter version) ahead of time (AOT) using Numba
'''

import scipy.linalg
from numba.pycc import CC
from Settings.BRUCE_macros import *


cc = CC('BRUCE_estimation_KF')


# FREQUENCY SETTING
freq  = 500  # run at 500 Hz
dt    = 1. / freq
dt2   = dt * dt
dt2_2 = dt2 / 2.

# BODY POSITION AND VELOCITY
# constant
ns = 3 * 5  # number of states
nn = 3 * 4  # number of process noises
I3 = np.eye(3)
Is = np.eye(ns)

# model
Phi = np.copy(Is)
Phi[0:3, 3:6] = I3 * dt

B = np.zeros((ns, 3))
B[0:3, 0:3] = I3 * dt2_2
B[3:6, 0:3] = I3 * dt

Gam = np.zeros((ns, nn))
Gam[6:ns, 3:nn] = np.eye(ns - 6) * dt

# covariance
Qa  = 1e-1**2 * np.diag([1., 1., 1.])  # accelerometer noise
Qba = 1e-4**2 * np.diag([1., 1., 1.])  # accelerometer bias noise
Qc0 = 1.e5**2 * np.diag([1., 1., 1.])  # new foot contact noise
Qc1 = 1e-3**2 * np.diag([1., 1., 1.])  # foot contact noise
Q   = scipy.linalg.block_diag(Qa, Qba, Qc1, Qc1)

Vp  = 1e-3**2 * np.diag([1., 1., 1.])  # foot position FK noise
Vv  = 1e-2**2 * np.diag([1., 1., 1.])  # foot velocity FK noise


@cc.export('run', '(f8[:,:], f8[:], f8[:], f8[:], f8[:], f8[:], f8[:], f8[:], f8[:,:],'
                  'f8[:], f8[:], f8[:], f8[:], f8[:], f8[:],'
                  'f8[:,:], f8[:], f8[:], f8[:], f8)')
def run(R0, w0, p0, v0, a0, ba0, crm0, clm0, P0,
        prm, plm, vrm, vlm, crm, clm,
        Rm, wm, am, contacts_count, g):

    # SETTING
    # constant
    contacts_num = int(np.sum(contacts_count > 0))  # number of foot contacts

    # previous a posteriori state estimate
    x0 = np.hstack((p0, v0, ba0, crm0, clm0))

    # current available measurements
    z1 = np.hstack((prm, plm,
                    vrm, vlm,
                    crm, clm))

    # covariance reset for new foot contacts
    for i in range(2):  # now we only consider one contact for each foot
        if contacts_count[i] == 1:
            id1 = 3 * i + 9
            id2 = id1 + 3
            id3 = id2 + 3

            P0[id1:id2,    0:ns] = np.zeros((3, ns))
            P0[   0:ns, id1:id2] = np.zeros((ns, 3))
            P0[id1:id2, id1:id2] = Qc0

            x0[id1:id2] = z1[id2:id3]

    # POSITION AND VELOCITY ESTIMATE
    # predict
    Phik = np.copy(Phi)
    Phik[0:3, 6:9] = -R0 * dt2_2
    Phik[3:6, 6:9] = -R0 * dt

    Gamk = np.copy(Gam)
    Gamk[0:6, 0:3] = Phik[0:6, 6:9]

    P1 = Phik @ np.copy(P0) @ Phik.T + Gamk @ Q @ Gamk.T
    x1 = np.copy(x0)
    x1[0:3] = p0 + v0 * dt + a0 * dt2_2
    x1[3:6] = v0 + a0 * dt

    RT  = np.copy(Rm).T
    wRT = MF.hat(wm) @ RT

    # update
    if contacts_num > 0:
        nm = 6 * contacts_num  # current number of measurements
        Hk = np.zeros((nm, ns))
        Vk = np.zeros((nm, nm))
        yk = np.zeros(nm)      # innovation
        j  = 0
        for i in range(2):
            if contacts_count[i] > 0:
                id1 = 3 * j
                id2 = id1 + 3
                id3 = id1 + 3 * contacts_num
                id4 = id3 + 3

                id5 = 3 * i
                id6 = id5 + 3
                id7 = id6 + 3
                id8 = id7 + 3
                id9 = id8 + 3

                Hk[id1:id2,     0:3] = -RT
                Hk[id1:id2, id8:id9] =  RT
                Hk[id3:id4,     0:3] =  wRT
                Hk[id3:id4,     3:6] = -RT
                Hk[id3:id4, id8:id9] = -wRT

                Vk[id1:id2, id1:id2] = Vp
                Vk[id3:id4, id3:id4] = Vv

                ci_p = x1[id8:id9] - x1[0:3]
                yk[id1:id2] = z1[id5:id6] -  RT @ ci_p
                yk[id3:id4] = z1[id7:id8] + wRT @ ci_p + RT @ x1[3:6]

                j += 1

        PHT    = P1 @ Hk.T
        Sk     = Hk @ PHT + Vk
        Sk_inv = np.linalg.pinv(Sk)
        Kk     = PHT @ Sk_inv
        IKH    = Is - Kk @ Hk
        P1     = IKH @ P1 @ IKH.T + Kk @ Vk @ Kk.T
        # P1     = IKH @ P1
        x1    += Kk @ yk

    p1   = x1[0:3]
    v1   = x1[3:6]
    ba1  = x1[6:9]
    crm1 = x1[9:12]
    clm1 = x1[12:15]

    a1  = np.copy(Rm) @ (am - ba1) - np.array([0., 0., g])  # body acceleration excluding gravity
    bv1 = RT @ v1
    yaw = np.arctan2(Rm[1, 0], Rm[0, 0])

    return Rm, wm, p1, v1, a1, ba1, crm1, clm1, P1, \
           bv1, yaw


if __name__ == '__main__':
    cc.compile()
