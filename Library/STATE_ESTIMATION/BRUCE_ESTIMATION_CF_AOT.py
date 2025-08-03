#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Compile BRUCE state estimation (complementary filter version) ahead of time (AOT) using Numba
'''

from numba.pycc import CC
from Settings.BRUCE_macros import *


cc = CC('BRUCE_estimation_CF')


# FREQUENCY SETTING
freq  = 500.  # run at 500 Hz
dt    = 1. / freq
dt2   = dt * dt
dt2_2 = dt2 / 2.


@cc.export('run', '(f8[:,:], f8[:], f8[:], f8[:], f8[:], f8[:], f8[:],'
                  'f8[:], f8[:], f8[:], f8[:], f8[:], f8[:],'
                  'f8[:,:], f8[:], f8[:], f8[:], f8,'
                  'f8[:], f8[:])')
def run(R0, w0, p0, v0, a0, crm0, clm0,
        prm, plm, vrm, vlm, crm, clm,
        Rm, wm, am, foot_contacts, g,
        kp, kv):
    # POSITION AND VELOCITY ESTIMATE
    # predict
    p1 = p0 + v0 * dt + a0 * dt2_2
    v1 = v0 + a0 * dt
    crm1 = crm0
    clm1 = clm0

    # update
    pc   = np.zeros(3)
    vc   = np.zeros(3)
    what = MF.hat(wm)
    Rm   = np.copy(Rm)
    total_contacts = 0
    if foot_contacts[0]:
        total_contacts += 1
        prm = np.copy(prm)
        pc += crm1 - Rm @ prm
        vc -= Rm @ (what @ prm + vrm)
    else:
        crm1 = crm

    if foot_contacts[1]:
        total_contacts += 1
        plm = np.copy(plm)
        pc += clm1 - Rm @ plm
        vc -= Rm @ (what @ plm + vlm)
    else:
        clm1 = clm

    if total_contacts == 0:  # do nothing if lose all contacts to prevent divergence
        p1 = p0
        v1 = v0
    else:
        pc /= total_contacts
        vc /= total_contacts
        for i in range(3):
            p1[i] = kp[i] * pc[i] + (1. - kp[i]) * p1[i]

        for i in range(3):
            v1[i] = kv[i] * vc[i] + (1. - kv[i]) * v1[i]
    
    a1  = Rm @ np.copy(am) - np.array([0., 0., g])  # body acceleration excluding gravity
    bv1 = Rm.T @ np.copy(v1)
    yaw = np.arctan2(Rm[1, 0], Rm[0, 0])

    return Rm, wm, p1, v1, a1, crm1, clm1, \
           bv1, yaw


if __name__ == '__main__':
    cc.compile()
