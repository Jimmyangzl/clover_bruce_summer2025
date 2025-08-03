#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Compile BRUCE orientation estimation ahead of time (AOT) using Numba
'''

from numba.pycc import CC
from Settings.BRUCE_macros import *


cc = CC('BRUCE_orientation')


# FREQUENCY SETTING
freq = 500  # run at 500 Hz
dt   = 1. / freq

@cc.export('run', '(f8[:,:], f8[:],'
                  'f8[:], f8[:], f8, f8)')
def run(R0, w0,
        omg, acc, g, kR):
    # predict
    R1 = np.copy(R0) @ MF.hatexp(w0 * dt)
    w1 = omg

    # update
    gm   = R1 @ np.copy(acc)
    gmu  = gm / MF.norm(gm)
    dphi = np.arccos(gmu[2] * np.sign(g))
    nv   = np.zeros(3) if np.abs(dphi) < 1e-10 else MF.hat(gmu) @ np.array([0., 0., np.sign(g)]) / np.sin(dphi)  # rotation axis
    R1   = MF.hatexp(kR * dphi * nv) @ R1

    return R1, w1


if __name__ == '__main__':
    cc.compile()
