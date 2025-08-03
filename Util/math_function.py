#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Some useful math functions
'''

import numpy as np
from numba import jit


@jit(nopython=True)
def norm(x):
    """
    compute the norm of a vector in R^3
    """
    return np.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2])


@jit(nopython=True)
def sat(x, x_min, x_max):
    """
    simple saturation function
    """
    if x > x_max:
        return x_max
    elif x < x_min:
        return x_min
    else:
        return x


@jit(nopython=True)
def hat(p):
    """
    convert R^3 to so(3)
    """
    return np.array([[    0, -p[2],  p[1]],
                     [ p[2],     0, -p[0]],
                     [-p[1],  p[0],    0]])


@jit(nopython=True)
def vee(S):
    """
    convert so(3) to R^3
    """
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


@jit(nopython=True)
def hatexp(a):
    """
    convert rotation vector to rotation matrix
    """
    ah  = hat(a)
    ah2 = ah @ ah
    an2 = a @ a
    an  = np.sqrt(an2)
    if an < 1e-10:
        return np.eye(3)
    else:
        return np.eye(3) + np.sin(an) / an * ah + (1 - np.cos(an)) / an2 * ah2


@jit(nopython=True)
def logvee(R):
    """
    convert rotation matrix to rotation vector
    """
    va = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.)
    va = sat(va, -1., 1.)  # need to regulate due to potential numerical errors to make sure arccos works
    theta = np.arccos(va)
    if np.abs(theta) < 1e-10:
        return np.zeros(3)
    else:
        logR = theta / 2. / np.sin(theta) * (R - R.T)
        return np.array([logR[2, 1], logR[0, 2], logR[1, 0]])


@jit(nopython=True)
def Hat(v):
    Z = np.zeros((6, 6))
    Z[0:3, 0:3] = hat(v[0:3])
    Z[3:6, 3:6] = Z[0:3, 0:3]
    Z[3:6, 0:3] = hat(v[3:6])
    return Z


@jit(nopython=True)
def Hatstar(v):
    Z = np.zeros((6, 6))
    Z[0:3, 0:3] = hat(v[0:3])
    Z[3:6, 3:6] = Z[0:3, 0:3]
    Z[0:3, 3:6] = hat(v[3:6])
    return Z


@jit(nopython=True)
def HatStar(v):
    Z = np.zeros((6, 6))
    Z[0:3, 0:3] = hat(v[0:3])
    Z[3:6, 3:6] = Z[0:3, 0:3]
    Zstar = Z
    Z[3:6, 0:3] = hat(v[3:6])
    Zstar[0:3, 3:6] = Z[3:6, 0:3]
    return Z, Zstar


@jit(nopython=True)
def Rz(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  -s, 0.],
                     [s,   c, 0.],
                     [0., 0., 1.]])


@jit(nopython=True)
def Ry(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[ c,  0.,  s],
                     [ 0., 1., 0.],
                     [-s,  0.,  c]])


@jit(nopython=True)
def Rx(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1., 0., 0.],
                     [0.,  c, -s],
                     [0.,  s,  c]])


@jit(nopython=True)
def quat2rotm(Q):
    """
    convert quaternion to rotation matrix
    """
    Q  = Q / np.linalg.norm(Q)
    q  = Q[0:3]
    q4 = Q[3]
    R  = (q4 * q4 - q @ q) * np.eye(3) + 2.0 * q4 * hat(q) + 2.0 * np.outer(q, q)
    return R


@jit(nopython=True)
def exp_filter(history, present, weight):
    """
    exponential filter
    result = history * weight + present * (1. - weight)
    """
    return present + (history - present) * weight


if __name__ == "__main__":
    pass
