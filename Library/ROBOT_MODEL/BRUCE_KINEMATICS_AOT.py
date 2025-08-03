#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Compile BRUCE full-body kinematics ahead of time (AOT) using Numba
'''

from numba.pycc import CC
from Settings.BRUCE_macros import *


cc = CC('BRUCE_kinematics')


@cc.export('legIK_foot', '(f8[:], f8[:], f8)')
def legIK_foot(bp_foot, bx_foot, leg):
    # bp_foot is foot position in body frame
    # bx_foot is foot pointing direction in body frame

    r11 = bx_foot[0]
    r21 = bx_foot[1]
    r31 = bx_foot[2]
    dx  = bp_foot[0] - hx
    dy  = bp_foot[1] + hy * leg   # leg = -1. if left; +1. if right
    dz  = bp_foot[2] + hz

    k1 = dx * r31 - dz * r11
    k2 = dy * r31 - dz * r21
    t1 = np.arctan2(-k1, k2)
    c1 = np.cos(t1)
    s1 = np.sin(t1)

    dxc1_dys1 = dx * c1 + dy * s1

    t2 = np.arctan2(-dz, dxc1_dys1)
    c2 = np.cos(t2)
    s2 = np.sin(t2)

    c345 = r21 * c1 - r11 * s1

    s2n = np.abs(s2)
    if s2n > 1e-6:
        s345 = r31 / s2
        k3   = -dz / s2 - a5 * c345
    else:
        s345 = -(r11 * c1 + r21 * s1) / c2
        k3   = dxc1_dys1 / c2 - a5 * c345

    k4 = dy * c1 - dx * s1 - a5 * s345 - d2
    c4 = (k3 ** 2. + k4 ** 2. - a3 ** 2. - a4 ** 2.) / 2. / a3 / a4
    if c4 > 1.:
        c4 = 1.
    elif c4 < -1.:
        c4 = -1.
    s4 = -np.sqrt(1 - c4 ** 2.)
    t4 = np.arctan2(s4, c4)

    a34 = a3 + a4 * c4
    as4 = a4 * s4
    t3  = np.arctan2(a34 * k4 - as4 * k3, a34 * k3 + as4 * k4)
    t5  = np.arctan2(s345, c345) - t3 - t4
    return t1, t2, t3, t4, t5


@cc.export('legIK_ankle', '(f8[:], f8[:], f8)')
def legIK_ankle(bp_foot, bx_foot, leg):
    # bp_foot is foot position in body frame
    # bx_foot is foot pointing direction in body frame

    r11 = bx_foot[0]
    r21 = bx_foot[1]
    r31 = bx_foot[2]
    dx  = bp_foot[0] - hx
    dy  = bp_foot[1] + hy * leg   # leg = -1. if left; +1. if right
    dz  = bp_foot[2] + hz

    k1 = dx * r31 - dz * r11
    k2 = dy * r31 - dz * r21
    t1 = np.arctan2(-k1, k2)
    c1 = np.cos(t1)
    s1 = np.sin(t1)

    dxc1_dys1 = dx * c1 + dy * s1

    t2 = np.arctan2(-dz, dxc1_dys1)
    c2 = np.cos(t2)
    s2 = np.sin(t2)

    c345 = r21 * c1 - r11 * s1

    s2n = np.abs(s2)
    if s2n > 1e-6:
        s345 = r31 / s2
        k3   = -dz / s2
    else:
        s345 = -(r11 * c1 + r21 * s1) / c2
        k3   = dxc1_dys1 / c2

    k4 = dy * c1 - dx * s1 - d2
    c4 = (k3 ** 2. + k4 ** 2. - a3 ** 2. - a4 ** 2.) / 2. / a3 / a4
    if c4 > 1.:
        c4 = 1.
    elif c4 < -1.:
        c4 = -1.
    s4 = -np.sqrt(1 - c4 ** 2.)
    t4 = np.arctan2(s4, c4)

    a34 = a3 + a4 * c4
    as4 = a4 * s4
    t3  = np.arctan2(a34 * k4 - as4 * k3, a34 * k3 + as4 * k4)
    t5  = np.arctan2(s345, c345) - t3 - t4
    return t1, t2, t3, t4, t5


@cc.export('legFK', '(f8, f8, f8, f8, f8,'
                    ' f8, f8, f8, f8, f8,'
                    ' f8, f8, f8, f8, f8,'
                    ' f8, f8, f8, f8, f8)')
def legFK(r1, r2, r3, r4, r5,
          l1, l2, l3, l4, l5,
          rd1, rd2, rd3, rd4, rd5,
          ld1, ld2, ld3, ld4, ld5):
    # for both toe and heel of both legs
    dqr = np.array([rd1, rd2, rd3, rd4, rd5])
    dql = np.array([ld1, ld2, ld3, ld4, ld5])
    
    # RIGHT
    t1, t2, t3, t4, t5 = r1, r2, r3, r4, r5
    td1, td2, td3, td4, td5 = rd1, rd2, rd3, rd4, rd5
    
    s1 = np.sin(t1)
    c1 = np.cos(t1)
    s2 = np.sin(t2)
    c2 = np.cos(t2)

    s1s2 = s1 * s2
    c1s2 = c1 * s2
    c1c2 = c1 * c2
    s1c2 = s1 * c2
    t34  = t3 + t4
    t345 = t34 + t5
    s345 = np.sin(t345)
    c345 = np.cos(t345)

    a5s345 = a5 * s345
    a5c345 = a5 * c345
    a6s345 = a6 * s345
    a6c345 = a6 * c345

    a4s34 = a4 * np.sin(t34)
    a4c34 = a4 * np.cos(t34)
    a3s3  = a3 * np.sin(t3)
    a3c3  = a3 * np.cos(t3)

    # Ankle
    as34a   = a3s3 + a4s34
    ac34a   = a3c3 + a4c34
    d2as34a = d2 + as34a

    s2ac34a   =   s2 * ac34a
    s1c2ac34a = s1c2 * ac34a
    c1c2ac34a = c1c2 * ac34a

    c1c2ac34_s1d2as34a = c1c2ac34a - s1 * d2as34a
    s1c2ac34_c1d2as34a = s1c2ac34a + c1 * d2as34a

    # position
    pra = np.array([hx + c1c2ac34_s1d2as34a,
                   -hy + s1c2ac34_c1d2as34a,
                   -hz - s2ac34a])

    # Jacobian
    Jra = np.array([[-s1c2ac34_c1d2as34a, -c1s2 * ac34a, -s1 * ac34a - c1c2 * as34a, -s1 * a4c34 - c1c2 * a4s34, 0.],
                    [ c1c2ac34_s1d2as34a, -s1s2 * ac34a,  c1 * ac34a - s1c2 * as34a,  c1 * a4c34 - s1c2 * a4s34, 0.],
                    [                 0.,   -c2 * ac34a,                 s2 * as34a,                 s2 * a4s34, 0.]])
    vra = Jra @ dqr

    # Jacobian Derivative
    dJra = np.zeros((3, 5))
    dJra[0, 0] = -Jra[1, 0] * td1 - Jra[1, 1] * td2 - Jra[1, 2] * td3 - Jra[1, 3] * td4
    dJra[1, 0] =  Jra[0, 0] * td1 + Jra[0, 1] * td2 + Jra[0, 2] * td3 + Jra[0, 3] * td4

    astd34a = as34a * td3 + a4s34 * td4
    s2ac4a  = s2 * a4c34
    dJra[0, 1] = -Jra[1, 1] * td1 - c1c2ac34a * td2 + c1s2 * astd34a
    dJra[1, 1] =  Jra[0, 1] * td1 - s1c2ac34a * td2 + s1s2 * astd34a
    dJra[2, 1] =                      s2ac34a * td2 +   c2 * astd34a

    s1a4a = s1 * a4s34 - c1c2 * a4c34
    c1a4a = c1 * a4s34 + s1c2 * a4c34
    as34atd2 = as34a * td2
    dJra[0, 2] = -Jra[1, 2] * td1 + c1s2 * as34atd2 - (Jra[1, 0] + s1 * d2) * td3 +  s1a4a * td4
    dJra[1, 2] =  Jra[0, 2] * td1 + s1s2 * as34atd2 + (Jra[0, 0] + c1 * d2) * td3 -  c1a4a * td4
    dJra[2, 2] =                      c2 * as34atd2 +               s2ac34a * td3 + s2ac4a * td4

    td34 = td3 + td4
    as4atd2 = a4s34 * td2
    dJra[0, 3] = -Jra[1, 3] * td1 + c1s2 * as4atd2 +  s1a4a * td34
    dJra[1, 3] =  Jra[0, 3] * td1 + s1s2 * as4atd2 -  c1a4a * td34
    dJra[2, 3] =                      c2 * as4atd2 + s2ac4a * td34

    # Toe
    as56t   = a5s345 + a6c345 + (at - ah) * c345
    ac56t   = a5c345 - a6s345 - (at - ah) * s345
    as456t  = as56t  + a4s34
    ac456t  = ac56t  + a4c34
    as3456t = as456t + a3s3
    ac3456t = ac456t + a3c3
    d2as3456t = d2 + as3456t

    s2ac3456t   =   s2 * ac3456t
    s1c2ac3456t = s1c2 * ac3456t
    c1c2ac3456t = c1c2 * ac3456t

    c1c2ac3456_s1d2as3456t = c1c2ac3456t - s1 * d2as3456t
    s1c2ac3456_c1d2as3456t = s1c2ac3456t + c1 * d2as3456t

    # position
    prt = np.array([hx + c1c2ac3456_s1d2as3456t,
                   -hy + s1c2ac3456_c1d2as3456t,
                   -hz - s2ac3456t])

    # Jacobian
    Jrt = np.array([[-s1c2ac3456_c1d2as3456t, -c1s2 * ac3456t, -s1 * ac3456t - c1c2 * as3456t, -s1 * ac456t - c1c2 * as456t, -s1 * ac56t - c1c2 * as56t],
                    [ c1c2ac3456_s1d2as3456t, -s1s2 * ac3456t,  c1 * ac3456t - s1c2 * as3456t,  c1 * ac456t - s1c2 * as456t,  c1 * ac56t - s1c2 * as56t],
                    [                     0.,   -c2 * ac3456t,                   s2 * as3456t,                  s2 * as456t,                 s2 * as56t]])
    vrt = Jrt @ dqr

    # Jacobian Derivative
    dJrt       = np.zeros((3, 5))
    dJrt[0, 0] = -Jrt[1, 0] * td1 - Jrt[1, 1] * td2 - Jrt[1, 2] * td3 - Jrt[1, 3] * td4 - Jrt[1, 4] * td5
    dJrt[1, 0] =  Jrt[0, 0] * td1 + Jrt[0, 1] * td2 + Jrt[0, 2] * td3 + Jrt[0, 3] * td4 + Jrt[0, 4] * td5

    astd345t = as3456t * td3 + as456t * td4 + as56t * td5
    s2ac456t = s2 * ac456t
    s2ac56t  = s2 * ac56t
    dJrt[0, 1] = -Jrt[1, 1] * td1 - c1c2ac3456t * td2 + c1s2 * astd345t
    dJrt[1, 1] =  Jrt[0, 1] * td1 - s1c2ac3456t * td2 + s1s2 * astd345t
    dJrt[2, 1] =                      s2ac3456t * td2 +   c2 * astd345t

    s1a456t = s1 * as456t - c1c2 * ac456t
    c1a456t = c1 * as456t + s1c2 * ac456t
    s1a56t  = s1 *  as56t - c1c2 *  ac56t
    c1a56t  = c1 *  as56t + s1c2 *  ac56t
    as3456ttd2 = as3456t * td2
    s1a56ttd5  =  s1a56t * td5
    c1a56ttd5  =  c1a56t * td5
    s2ac56ttd5 = s2ac56t * td5
    dJrt[0, 2] = -Jrt[1, 2] * td1 + c1s2 * as3456ttd2 - (Jrt[1, 0] + s1 * d2) * td3 +  s1a456t * td4 +  s1a56ttd5
    dJrt[1, 2] =  Jrt[0, 2] * td1 + s1s2 * as3456ttd2 + (Jrt[0, 0] + c1 * d2) * td3 -  c1a456t * td4 -  c1a56ttd5
    dJrt[2, 2] =                      c2 * as3456ttd2 +             s2ac3456t * td3 + s2ac456t * td4 + s2ac56ttd5

    # td34 = td3 + td4
    as456ttd2 = as456t * td2
    dJrt[0, 3] = -Jrt[1, 3] * td1 + c1s2 * as456ttd2 +  s1a456t * td34 +  s1a56ttd5
    dJrt[1, 3] =  Jrt[0, 3] * td1 + s1s2 * as456ttd2 -  c1a456t * td34 -  c1a56ttd5
    dJrt[2, 3] =                      c2 * as456ttd2 + s2ac456t * td34 + s2ac56ttd5

    td345 = td34 + td5
    as56ttd2 = as56t * td2
    dJrt[0, 4] = -Jrt[1, 4] * td1 + c1s2 * as56ttd2 +  s1a56t * td345
    dJrt[1, 4] =  Jrt[0, 4] * td1 + s1s2 * as56ttd2 -  c1a56t * td345
    dJrt[2, 4] =                      c2 * as56ttd2 + s2ac56t * td345

    # Heel
    as56h   = a5s345 - a6c345
    ac56h   = a5c345 + a6s345
    as456h  = as56h  + a4s34
    ac456h  = ac56h  + a4c34
    as3456h = as456h + a3s3
    ac3456h = ac456h + a3c3
    d2as3456h = d2 + as3456h

    s2ac3456h   =   s2 * ac3456h
    s1c2ac3456h = s1c2 * ac3456h
    c1c2ac3456h = c1c2 * ac3456h

    c1c2ac3456_s1d2as3456h = c1c2ac3456h - s1 * d2as3456h
    s1c2ac3456_c1d2as3456h = s1c2ac3456h + c1 * d2as3456h

    # position
    prh = np.array([hx + c1c2ac3456_s1d2as3456h,
                   -hy + s1c2ac3456_c1d2as3456h,
                   -hz - s2ac3456h])

    # Jacobian
    Jrh = np.array([[-s1c2ac3456_c1d2as3456h, -c1s2 * ac3456h, -s1 * ac3456h - c1c2 * as3456h, -s1 * ac456h - c1c2 * as456h, -s1 * ac56h - c1c2 * as56h],
                    [ c1c2ac3456_s1d2as3456h, -s1s2 * ac3456h,  c1 * ac3456h - s1c2 * as3456h,  c1 * ac456h - s1c2 * as456h,  c1 * ac56h - s1c2 * as56h],
                    [                     0.,   -c2 * ac3456h,                   s2 * as3456h,                  s2 * as456h,                 s2 * as56h]])
    vrh = Jrh @ dqr

    # Jacobian Derivative
    dJrh       = np.zeros((3, 5))
    dJrh[0, 0] = -Jrh[1, 0] * td1 - Jrh[1, 1] * td2 - Jrh[1, 2] * td3 - Jrh[1, 3] * td4 - Jrh[1, 4] * td5
    dJrh[1, 0] =  Jrh[0, 0] * td1 + Jrh[0, 1] * td2 + Jrh[0, 2] * td3 + Jrh[0, 3] * td4 + Jrh[0, 4] * td5

    astd345h = as3456h * td3 + as456h * td4 + as56h * td5
    s2ac456h = s2 * ac456h
    s2ac56h  = s2 *  ac56h
    dJrh[0, 1] = -Jrh[1, 1] * td1 - c1c2ac3456h * td2 + c1s2 * astd345h
    dJrh[1, 1] =  Jrh[0, 1] * td1 - s1c2ac3456h * td2 + s1s2 * astd345h
    dJrh[2, 1] =                      s2ac3456h * td2 +   c2 * astd345h

    s1a456h = s1 * as456h - c1c2 * ac456h
    c1a456h = c1 * as456h + s1c2 * ac456h
    s1a56h  = s1 *  as56h - c1c2 *  ac56h
    c1a56h  = c1 *  as56h + s1c2 *  ac56h
    as3456htd2 = as3456h * td2
    s1a56htd5  =  s1a56h * td5
    c1a56htd5  =  c1a56h * td5
    s2ac56htd5 = s2ac56h * td5
    dJrh[0, 2] = -Jrh[1, 2] * td1 + c1s2 * as3456htd2 - (Jrh[1, 0] + s1 * d2) * td3 +  s1a456h * td4 +  s1a56htd5
    dJrh[1, 2] =  Jrh[0, 2] * td1 + s1s2 * as3456htd2 + (Jrh[0, 0] + c1 * d2) * td3 -  c1a456h * td4 -  c1a56htd5
    dJrh[2, 2] =                      c2 * as3456htd2 +             s2ac3456h * td3 + s2ac456h * td4 + s2ac56htd5

    as456td2 = as456h * td2
    dJrh[0, 3] = -Jrh[1, 3] * td1 + c1s2 * as456td2 +  s1a456h * td34 +  s1a56htd5
    dJrh[1, 3] =  Jrh[0, 3] * td1 + s1s2 * as456td2 -  c1a456h * td34 -  c1a56htd5
    dJrh[2, 3] =                      c2 * as456td2 + s2ac456h * td34 + s2ac56htd5

    as56td2 = as56h * td2
    dJrh[0, 4] = -Jrh[1, 4] * td1 + c1s2 * as56td2 +  s1a56h * td345
    dJrh[1, 4] =  Jrh[0, 4] * td1 + s1s2 * as56td2 -  c1a56h * td345
    dJrh[2, 4] =                      c2 * as56td2 + s2ac56h * td345

    # Middle
    prm = (prt + prh) / 2.
    vrm = (vrt + vrh) / 2.

    # orientation
    Rr = np.array([[-c1c2 * s345 - s1 * c345, -c1s2, -c1c2 * c345 + s1 * s345],
                   [-s1c2 * s345 + c1 * c345, -s1s2, -s1c2 * c345 - c1 * s345],
                   [               s2 * s345,   -c2,                s2 * c345]])

    # rotational Jacobian
    Jwr = np.array([[0., -s1, c1s2, c1s2, c1s2],
                    [0.,  c1, s1s2, s1s2, s1s2],
                    [1.,  0.,   c2,   c2,   c2]])

    # rotational Jacobian Derivative
    dJwr = np.zeros((3, 5))
    dJwr[0, 1] = -c1 * td1
    dJwr[1, 1] = -s1 * td1

    dJwr[0, 2] = -s1s2 * td1 + c1c2 * td2
    dJwr[1, 2] =  c1s2 * td1 + s1c2 * td2
    dJwr[2, 2] =                -s2 * td2

    dJwr[0:3, 3] = dJwr[0:3, 2]
    dJwr[0:3, 4] = dJwr[0:3, 2]

    # LEFT
    t1, t2, t3, t4, t5 = l1, l2, l3, l4, l5
    td1, td2, td3, td4, td5 = ld1, ld2, ld3, ld4, ld5

    s1 = np.sin(t1)
    c1 = np.cos(t1)
    s2 = np.sin(t2)
    c2 = np.cos(t2)

    s1s2 = s1 * s2
    c1s2 = c1 * s2
    c1c2 = c1 * c2
    s1c2 = s1 * c2
    t34  = t3 + t4
    t345 = t34 + t5
    s345 = np.sin(t345)
    c345 = np.cos(t345)

    a5s345 = a5 * s345
    a5c345 = a5 * c345
    a6s345 = a6 * s345
    a6c345 = a6 * c345

    a4s34 = a4 * np.sin(t34)
    a4c34 = a4 * np.cos(t34)
    a3s3  = a3 * np.sin(t3)
    a3c3  = a3 * np.cos(t3)

    # Ankle
    as34a = a3s3 + a4s34
    ac34a = a3c3 + a4c34
    d2as34a = d2 + as34a

    s2ac34a   =   s2 * ac34a
    s1c2ac34a = s1c2 * ac34a
    c1c2ac34a = c1c2 * ac34a

    c1c2ac34_s1d2as34a = c1c2ac34a - s1 * d2as34a
    s1c2ac34_c1d2as34a = s1c2ac34a + c1 * d2as34a

    # position
    pla = np.array([hx + c1c2ac34_s1d2as34a,
                    hy + s1c2ac34_c1d2as34a,
                   -hz - s2ac34a])

    # Jacobian
    Jla = np.array([[-s1c2ac34_c1d2as34a, -c1s2 * ac34a, -s1 * ac34a - c1c2 * as34a, -s1 * a4c34 - c1c2 * a4s34, 0.],
                    [ c1c2ac34_s1d2as34a, -s1s2 * ac34a,  c1 * ac34a - s1c2 * as34a,  c1 * a4c34 - s1c2 * a4s34, 0.],
                    [                 0.,   -c2 * ac34a,                 s2 * as34a,                 s2 * a4s34, 0.]])
    vla = Jla @ dql

    # Jacobian Derivative
    dJla = np.zeros((3, 5))
    dJla[0, 0] = -Jla[1, 0] * td1 - Jla[1, 1] * td2 - Jla[1, 2] * td3 - Jla[1, 3] * td4
    dJla[1, 0] =  Jla[0, 0] * td1 + Jla[0, 1] * td2 + Jla[0, 2] * td3 + Jla[0, 3] * td4

    astd34a = as34a * td3 + a4s34 * td4
    s2ac4a  = s2 * a4c34
    dJla[0, 1] = -Jla[1, 1] * td1 - c1c2ac34a * td2 + c1s2 * astd34a
    dJla[1, 1] =  Jla[0, 1] * td1 - s1c2ac34a * td2 + s1s2 * astd34a
    dJla[2, 1] =                      s2ac34a * td2 +   c2 * astd34a

    s1a4a = s1 * a4s34 - c1c2 * a4c34
    c1a4a = c1 * a4s34 + s1c2 * a4c34
    as34atd2 = as34a * td2
    dJla[0, 2] = -Jla[1, 2] * td1 + c1s2 * as34atd2 - (Jla[1, 0] + s1 * d2) * td3 +  s1a4a * td4
    dJla[1, 2] =  Jla[0, 2] * td1 + s1s2 * as34atd2 + (Jla[0, 0] + c1 * d2) * td3 -  c1a4a * td4
    dJla[2, 2] =                      c2 * as34atd2 +               s2ac34a * td3 + s2ac4a * td4

    td34 = td3 + td4
    as4atd2 = a4s34 * td2
    dJla[0, 3] = -Jla[1, 3] * td1 + c1s2 * as4atd2 +  s1a4a * td34
    dJla[1, 3] =  Jla[0, 3] * td1 + s1s2 * as4atd2 -  c1a4a * td34
    dJla[2, 3] =                      c2 * as4atd2 + s2ac4a * td34

    # Toe
    as56t   = a5s345 + a6c345 + (at - ah) * c345
    ac56t   = a5c345 - a6s345 - (at - ah) * s345
    as456t  = as56t  + a4s34
    ac456t  = ac56t  + a4c34
    as3456t = as456t + a3s3
    ac3456t = ac456t + a3c3
    d2as3456t = d2 + as3456t

    s2ac3456t   = s2   * ac3456t
    s1c2ac3456t = s1c2 * ac3456t
    c1c2ac3456t = c1c2 * ac3456t

    c1c2ac3456_s1d2as3456t = c1c2ac3456t - s1 * d2as3456t
    s1c2ac3456_c1d2as3456t = s1c2ac3456t + c1 * d2as3456t

    # position
    plt = np.array([hx + c1c2ac3456_s1d2as3456t,
                    hy + s1c2ac3456_c1d2as3456t,
                   -hz - s2ac3456t])

    # Jacobian
    Jlt = np.array([[-s1c2ac3456_c1d2as3456t, -c1s2 * ac3456t, -s1 * ac3456t - c1c2 * as3456t, -s1 * ac456t - c1c2 * as456t, -s1 * ac56t - c1c2 * as56t],
                    [ c1c2ac3456_s1d2as3456t, -s1s2 * ac3456t,  c1 * ac3456t - s1c2 * as3456t,  c1 * ac456t - s1c2 * as456t,  c1 * ac56t - s1c2 * as56t],
                    [                     0.,   -c2 * ac3456t,                   s2 * as3456t,                  s2 * as456t,                 s2 * as56t]])
    vlt = Jlt @ dql

    # Jacobian Derivative
    dJlt       = np.zeros((3, 5))
    dJlt[0, 0] = -Jlt[1, 0] * td1 - Jlt[1, 1] * td2 - Jlt[1, 2] * td3 - Jlt[1, 3] * td4 - Jlt[1, 4] * td5
    dJlt[1, 0] =  Jlt[0, 0] * td1 + Jlt[0, 1] * td2 + Jlt[0, 2] * td3 + Jlt[0, 3] * td4 + Jlt[0, 4] * td5

    astd345t = as3456t * td3 + as456t * td4 + as56t * td5
    s2ac456t = s2 * ac456t
    s2ac56t  = s2 * ac56t
    dJlt[0, 1] = -Jlt[1, 1] * td1 - c1c2ac3456t * td2 + c1s2 * astd345t
    dJlt[1, 1] =  Jlt[0, 1] * td1 - s1c2ac3456t * td2 + s1s2 * astd345t
    dJlt[2, 1] =                      s2ac3456t * td2 +   c2 * astd345t

    s1a456t = s1 * as456t - c1c2 * ac456t
    c1a456t = c1 * as456t + s1c2 * ac456t
    s1a56t  = s1 *  as56t - c1c2 *  ac56t
    c1a56t  = c1 *  as56t + s1c2 *  ac56t
    as3456ttd2 = as3456t * td2
    s1a56ttd5  =  s1a56t * td5
    c1a56ttd5  =  c1a56t * td5
    s2ac56ttd5 = s2ac56t * td5
    dJlt[0, 2] = -Jlt[1, 2] * td1 + c1s2 * as3456ttd2 - (Jlt[1, 0] + s1 * d2) * td3 +  s1a456t * td4 +  s1a56ttd5
    dJlt[1, 2] =  Jlt[0, 2] * td1 + s1s2 * as3456ttd2 + (Jlt[0, 0] + c1 * d2) * td3 -  c1a456t * td4 -  c1a56ttd5
    dJlt[2, 2] =                      c2 * as3456ttd2 +             s2ac3456t * td3 + s2ac456t * td4 + s2ac56ttd5

    td34 = td3 + td4
    as456ttd2 = as456t * td2
    dJlt[0, 3] = -Jlt[1, 3] * td1 + c1s2 * as456ttd2 +  s1a456t * td34 +  s1a56ttd5
    dJlt[1, 3] =  Jlt[0, 3] * td1 + s1s2 * as456ttd2 -  c1a456t * td34 -  c1a56ttd5
    dJlt[2, 3] =                      c2 * as456ttd2 + s2ac456t * td34 + s2ac56ttd5

    td345 = td34 + td5
    as56ttd2 = as56t * td2
    dJlt[0, 4] = -Jlt[1, 4] * td1 + c1s2 * as56ttd2 +  s1a56t * td345
    dJlt[1, 4] =  Jlt[0, 4] * td1 + s1s2 * as56ttd2 -  c1a56t * td345
    dJlt[2, 4] =                      c2 * as56ttd2 + s2ac56t * td345

    # Heel
    as56h   = a5s345 - a6c345
    ac56h   = a5c345 + a6s345
    as456h  = as56h  + a4s34
    ac456h  = ac56h  + a4c34
    as3456h = as456h + a3s3
    ac3456h = ac456h + a3c3
    d2as3456h = d2 + as3456h

    s2ac3456h   = s2   * ac3456h
    s1c2ac3456h = s1c2 * ac3456h
    c1c2ac3456h = c1c2 * ac3456h

    c1c2ac3456_s1d2as3456h = c1c2ac3456h - s1 * d2as3456h
    s1c2ac3456_c1d2as3456h = s1c2ac3456h + c1 * d2as3456h

    # position
    plh = np.array([hx + c1c2ac3456_s1d2as3456h,
                    hy + s1c2ac3456_c1d2as3456h,
                   -hz - s2ac3456h])

    # Jacobian
    Jlh = np.array([[-s1c2ac3456_c1d2as3456h, -c1s2 * ac3456h, -s1 * ac3456h - c1c2 * as3456h, -s1 * ac456h - c1c2 * as456h, -s1 * ac56h - c1c2 * as56h],
                    [ c1c2ac3456_s1d2as3456h, -s1s2 * ac3456h,  c1 * ac3456h - s1c2 * as3456h,  c1 * ac456h - s1c2 * as456h,  c1 * ac56h - s1c2 * as56h],
                    [                     0.,   -c2 * ac3456h,                   s2 * as3456h,                  s2 * as456h,                 s2 * as56h]])
    vlh = Jlh @ dql

    # Jacobian Derivative
    dJlh       = np.zeros((3, 5))
    dJlh[0, 0] = -Jlh[1, 0] * td1 - Jlh[1, 1] * td2 - Jlh[1, 2] * td3 - Jlh[1, 3] * td4 - Jlh[1, 4] * td5
    dJlh[1, 0] =  Jlh[0, 0] * td1 + Jlh[0, 1] * td2 + Jlh[0, 2] * td3 + Jlh[0, 3] * td4 + Jlh[0, 4] * td5

    astd345h = as3456h * td3 + as456h * td4 + as56h * td5
    s2ac456h = s2 * ac456h
    s2ac56h  = s2 *  ac56h
    dJlh[0, 1] = -Jlh[1, 1] * td1 - c1c2ac3456h * td2 + c1s2 * astd345h
    dJlh[1, 1] =  Jlh[0, 1] * td1 - s1c2ac3456h * td2 + s1s2 * astd345h
    dJlh[2, 1] =                      s2ac3456h * td2 +   c2 * astd345h

    s1a456h = s1 * as456h - c1c2 * ac456h
    c1a456h = c1 * as456h + s1c2 * ac456h
    s1a56h  = s1 *  as56h - c1c2 *  ac56h
    c1a56h  = c1 *  as56h + s1c2 *  ac56h
    as3456htd2 = as3456h * td2
    s1a56htd5  =  s1a56h * td5
    c1a56htd5  =  c1a56h * td5
    s2ac56htd5 = s2ac56h * td5
    dJlh[0, 2] = -Jlh[1, 2] * td1 + c1s2 * as3456htd2 - (Jlh[1, 0] + s1 * d2) * td3 +  s1a456h * td4 +  s1a56htd5
    dJlh[1, 2] =  Jlh[0, 2] * td1 + s1s2 * as3456htd2 + (Jlh[0, 0] + c1 * d2) * td3 -  c1a456h * td4 -  c1a56htd5
    dJlh[2, 2] =                      c2 * as3456htd2 +             s2ac3456h * td3 + s2ac456h * td4 + s2ac56htd5

    as456td2 = as456h * td2
    dJlh[0, 3] = -Jlh[1, 3] * td1 + c1s2 * as456td2 +  s1a456h * td34 +  s1a56htd5
    dJlh[1, 3] =  Jlh[0, 3] * td1 + s1s2 * as456td2 -  c1a456h * td34 -  c1a56htd5
    dJlh[2, 3] =                      c2 * as456td2 + s2ac456h * td34 + s2ac56htd5

    as56td2 = as56h * td2
    dJlh[0, 4] = -Jlh[1, 4] * td1 + c1s2 * as56td2 +  s1a56h * td345
    dJlh[1, 4] =  Jlh[0, 4] * td1 + s1s2 * as56td2 -  c1a56h * td345
    dJlh[2, 4] =                      c2 * as56td2 + s2ac56h * td345

    # Middle
    plm = (plt + plh) / 2.
    vlm = (vlt + vlh) / 2.

    # orientation
    Rl = np.array([[-c1c2 * s345 - s1 * c345, -c1s2, -c1c2 * c345 + s1 * s345],
                   [-s1c2 * s345 + c1 * c345, -s1s2, -s1c2 * c345 - c1 * s345],
                   [               s2 * s345,   -c2,                s2 * c345]])

    # rotational Jacobian
    Jwl = np.array([[0., -s1, c1s2, c1s2, c1s2],
                    [0.,  c1, s1s2, s1s2, s1s2],
                    [1.,  0.,   c2,   c2,   c2]])

    # rotational Jacobian Derivative
    dJwl = np.zeros((3, 5))
    dJwl[0, 1] = -c1 * td1
    dJwl[1, 1] = -s1 * td1

    dJwl[0, 2] = -s1s2 * td1 + c1c2 * td2
    dJwl[1, 2] =  c1s2 * td1 + s1c2 * td2
    dJwl[2, 2] =                -s2 * td2

    dJwl[0:3, 3] = dJwl[0:3, 2]
    dJwl[0:3, 4] = dJwl[0:3, 2]

    return prt, vrt, Jrt, dJrt, \
           prh, vrh, Jrh, dJrh, \
           pra, vra, Jra, dJra, \
           prm, vrm, Rr, Jwr, dJwr, \
           plt, vlt, Jlt, dJlt, \
           plh, vlh, Jlh, dJlh, \
           pla, vla, Jla, dJla, \
           plm, vlm, Rl, Jwl, dJwl


@cc.export('robotFK', '(f8[:,:], f8[:], f8[:], f8[:],'
                      'f8[:], f8[:,:], f8[:,:],'
                      'f8[:], f8[:,:], f8[:,:],'
                      'f8[:], f8[:,:], f8[:,:], f8[:,:], f8[:,:], f8[:,:],'
                      'f8[:], f8[:,:], f8[:,:],'
                      'f8[:], f8[:,:], f8[:,:],'
                      'f8[:], f8[:,:], f8[:,:], f8[:,:], f8[:,:], f8[:,:],'
                      'f8, f8, f8, f8, f8,'
                      'f8, f8, f8, f8, f8)')
def robotFK(R, p, w, bv,
            prt, Jrt, dJrt,
            prh, Jrh, dJrh,
            pra, Jra, dJra, Rr, Jwr, dJwr,
            plt, Jlt, dJlt,
            plh, Jlh, dJlh,
            pla, Jla, dJla, Rl, Jwl, dJwl,
            rd1, rd2, rd3, rd4, rd5,
            ld1, ld2, ld3, ld4, ld5):
    R = np.copy(R)

    # foot position in world frame
    prt = np.copy(prt)
    prh = np.copy(prh)
    pra = np.copy(pra)
    plt = np.copy(plt)
    plh = np.copy(plh)
    pla = np.copy(pla)

    crt = p + R @ prt
    crh = p + R @ prh
    cra = p + R @ pra
    clt = p + R @ plt
    clh = p + R @ plh
    cla = p + R @ pla

    crm = (crt + crh) / 2.
    clm = (clt + clh) / 2.

    # foot orientation in world frame
    Rr = np.copy(Rr)
    Rl = np.copy(Rl)

    wRr = R @ Rr
    wRl = R @ Rl

    # robot Jacobians in world frame
    wJrt = np.zeros((3, 16))
    wJrh = np.zeros((3, 16))
    wJra = np.zeros((3, 16))
    wJlt = np.zeros((3, 16))
    wJlh = np.zeros((3, 16))
    wJla = np.zeros((3, 16))

    Jrt = np.copy(Jrt)
    Jrh = np.copy(Jrh)
    Jra = np.copy(Jra)
    Jlt = np.copy(Jlt)
    Jlh = np.copy(Jlh)
    Jla = np.copy(Jla)

    wJrt[ 0:3, 0:3] = -R @ MF.hat(prt)
    wJrt[ 0:3, 3:6] =  R
    wJrt[0:3, 6:11] =  R @ Jrt

    wJrh[ 0:3, 0:3] = -R @ MF.hat(prh)
    wJrh[ 0:3, 3:6] =  R
    wJrh[0:3, 6:11] =  R @ Jrh

    wJra[0:3, 0:3]  = -R @ MF.hat(pra)
    wJra[0:3, 3:6]  =  R
    wJra[0:3, 6:11] =  R @ Jra

    wJlt[0:3,   0:3] = -R @ MF.hat(plt)
    wJlt[0:3,   3:6] =  R
    wJlt[0:3, 11:16] =  R @ Jlt

    wJlh[0:3,   0:3] = -R @ MF.hat(plh)
    wJlh[0:3,   3:6] =  R
    wJlh[0:3, 11:16] =  R @ Jlh

    wJla[0:3,   0:3] = -R @ MF.hat(pla)
    wJla[0:3,   3:6] =  R
    wJla[0:3, 11:16] =  R @ Jla

    # robot rotational Jacobians in foot frame
    wJwr = np.zeros((3, 16))
    wJwl = np.zeros((3, 16))

    Jwr = np.copy(Jwr)
    Jwl = np.copy(Jwl)

    RrT = Rr.T
    wJwr[0:3,  0:3] = RrT
    wJwr[0:3, 6:11] = RrT @ Jwr

    RlT = Rl.T
    wJwl[0:3,   0:3] = RlT
    wJwl[0:3, 11:16] = RlT @ Jwl

    dqr = np.array([rd1, rd2, rd3, rd4, rd5])
    dql = np.array([ld1, ld2, ld3, ld4, ld5])

    dqAll = np.hstack((w, bv, dqr, dql))

    # foot velocity in world frame
    dcrt = wJrt @ dqAll
    dcrh = wJrh @ dqAll
    dcra = wJra @ dqAll
    dclt = wJlt @ dqAll
    dclh = wJlh @ dqAll
    dcla = wJla @ dqAll

    dcrm = (dcrt + dcrh) / 2.
    dclm = (dclt + dclh) / 2.

    # foot angular rate in foot frame
    wr = wJwr @ dqAll
    wl = wJwl @ dqAll

    # dJdq
    what = MF.hat(w)
    bv   = np.copy(bv)

    whatwhat = what @ what
    whatbv   = what @ bv
    what2    = 2 * what

    wdJrtdq = R @ (whatbv + whatwhat @ prt + (what2 @ Jrt + dJrt) @ dqr)
    wdJrhdq = R @ (whatbv + whatwhat @ prh + (what2 @ Jrh + dJrh) @ dqr)
    wdJradq = R @ (whatbv + whatwhat @ pra + (what2 @ Jra + dJra) @ dqr)

    wdJltdq = R @ (whatbv + whatwhat @ plt + (what2 @ Jlt + dJlt) @ dql)
    wdJlhdq = R @ (whatbv + whatwhat @ plh + (what2 @ Jlh + dJlh) @ dql)
    wdJladq = R @ (whatbv + whatwhat @ pla + (what2 @ Jla + dJla) @ dql)

    wdJwrdq = RrT @ (what @ Jwr + dJwr) @ dqr
    wdJwldq = RlT @ (what @ Jwl + dJwl) @ dql

    return crt, dcrt, wJrt, wdJrtdq, \
           crh, dcrh, wJrh, wdJrhdq, \
           cra, dcra, wJra, wdJradq, \
           crm, dcrm, wRr, wr, wJwr, wdJwrdq, \
           clt, dclt, wJlt, wdJltdq, \
           clh, dclh, wJlh, wdJlhdq, \
           cla, dcla, wJla, wdJladq, \
           clm, dclm, wRl, wl, wJwl, wdJwldq


if __name__ == '__main__':
    cc.compile()
