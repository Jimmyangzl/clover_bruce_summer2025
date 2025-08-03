#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

"""
Script that holds useful macros for demo
"""

from collections import defaultdict
from Settings.BRUCE_macros import *

# MODE
BALANCE = 0
WALK    = 1
RUN     = 2

# HIGH-LEVEL
# velocity adjustment in moving [m]
bx_offset = {WALK: 0.000, RUN: 0.000}
by_offset = {WALK: 0.000, RUN: 0.000}

# CoM position adjustment in standing [m]
px_offset = -0.005

# TOP-LEVEL
# velocity limit in moving [m]
COM_VELOCITY_LIMIT = {WALK: [+0.20, -0.20, +0.10, -0.10],  # [x_max, x_min, y_max, y_min]
                      RUN:  [+0.20, -0.20, +0.10, -0.10]}

COM_POSITION_X     = 0
COM_POSITION_Y     = 1
COM_POSITION_Z     = 2

BODY_ORIENTATION_X = 3
BODY_ORIENTATION_Y = 4
BODY_ORIENTATION_Z = 5

COM_VELOCITY_X     = 6
COM_VELOCITY_Y     = 7
BODY_YAW_RATE      = 8

FOOT_YAW_RIGHT     = 9
FOOT_YAW_LEFT      = 10
FOOT_CLEARANCE     = 11

COOLING_SPEED      = 12

PARAMETER_ID_LIST      = range(13)
PARAMETER_INCREMENT    = [ 0.002,  0.002,  0.002,       1,     1,     2,    0.01,  0.01,     1,       1,     1,   0.01,       1]
PARAMETER_DEFAULT      = [ 0.000,  0.000,  0.000,       0,     0,     0,     0.0,   0.0,     0,       0,     0,   0.00,       0]
PARAMETER_MAX          = [ 0.010,  0.020,  0.020,       8,    10,    20,    0.10,  0.10,    15,      10,    10,   0.05,       5]
PARAMETER_MIN          = [-0.010, -0.020, -0.030,      -8,   -10,   -20,   -0.10, -0.10,   -15,     -10,   -10,  -0.05,       0]
PARAMETER_BUTTON_PLUS  = [   'g',    'j',    'l',     'y',   'i',   'p',     'w',   'a',   'q',     'x',   'v',    'm',     '=']
PARAMETER_BUTTON_MINUS = [   'f',    'h',    'k',     't',   'u',   'o',     's',   'd',   'e',     'z',   'c',    'n',     '-']
PARAMETER_TYPE         = [ 'len',  'len',  'len',   'ang', 'ang', 'ang',   'len', 'len', 'ang',   'ang', 'ang',  'len',   'len']
PARAMETER_RECOVER      = [   'y',    'y',    'y',     'y',   'y',   'y',     'y',   'y',   'y',     'y',   'y',    'y',     'n']

PARAMETER_MODE_LIST = {COM_POSITION_X:     [BALANCE],
                       COM_POSITION_Y:     [BALANCE],
                       COM_POSITION_Z:     [BALANCE],
                       BODY_ORIENTATION_X: [BALANCE],
                       BODY_ORIENTATION_Y: [BALANCE],
                       BODY_ORIENTATION_Z: [BALANCE],
                       COM_VELOCITY_X:     [WALK, RUN],
                       COM_VELOCITY_Y:     [WALK, RUN],
                       BODY_YAW_RATE:      [WALK, RUN],
                       FOOT_YAW_RIGHT:     [WALK, RUN],
                       FOOT_YAW_LEFT:      [WALK, RUN],
                       FOOT_CLEARANCE:     [WALK, RUN],
                       COOLING_SPEED:      [BALANCE, WALK, RUN]
                       }

# wave trajectory
arm_position_nominal = np.array([-0.7,  1.3,  2.0, 
                                  0.7, -1.3, -2.0])
arm_position_goal    = np.array([0.0, -1.2, 0.0,
                                 0.0,  1.2, 0.0])
arm_trajectory = defaultdict()

for i in range(6):
    arm_trajectory[i] = np.linspace(arm_position_nominal[i], arm_position_goal[i], 20, endpoint=True)

traj_time = np.linspace(0, 2.75 * 2 * np.pi, 30)
for tdx in traj_time:
    arm_trajectory[1] = np.append(arm_trajectory[1], arm_position_goal[1] - 0.3 * np.sin(tdx))
    arm_trajectory[4] = np.append(arm_trajectory[4], arm_position_goal[4] + 0.3 * np.sin(tdx))

    for i in [0, 2, 3, 5]:
        arm_trajectory[i] = np.append(arm_trajectory[i], arm_position_goal[i])

for i in range(6):
    arm_trajectory[i] = np.append(arm_trajectory[i], np.linspace(arm_trajectory[i][-1], arm_position_nominal[i], 20, endpoint=True))
