#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script that holds useful macros for DCM walking tuning
'''

import numpy as np
from collections import defaultdict
from numpy import sin, cos
from scipy import optimize as optimize

# HIGH-LEVEL
# Walking
# foot swing trajectory
Ts     = 0.24   # desired stance phase duration [s]
Ts_min = 0.20   # minimum stance duration       [s]
Ts_max = 0.28   # maximum stance duration       [s]
T_buff = 0.05   # stop plan before T - T_buff   [s]

Txi = 0.00      # x stay before Txi
Txn = 0.05      # x go to nominal before Txn
Txf = 0.00      # x arrive before T - Txf

Tyi = 0.00      # y stay before Tyi
Tyn = 0.05      # y go to nominal before Tyn
Tyf = 0.00      # y arrive before T - Tyf

Tzm = 0.10      # desired swing apex time [s]
Tzf = 0.00      # z arrive before T - Tzf

zm_l = 0.035    # left  swing apex height [m]
zm_r = 0.035    # right swing apex height [m]

zf_l = -0.002   # left  swing final height [m]
zf_r = -0.002   # right swing final height [m]

hz = 0.365      # desired CoM height [m]

yaw_f_offset = 0.02  # foot yaw offset [rad]

# kinematic reachability [m]
lx  = 0.20      # max longitudinal step length
lyi = 0.04      # min lateral distance between feet
lyo = 0.20      # max lateral distance between feet

# velocity offset compensation [m]
bx_offset = +0.010  # set to negative if BRUCE tends to go forward
by_offset = +0.000  # set to negative if BRUCE tends to go left

# Stance
ka = -0.0       # x position of CoM from the center of foot, in scale of 1/2 foot length
                # ka = 1 puts CoM at the front tip of foot

# TOP-LEVEL
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
PARAMETER_INCREMENT    = [ 0.05,  0.05,  0.002,       1,     1,     2,    0.01,  0.01,     1,       1,     1,  0.01,       1]
PARAMETER_DEFAULT      = [ 0.00,  0.00,  0.000,       0,     0,     0,     0.0,   0.0,     0,       0,     0,  0.05,       0]
PARAMETER_MAX          = [ 0.20,  0.50,  0.020,       8,    10,    20,    0.10,  0.10,    15,      10,    10,  0.08,       5]
PARAMETER_MIN          = [-0.20, -0.50, -0.030,      -8,   -10,   -20,   -0.10, -0.10,   -15,     -10,   -10,  0.03,       0]
PARAMETER_BUTTON_PLUS  = [  'g',   'j',    'l',     'y',   'i',   'p',     'w',   'a',   'q',     'x',   'v',   'm',     '=']
PARAMETER_BUTTON_MINUS = [  'f',   'h',    'k',     't',   'u',   'o',     's',   'd',   'e',     'z',   'c',   'n',     '-']
PARAMETER_TYPE         = ['len', 'len',  'len',   'ang', 'ang', 'ang',   'len', 'len', 'ang',   'ang', 'ang', 'len',   'len']
PARAMETER_RECOVER      = [  'y',   'y',    'y',     'y',   'y',   'y',     'y',   'y',   'y',     'y',   'y',   'y',     'n']

BALANCE = 0
WALK    = 1
PARAMETER_MODE_LIST = {COM_POSITION_X:     [BALANCE],
                       COM_POSITION_Y:     [BALANCE],
                       COM_POSITION_Z:     [BALANCE],
                       BODY_ORIENTATION_X: [BALANCE],
                       BODY_ORIENTATION_Y: [BALANCE],
                       BODY_ORIENTATION_Z: [BALANCE],
                       COM_VELOCITY_X:     [WALK],
                       COM_VELOCITY_Y:     [WALK],
                       BODY_YAW_RATE:      [WALK],
                       FOOT_YAW_RIGHT:     [WALK],
                       FOOT_YAW_LEFT:      [WALK],
                       FOOT_CLEARANCE:     [WALK],
                       COOLING_SPEED:      [BALANCE, WALK]
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


######################################## Kinematics ########################################
def dh_transform(alpha, a, d, theta):   
    transform = np.array([
        [np.cos(theta), - np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), - np.sin(alpha), - np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0.0, 0.0, 0.0, 1.0],
    ])
    return transform

# Forward Kinematics
# thetas: joint angles in radians, e.g. [0.1, np.pi/2, -0.2]
# returns position of end-effector (3D array), e.g. np.array([0.1, 0.2, 0.3])
def arm_fk(thetas):
    theta1 = thetas[0]
    theta2 = thetas[1]
    theta3 = thetas[2]
    T1 = dh_transform(0, 0, 0, theta1)
    T2 = dh_transform(np.pi/2, 0.0, 0.0, (theta2+np.pi/2))
    T3 = dh_transform(np.pi/2, 0.11, 0.0, theta3)
    T4 = dh_transform(0, 0.13, 0.0, 0)
    pose_ee = T1 @ T2 @ T3 @ T4
    return pose_ee[:3,-1]

def pos_cost(q, pos_d):
    return pos_d - arm_fk(q)

# Inverse Kinematics (IK) solver
# pos_d: desired end-effector position, e.g. np.array([0.1, 0.2, 0.3])
# q0: initial guess for joint angles, e.g. [0.1, np.pi/2, -0.2]
# returns the joint angles that achieve the desired position, e.g. [0.1, np.pi/2, -0.2]
def arm_ik_solver(pos_d, q0):
    optimization_result = optimize.least_squares(
        fun=pos_cost, x0=q0,  args=(pos_d,), method='trf'
    )
    if optimization_result.success:
        q = optimization_result.x
        return q
    else:
        raise ValueError("IK solver failed to converge: " + optimization_result.message)
############################################################################################


####################################### Assignment 1 #######################################

# *************** TODO: Finish the function get_square_traj_joint_space *************** #
def get_square_traj_joint_space():
    square_q_nominal = np.array([-0.7, 1.3, 2.0, 0.7, -1.3, -2.0])
    square_trajectory = defaultdict()  

    # --- TODO: Define square corner positions for the square trajectory --- # 
    positions = [
        ...
    ]
    # ---------------------------------------------------------------------- #

    # compute IK
    q_goals = [square_q_nominal[:3]]  # initial position

    # --- TODO: Get the IK solutions for the four corner positions --- #
    for pos in positions:
        pass
    # ---------------------------------------------------------------- #

    # --- TODO: define start-end pairs for each segment of the trajectory --- #
    start_end_pairs = [
        (..., ...),
        ...,
    ]
    # ----------------------------------------------------------------------- #

    # --- TODO generate trajectory in joint space --- #
    for i in range(6):
        trajectory = np.array([])
        for start, end in start_end_pairs:
            pass # segment = ?
            trajectory = np.append(trajectory, segment)
        square_trajectory[i] = trajectory
    return square_trajectory
square_trajectory_joint_space = get_square_traj_joint_space()


# *************** TODO: Finish the function get_square_traj_cart_space *************** #
def get_square_traj_cart_space():
    square_q_nominal = np.array([-0.7, 1.3, 2.0, 0.7, -1.3, -2.0])
    square_trajectory = defaultdict()
    # NOTICE: nominal cartesian position, can be used later
    cart_nominal = arm_fk(square_q_nominal[:3])   

    # --- TODO: Define square corner positions for the square trajectory --- # 
    pass
    # ---------------------------------------------------------------------- #

    # --- TODO: define start-end pairs for each segment of the trajectory --- #
    pass
    # ----------------------------------------------------------------------- #

    # --- TODO: Interpolate in cartesian space --- #
    trajectory_cart = np.zeros((1, 3))
    for start, end in start_end_pairs:
        # Interpolate between start and end points in cartesian space
        pass
        # segment = ?
        # NOTICE: np.linspace can not deal with 3D arrays directly,
        # you need to use np.linspace for each dimension separately
        trajectory_cart = np.append(trajectory_cart, segment, axis=0)
    # -------------------------------------------- #

    # --- TODO: Solve IK --- #
    q_goals = [square_q_nominal] 
    for cart_point in trajectory_cart[1:]:
        pass
        q_goals.append(q_goal)
    # Format the output
    for i in range(6):
        trajectory = np.array([])
        for q_goal in q_goals:
            trajectory = np.append(trajectory, q_goal[i])
        square_trajectory[i] = trajectory
    return square_trajectory
square_trajectory_cart_space = get_square_traj_cart_space()

############################################################################################



####################################### Assignment 2 #######################################
def get_lay_arms_traj():
    q_nominal = np.array([-0.7, 1.3, 2.0, 0.7, -1.3, -2.0])
    # --- TODO: Design a trajectory to move the arm to a dropping position --- #
    pass
    # ------------------------------------------------------------------------ #
    return lay_arms_traj
lay_arms_traj = get_lay_arms_traj()

def get_arms_swing_traj():
    arms_swing_trajectory = defaultdict()
    # --- TODO: Design swinging trajectories for both arms --- #
    pass
    # -------------------------------------------------------- #
    return arms_swing_trajectory
arms_swing_trajectory = get_arms_swing_traj()
############################################################################################
