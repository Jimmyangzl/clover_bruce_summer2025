#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Launch Configuration
'''

SIMULATION = True  # if in simulation or not
GAMEPAD    = False  # if using gamepad or not
ESTIMATION = True   # if running estimation or not (for simulation)
EST_KF     = True   # if KF or CF for estimation
DEMO       = True   # if running demo or op

HARDWARE = not SIMULATION
EST_CF   = not EST_KF 
if HARDWARE:
    ESTIMATION = True
elif SIMULATION:
    GAMEPAD = False

if __name__ == '__main__':
    print(DEMO)