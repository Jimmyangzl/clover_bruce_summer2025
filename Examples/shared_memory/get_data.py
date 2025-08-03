#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Get data from shared memory
'''

import time
import Examples.shared_memory.memory_manager as mm

if __name__ == '__main__':
    while True:
        t0 = time.time()
        data = mm.TIME_STATE.get()
        t1 = time.time()
        print("Time get: {:.6f} [sec]  Frequency: {:.2f} [Hz]".format(data['time'][0], 1. / (t1 - t0)), end='\r')
