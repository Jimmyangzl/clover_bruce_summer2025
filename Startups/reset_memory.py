#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Reset shared memory
'''

import Startups.memory_manager as MM


def reset_memory(segment):
    data = {}
    for block in segment.blocks:
        data[block['name']] = block['data']
    segment.set(data)


if __name__ == "__main__":
    reset_memory(MM.THREAD_STATE)
    reset_memory(MM.SIMULATOR_STATE)
    reset_memory(MM.SENSE_STATE)
    reset_memory(MM.GAMEPAD_STATE)
    reset_memory(MM.LEG_STATE)
    reset_memory(MM.LEG_COMMAND)
    reset_memory(MM.ARM_STATE)
    reset_memory(MM.ARM_COMMAND)
    reset_memory(MM.ESTIMATOR_STATE)
    reset_memory(MM.ESTIMATOR_COMMAND)
    reset_memory(MM.PLANNER_COMMAND)
    reset_memory(MM.USER_COMMAND)