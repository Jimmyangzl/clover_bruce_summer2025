#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for monitoring joint data
'''

import sys
import time
import matplotlib.pyplot as plt
import Settings.BRUCE_data as RDS
from Settings.BRUCE_macros import *


if __name__ == '__main__':
    Bruce = RDS.BRUCE()

    ts = []
    q1 = []
    q2 = []
    q3 = []
    q4 = []
    q5 = []

    num    = 1000
    target = 0

    fig, ax = plt.subplots()  # can add multiple subplots here, e.g., fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    line1, = ax.plot([], [], animated=True, label='q1')
    line2, = ax.plot([], [], animated=True, label='q2')
    line3, = ax.plot([], [], animated=True, label='q3')
    line4, = ax.plot([], [], animated=True, label='q4')
    line5, = ax.plot([], [], animated=True, label='q5')

    tmax = 10  # maximum time spam to show [unit in second]
    ax.grid()
    ax.set_xticks(np.arange(0, 100, 1))
    ax.set_xlim([0, tmax])
    ax.set_ylim([-2, 2])
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Data')
    fig.legend()
    fig.show()
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(ax.bbox)

    t0 = time.time()
    te = time.time() - t0
    print("Joint Position:")
    while True:
        Bruce.update_leg_status()

        q1.append(Bruce.joint[HIP_YAW_R]['q'])
        q2.append(Bruce.joint[HIP_ROLL_R]['q'])
        q3.append(Bruce.joint[HIP_PITCH_R]['q'])
        q4.append(Bruce.joint[KNEE_PITCH_R]['q'])
        q5.append(Bruce.joint[ANKLE_PITCH_R]['q'])

        sys.stdout.write("\r {:.2f} / {:.2f} / {:.2f} / {:.2f} / {:.2f}".format(q1[-1], q2[-1], q3[-1], q4[-1], q5[-1]))
        sys.stdout.flush()

        te = time.time() - t0
        ts.append(te)
        rem = ts[-1] // tmax  # calculate remainder
        if rem > target:      # check if current time is beyond the current time spam shown on the figure
            ax.set_xlim([rem*tmax, (rem + 1)*tmax])
            target += 1
            fig.canvas.draw()

        line1.set_data(ts[-num:], q1[-num:])
        line2.set_data(ts[-num:], q2[-num:])
        line3.set_data(ts[-num:], q3[-num:])
        line4.set_data(ts[-num:], q4[-num:])
        line5.set_data(ts[-num:], q5[-num:])

        fig.canvas.restore_region(background)

        ax.draw_artist(line1)
        ax.draw_artist(line2)
        ax.draw_artist(line3)
        ax.draw_artist(line4)
        ax.draw_artist(line5)

        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()
