#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script for monitoring estimator data
'''

import sys
import time
import matplotlib.pyplot as plt
import Settings.BRUCE_data as RDS
from Settings.BRUCE_macros import *


if __name__ == '__main__':
    Bruce = RDS.BRUCE()

    ts = []
    x  = []
    y  = []
    z  = []

    num = 800
    target = 0

    fig, ax = plt.subplots()   # can add multiple subplots here, e.g., fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    line1, = ax.plot([], [], animated=True, label='x')
    line2, = ax.plot([], [], animated=True, label='y')
    line3, = ax.plot([], [], animated=True, label='z')

    tmax = 10  # maximum time spam to show [unit in second]
    ax.grid()
    ax.set_xticks(np.arange(0, 100, 1))
    ax.set_xlim([0, tmax])
    ax.set_ylim([-0.5, 0.5])
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Data')
    fig.legend()
    fig.show()
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(ax.bbox)

    np.set_printoptions(formatter={'float': '{:0.2f}'.format})  # show 2 decimal places

    t0 = time.time()
    while True:
        Bruce.update_robot_status()

        plot_data = MF.logvee(Bruce.R_wb)
        # plot_data = Bruce.p_wg
        # plot_data = Bruce.v_wb
        # plot_data = Bruce.foot_contacts

        sys.stdout.write("\r {}".format(plot_data))
        sys.stdout.flush()

        x.append(plot_data[0])
        y.append(plot_data[1])
        z.append(plot_data[2])

        te = time.time() - t0
        ts.append(te)
        rem = ts[-1] // tmax  # calculate remainder
        if rem > target:      # check if current time is beyond the current time spam shown on the figure
            ax.set_xlim([rem * tmax, (rem + 1) * tmax])
            target += 1
            fig.canvas.draw()

        line1.set_data(ts[-num:], x[-num:])
        line2.set_data(ts[-num:], y[-num:])
        line3.set_data(ts[-num:], z[-num:])

        fig.canvas.restore_region(background)

        ax.draw_artist(line1)
        ax.draw_artist(line2)
        ax.draw_artist(line3)

        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()
