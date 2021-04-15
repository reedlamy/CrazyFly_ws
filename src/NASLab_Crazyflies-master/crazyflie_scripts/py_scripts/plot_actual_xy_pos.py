#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Plot position over time for all Crazyflies using xy_pos csv log

import pandas as pd
import zenipy
import sys
from matplotlib import pyplot as plt
from tkinter.filedialog import Tk, askopenfilename

# Ask user for work width and length measurements
work_width = zenipy.entry(text="Enter workspace X length (meters):", placeholder="3.5",
                          title="Crazyflie Mission Planning")
work_length = zenipy.entry(text="Enter workspace Y length (meters):", placeholder="4",
                           title="Crazyflie Mission Planning")

work_width = float(work_width)
work_length = float(work_length)

# Ask user which scenario to plot (static, mobile preplan, mobile replan)
mission_type = zenipy.zlist(title="Crazyflie Mission Planning", text="Select the mission type",
                            columns=["Mission Type"], items=["Static", "MobilePreplan", "MobileReplan"])
mission_type = mission_type[0]

if mission_type == "Static":
    log_location = '/home/naslab/crazyflie_ws/src/NASLab_Crazyflies/crazyflie_scripts/static_cf_actual_pos_logs/'
elif mission_type == "MobilePreplan":
    log_location = '/home/naslab/crazyflie_ws/src/NASLab_Crazyflies/crazyflie_scripts/' \
                   'mobile_preplan_cf_actual_pos_logs/'
elif mission_type == "MobileReplan":
    log_location = '/home/naslab/crazyflie_ws/src/NASLab_Crazyflies/crazyflie_scripts/' \
                   'mobile_replan_cf_actual_pos_logs/'
else:
    print('No selection was made.')
    sys.exit()

Tk().withdraw()
filename = askopenfilename(title='Select XY log file', initialdir=log_location, filetypes=[('CSV Files', '.csv')])
if filename:
    df = pd.read_csv(filename)
    column_names = df.columns
    cf_names = column_names[::2].tolist()
    cf_names = [name[0:-2] for name in cf_names]

    for i in range(0, len(column_names), 2):
        plt.plot(df[column_names[i]], df[column_names[i+1]])
    plt.title(f'Actual {mission_type} Mission Flight Path')
    plt.legend(cf_names)
    plt.xlabel('Workspace X (m)')
    plt.ylabel('Workspace Y (m)')
    plt.xlim(left=-work_width/2)
    plt.ylim(bottom=-work_length/2)
    plt.grid(True, which='both')
    plt.show()
