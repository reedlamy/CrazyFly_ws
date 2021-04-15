#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Run MATLAB mission planning algorithm for static/mobile charging case using MATLAB engine in Python


import matlab.engine
import numpy as np


# Return sequence of mission points to be covered by each Crazyflie
def calc_mission_plan(charge_type, num_cfs, work_width, work_length, res, map_name, start_points):

    eng = matlab.engine.start_matlab()
    eng.addpath('/home/naslab/Documents/Static_project-20200304T172257Z-001/Static_project')
    eng.addpath('/home/naslab/Documents/Static_project-20200304T172257Z-001/Static_project/functions')

    # start_points = matlab.double([0.2, 0.7, 0.2, 0.7, 0.2, 0.7])
    start_points = matlab.double(start_points)
    start_points.reshape((num_cfs, 2))

    # Run different MATLAB script depending on charge type selected in UI
    if charge_type == "Static":

        # Pass parameters from UI and CF positions from Qualisys to run MATLAB script
        [seq_x, seq_y, stations, boundary] = eng.run_static(num_cfs, work_width, work_length, res, map_name,
                                                            start_points, nargout=4)

        for c in range(num_cfs):
            seq_x[c] = seq_x[c][0]
            seq_y[c] = seq_y[c][0]

        seq_x = np.asarray(seq_x, float).tolist()
        seq_y = np.asarray(seq_y, float).tolist()

        return [seq_x, seq_y, stations, boundary]

    elif charge_type == "Mobile":
        print('The mobile charge type has not yet been configured')
        return

    else:
        print('ERROR - No charge type specified')
        return
