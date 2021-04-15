#!/usr/bin/python3

from matplotlib import pyplot as plt
import numpy as np
import os

start_points = [[0.4, 1.4], [0.4, 1.4], [0.4, 1.4]]
start_pCharger = [[0, 0]] * 10
library_path = os.getenv('LD_LIBRARY_PATH')
os.environ['LD_LIBRARY_PATH'] = library_path + ':/usr/local/MATLAB/R2019a/MATLAB_Runtime_R2019a_Update_7_glnxa64/v96/' \
                                               'runtime/glnxa64:/usr/local/MATLAB/R2019a/MATLAB_Runtime_R2019a_Update' \
                                               '_7_glnxa64/v96/bin/glnxa64:/usr/local/MATLAB/R2019a/MATLAB_Runtime' \
                                               '_R2019a_Update_7_glnxa64/v96/sys/os/glnxa64:/usr/local/MATLAB/R2019a' \
                                               '/MATLAB_Runtime_R2019a_Update_7_glnxa64/v96/extern/bin/glnxa64'
import MobileChargerPkg
MyScript = MobileChargerPkg.initialize()
[seq_x, seq_y, charge_seq_indices, stations, boundary, time_given_charger] = \
    MyScript.run_mobile(3, 2, 4, 5, 0.2, 'map_mh370', start_points, start_pCharger, nargout=6)
# print(seq_x, seq_y, charge_seq_indices, charging_pos, boundary, time_given_charger)
print(time_given_charger)
MyScript.terminate()
os.environ['LD_LIBRARY_PATH'] = library_path

for c in range(3):
    seq_x[c] = np.asarray(seq_x[c][0], float).tolist()
    seq_y[c] = np.asarray(seq_y[c][0], float).tolist()

# Remove extra brackets (inner and outer), remove matlab.double type, and sort indices
temp = []
for d in range(len(charge_seq_indices)):
    arr = np.asarray(charge_seq_indices[d][0], int).ravel().tolist()
    arr.sort()
    temp.append(arr)
charge_seq_indices = temp

# Remove extra brackets (inner and outer) and remove matlab.double type
temp = []
for k in range(len(stations)):
    temp.append(np.asarray(stations[k][0], float).tolist())
stations = temp

sequence = seq_x
for i in range(len(seq_x)):
    for j in range(len(seq_x[i])):
        # Sequence: x, y, z, yaw (z and yaw specified later)
        sequence[i][j] = [round(seq_x[i][j] - (4/2), 2), round(seq_y[i][j] - (5/2), 2), 0, 0]

# Remove extra outer bracket
time_given_charger = time_given_charger[0]
print(time_given_charger)

plt.figure('Final Result')
for traj in sequence:
    x = [point[0] for point in traj]
    y = [point[1] for point in traj]
    plt.plot(x, y)
boundary_x = [round(point[0] - (4/2), 2) for point in boundary]
boundary_y = [round(point[1] - (5/2), 2) for point in boundary]
plt.plot(boundary_x, boundary_y, 'k:')
for a in range(len(stations)):
    stations[a] = [b_i - w_i for b_i, w_i in zip(stations[a], [4/2, 5/2])]
    plt.plot(*stations[a], '^k')
plt.title('Mobile Mission Solution')
plt.legend(['CF1', 'CF2', 'CF3', 'Boundary', 'Charger'])
plt.xlabel('Workspace X (m)')
plt.ylabel('Workspace Y (m)')
plt.xlim(left=-4/2)
plt.ylim(bottom=-5/2)
plt.grid(True, which='both')
plt.show()
