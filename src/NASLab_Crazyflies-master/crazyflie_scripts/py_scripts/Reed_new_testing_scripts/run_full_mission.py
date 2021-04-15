#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Controller Script - Specifically for static/mobile mission planning
# Get Crazyflie starting positions from Qualisys and run static
# or mobile GA algorithm in MATLAB. Gets parameters from
# 'launch_mission.launch' that is populated from UI/bash script ('mission_plan.sh').
# Send sequence of points to 'flight_path.py' threads to control Crazyflies

import rospy
import Crazyflie
import sys
import os
import glob
import signal
import zenipy
import csv
import numpy as np
import time
import queue
import math
import shutil
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, PoseStamped
from crazyflie_driver.msg import NameArray
from threading import Thread, Barrier
from flight_path import flight_path
from rospy_spin import rospy_spin
from matplotlib import pyplot as plt


# Charging Pad Position Subscriber Callback
# Input: data = data returned from charger position subscriber (charger_pos_subscriber),
# charger_idx = charging pad index
def charger_callback(data, charger_idx):

    ros_hz = 10
    ros_rate = 1/ros_hz  # ROS topic publish rate (10 hz)

    # For landing on moving pad
    # if len(Crazyflie.Crazyflie.all_charger_pos[charger_idx]) == 3:
    #     x_vel = (data.pose.position.x - Crazyflie.Crazyflie.all_charger_pos[charger_idx][0]) / ros_rate
    #     y_vel = (data.pose.position.y - Crazyflie.Crazyflie.all_charger_pos[charger_idx][1]) / ros_rate
    #     Crazyflie.Crazyflie.all_charger_vel[charger_idx] = [x_vel, y_vel, 0]

    # Replace previous charger position in all_charger_pos
    Crazyflie.Crazyflie.all_charger_pos[charger_idx] = \
        [data.pose.position.x, data.pose.position.y, data.pose.position.z]


# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)


# Main function to execute when script is called from launch file
if __name__ == '__main__':

    # Use to test w/o Qualisys
    qualisys_connected = True

    # Initialize ROS node for the controller
    rospy.init_node('run_full_mission', anonymous=True)

    # Initialize pad names publisher
    pad_names_pub = rospy.Publisher("pad_names", NameArray, queue_size=1)

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Get list of Crazyflie names from launch file that will be controlled
    # Now has the ability to handle any CF number as long as it is named 'CF#'
    cf_names = rospy.get_param("cf_names")
    cf_names = cf_names.split(',')
    cf_names.sort()
    cf_names.sort(key=len)
    cf_nums = [int(name[2:]) for name in cf_names]

    # Initialize array so that each Crazyflie's position is available to every other Crazyflie
    Crazyflie.Crazyflie.cfs_curr_pos = [[0] * 3] * int(cf_names[-1][2:])

    # Initialize array to say all cfs are being tracked
    Crazyflie.Crazyflie.all_cfs_is_tracking = [True] * int(cf_names[-1][2:])

    crazy_instances = []
    start_pCFs = []  # format: [[x1, y1], [x2, y2], ...]
    start_pCharger = []  # format: [[x1, y1], [x2, y2], ...]
    for name in cf_names:
        if not name:
            rospy.logerr('Please include at least one Crazyflie')
            sys.exit()

        # Create Crazyflie instance based on body name in launch file
        idx = cf_names.index(name)  # Crazyflie's index in 'cf_names' array
        crazy = Crazyflie.Crazyflie(name, idx, qualisys_connected)

        # Subscribe to external_position topic output by 'qualisys_cf_stream.py'
        crazy.pos_subscriber = rospy.Subscriber(name + "/external_position", PointStamped, crazy.callback)
        # crazy.pos_subscriber = rospy.Subscriber(name + "/external_pose", PoseStamped, crazy.callback)

        # Check for successful subscription to external position topic
        if qualisys_connected:
            try:
                ext_pos_msg = rospy.wait_for_message(name + "/external_position", PointStamped, timeout=2)
                if any([math.isnan(coord) for coord in
                        [ext_pos_msg.point.x, ext_pos_msg.point.y, ext_pos_msg.point.z]]):
                    rospy.loginfo(f'CF{crazy.cf_num}: '
                                  f'[{ext_pos_msg.point.x}, {ext_pos_msg.point.y}, {ext_pos_msg.point.z}]')
                    raise rospy.ROSException
                else:
                    start_pCFs.append([ext_pos_msg.point.x, ext_pos_msg.point.y])
                # rospy.wait_for_message(name + "/external_pose", PoseStamped, timeout=1)
            except rospy.ROSException:
                rospy.logerr('Could not subscribe to ' + name + '/external_position message: Timeout')
                # rospy.logerr('Could not subscribe to ' + name + '/external_pose message: Timeout')
                sys.exit()
            except rospy.ROSInterruptException:
                rospy.logerr('USER INTERRUPTION')
                sys.exit()

            # Subscribe to battery topic output by Crazyflie driver
            crazy.batt_subscriber = rospy.Subscriber(name + "/battery", Float32, crazy.battery_callback)

            # Check for successful subscription to battery topic
            try:
                rospy.wait_for_message(name + "/battery", Float32, timeout=1)
            except rospy.ROSException:
                rospy.logerr('Could not subscribe to ' + name + '/battery message: Timeout')
                sys.exit()
            except rospy.ROSInterruptException:
                rospy.logerr('USER INTERRUPTION')
                sys.exit()

            # Subscribe to lost frames topic output by Qualisys CF stream
            # Don't wait for message because the first message will be a signal to stop the Crazyflie
            crazy.lost_frames_subscriber = rospy.Subscriber(name + "/lost_frames", PointStamped,
                                                            crazy.lost_frames_callback)

        # Store instance to use later to start flight path threads
        crazy_instances.append(crazy)

    # Parameters from Zenity UI
    charge_type = rospy.get_param("charge_type")
    work_width = rospy.get_param("work_width")
    work_length = rospy.get_param("work_length")
    res = rospy.get_param("res")
    map_name = rospy.get_param("map_name")
    replan = rospy.get_param("replan")  # Always False for Static charging
    if replan:
        orig_num_cfs = len(cf_names)
    num_chargers = rospy.get_param("num_chargers")

    # CSV file location/name (sequence_chargeType_workWidth_workLength_missionResolution_mapName_numCFS_numChargers.csv)
    file_loc = '/home/reed/CrazyFly_ws/src/NASLab_Crazyflies-master/crazyflie_scripts/mission_sequences/'
    file_name = 'seq_' + charge_type + '_' + str(work_width) + '_' + str(work_length) + '_' + str(res) + '_' \
                + map_name + '_' + str(len(cf_names))

    if charge_type == "Mobile":
        # Number of chargers specific to mobile mission solution
        search_file_name = file_name + '_' + str(num_chargers)
    else:
        search_file_name = file_name

    if not qualisys_connected:
        # Start all CFs in the middle of the space
        start_pCFs = [[0, 0]] * len(cf_names)

    # Start all mobile chargers in southwest corner of space
    start_pCharger = [[0, 0]] * 10

    # Have these parameters already been used to calculate trajectories?
    calc_mission = True
    matching_files = glob.glob(file_loc + search_file_name + '*.csv')

    # Look for files specifically for replan and copy mobile solution file with replan name
    # so that deciphering results is more clear
    if replan:
        file_name2 = 'seq_' + charge_type + '_Replan0_' + str(work_width) + '_' + str(work_length) + '_' \
                     + str(res) + '_' + map_name + '_' + str(len(cf_names))
        search_file_name2 = file_name2 + '_' + str(num_chargers)
        matching_files2 = glob.glob(file_loc + search_file_name2 + '*.csv')
        if matching_files2:
            matching_files = matching_files2
        else:
            if matching_files:
                # Copy solution and rename to have replan
                shutil.copyfile(file_loc + search_file_name + '.csv', file_loc + search_file_name2 + '.csv')
                matching_files = glob.glob(file_loc + search_file_name2 + '*.csv')
        file_name = file_name2

    if matching_files:

        # Question to rerun MATLAB script to calculate mission plan
        calc_mission = zenipy.question(title="Crazyflie Mission Planning",
                                       text="Would you like to recalculate new trajectories?")

    # Used to run MATLAB compile packages
    # Keep outside of calc_mission if statement since it is used in replanning
    matlab_runtime_path = ':/usr/local/MATLAB/MATLAB_Runtime/v97/runtime/glnxa64' \
                          ':/usr/local/MATLAB/MATLAB_Runtime/v97/bin/glnxa64' \
                          ':/usr/local/MATLAB/MATLAB_Runtime/v97/sys/os/glnxa64' \
                          ':/usr/local/MATLAB/MATLAB_Runtime/v97/extern/bin/glnxa64'

    if calc_mission:

        # Round start points to nearest discretized point in mission area (and convert to only positive directions)
        disc_start_pCFs = start_pCFs[:]  # just doing disc_start_pCFs = start_pCFs does not create a COPY
        for s in range(len(disc_start_pCFs)):
            disc_start_pCFs[s] = [round(round((s_i + w_i)/res)*res, 2)
                                  for s_i, w_i in zip(disc_start_pCFs[s], [work_width/2, work_length/2])]

        rospy.loginfo('Calculating optimal trajectories in MATLAB...')

        # Change LD_LIBRARY_PATH environment variable temporarily to use MATLAB Runtime
        library_path = os.getenv('LD_LIBRARY_PATH')
        os.environ['LD_LIBRARY_PATH'] = library_path + matlab_runtime_path

        # Run different MATLAB script depending on charge type selected in UI
        if charge_type == "Static":

            # Run MATLAB mission calculate using executable package (StaticChargerPkg)
            # seq_x: x coordinate sequence for each CF to follow
            # seq_y: y coordinate sequence for each CF to follow
            # charge_seq_indices: indices in sequence of points where CFs exit to charge
            # stations: positions of static chargers
            # boundary: coordinates of mission area boundary
            import StaticChargerPkg
            MyStaticCharger = StaticChargerPkg.initialize()
            [seq_x, seq_y, charge_seq_indices, stations, boundary, solve_time] = \
                MyStaticCharger.run_static(len(cf_names), work_width, work_length, res, map_name,
                                           disc_start_pCFs, nargout=6)
            MyStaticCharger.terminate()

            # Reset LD_LIBRARY_PATH environment variable
            os.environ['LD_LIBRARY_PATH'] = library_path

            rospy.loginfo(f'The algorithm solution time for the static charging case was {solve_time} seconds')

            # Remove extra brackets in output arrays and remove matlab.double type
            for c in range(len(cf_names)):
                seq_x[c] = np.asarray(seq_x[c][0], float).tolist()
                seq_y[c] = np.asarray(seq_y[c][0], float).tolist()

            # Remove extra brackets (inner and outer), remove matlab.double type, and sort indices
            temp = []
            for d in range(len(charge_seq_indices)):
                arr = np.asarray(charge_seq_indices[d][0], int).ravel().tolist()
                arr.sort()
                temp.append(arr)
            charge_seq_indices = temp

        elif charge_type == "Mobile":

            # Run MATLAB mission calculate using executable package (StaticChargerPkg)
            # seq_x: x coordinate sequence for each CF to follow
            # seq_y: y coordinate sequence for each CF to follow
            # charge_seq_indices: indices in sequence of points where CFs exit to charge
            # stations: locations where mobile chargers must move to for charging CFs
            # boundary: coordinates of mission area boundary
            import MobileChargerPkg
            MyMobileCharger = MobileChargerPkg.initialize()
            [seq_x, seq_y, charge_seq_indices, stations, boundary, time_given_charger, solve_time] = \
                MyMobileCharger.run_mobile(len(cf_names), num_chargers, work_width, work_length, res, map_name,
                                           disc_start_pCFs, start_pCharger, nargout=7)
            MyMobileCharger.terminate()

            # Reset LD_LIBRARY_PATH environment variable
            os.environ['LD_LIBRARY_PATH'] = library_path

            rospy.loginfo(f'The algorithm solution time for the pre-planned mobile charging case was '
                          f'{solve_time} seconds')

            # Convert from MATLAB array and remove extra outer bracket if needed
            time_given_charger = np.asarray(time_given_charger, float).tolist()
            if type(time_given_charger) == float or type(time_given_charger) == int:
                time_given_charger = [time_given_charger]
            elif type(time_given_charger[0]) == list:
                time_given_charger = time_given_charger[0]

            # Solution not feasible - more chargers
            if any(value < 0 for value in time_given_charger):
                rospy.logerr("Not enough mobile chargers for mission. "
                             "Try making the number of chargers equal to the number of Crazyflies.")
                sys.exit()

            # Remove extra brackets in output arrays and remove matlab.double type
            for c in range(len(cf_names)):
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
                if len(stations[k]) == 0:  # Deal with case where there is an empty array
                    continue
                temp.append(np.asarray(stations[k][0], float).tolist())
            stations = temp

        else:
            rospy.logerr('No charge type specified!')
            sys.exit(0)

        # Save to csv file (row format: all CF x's followed by all CF y's, then charge indices and station/boundary pos)
        with open(file_loc + file_name + '_' + str(len(stations)) + '.csv', 'w', newline='') as file:
            writer = csv.writer(file, quoting=csv.QUOTE_NONNUMERIC)
            writer.writerows(seq_x)
            writer.writerows(seq_y)
            writer.writerows(charge_seq_indices)
            writer.writerows(stations)
            writer.writerows(boundary)

    else:

        # Get routes and stations from csv file
        with open(matching_files[0], newline='') as file:
            reader = csv.reader(file, quoting=csv.QUOTE_NONNUMERIC)
            line_count = 0
            seq_x = []
            seq_y = []
            charge_seq_indices = []
            stations = []
            boundary = []
            for row in reader:
                if line_count < len(cf_names):
                    seq_x.append(row)
                elif line_count < (2 * len(cf_names)):
                    seq_y.append(row)
                elif line_count < (3 * len(cf_names)):
                    charge_seq_indices.append([int(i) for i in row])
                elif line_count < (3 * len(cf_names) + int(file.name[-5])):
                    stations.append(row)
                else:
                    boundary.append(row)
                line_count += 1

    # Reshape mobile stations array to be 3D
    if charge_type == "Mobile":
        temp = []
        for k in range(len(stations)):
            temp.append(np.asarray(stations[k], float).reshape((int(len(stations[k]) / 2), 2)).tolist())
        stations = temp

    # Set num_chargers and pad_names variables
    # Pad names CANNOT handle more than 9 pads at the moment
    num_chargers = len(stations)
    if num_chargers == 0:
        pad_names = ""
    else:
        pad_names = ['Pad1']
        for k in range(1, num_chargers):
            pad_names.append('Pad' + str(k + 1))

    # Put x, y seq together to match goToSequence format (and convert to include negative directions - (0,0) in center)
    sequence = seq_x
    for i in range(len(seq_x)):
        for j in range(len(seq_x[i])):

            # Sequence: x, y, z, yaw (z and yaw specified later)
            # noinspection PyTypeChecker
            sequence[i][j] = [round(seq_x[i][j] - (work_width/2), 2), round(seq_y[i][j] - (work_length/2), 2), 0, 0]

    # Find max length of charge_seq_indices to initialize charging_separated_sequence
    max_seq_indices_length = 0
    for i in range(len(charge_seq_indices)):
        length_i = len(charge_seq_indices[i])
        if length_i > max_seq_indices_length:
            max_seq_indices_length = length_i

    # Make charging_separated_sequence a 4D array with each 3D array broken at the charger_seq_indices
    # charging_separated_sequence = [[]] * (len(charge_seq_indices[0]) + 1)  # bug when index 1 is longer than index 0
    charging_separated_sequence = [[]] * (max_seq_indices_length + 1)
    for j in range(len(charging_separated_sequence)):
        if max_seq_indices_length == 0:
            charging_separated_sequence = [sequence]
            break
        for k in range(len(sequence)):
            if j > len(charge_seq_indices[k]) or len(charge_seq_indices[k]) == 0:
                charging_separated_sequence[j].append([])
                continue
            # append had issues for k = 0
            if k == 0:
                if j == 0:
                    charging_separated_sequence[j] = [sequence[k][0:charge_seq_indices[k][0]]]
                # elif j == (len(charging_separated_sequence) - 1):
                elif j == len(charge_seq_indices[k]):
                    charging_separated_sequence[j] = [sequence[k][charge_seq_indices[k][-1]:]]
                else:
                    charging_separated_sequence[j] = [sequence[k][charge_seq_indices[k][j-1]:charge_seq_indices[k][j]]]
            else:
                if j == 0:
                    try:
                        charging_separated_sequence[j].append(sequence[k][0:charge_seq_indices[k][0]])
                    except IndexError:  # denotes no charge sequence index for that Crazyflie
                        charging_separated_sequence[j].append(sequence[k])
                # elif j == (len(charging_separated_sequence) - 1):
                elif j == len(charge_seq_indices[k]):
                    try:
                        charging_separated_sequence[j].append(sequence[k][charge_seq_indices[k][-1]:])
                    except IndexError:  # denotes no charge sequence index for that Crazyflie
                        charging_separated_sequence[j].append([])
                else:
                    try:
                        charging_separated_sequence[j].append(sequence[k][charge_seq_indices[k][j-1]:
                                                                          charge_seq_indices[k][j]])
                    except IndexError:  # denotes no charge sequence index for that Crazyflie
                        charging_separated_sequence[j].append(sequence[k][charge_seq_indices[k][j-1]:])

    # Show plot of solution (Crazyflie paths, charging stations, mission area outline)
    # Must convert boundary, stations, and plot limits to include negative directions - (0,0) in center)
    plt.figure('Final Result')
    color_cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
    for t in range(len(sequence)):
        x = [point[0] for point in sequence[t]]
        x = [start_pCFs[t][0], *x]
        y = [point[1] for point in sequence[t]]
        y = [start_pCFs[t][1], *y]
        plt.plot(x, y)
    boundary_x = [round(point[0] - (work_width/2), 2) for point in boundary]
    boundary_y = [round(point[1] - (work_length/2), 2) for point in boundary]
    plt.plot(boundary_x, boundary_y, 'k:')
    if charge_type == "Static":
        for a in range(len(stations)):
            stations[a] = [round(b_i - w_i, 2) for b_i, w_i in zip(stations[a], [work_width/2, work_length/2])]
            plt.plot(*stations[a], '^k')
        print('Place static charging stations at the following locations (meters): ')
        print(stations)
    else:
        temp = stations
        for i in range(len(stations)):
            stations_x = [start_pCharger[i][0] - work_width/2]
            stations_y = [start_pCharger[i][1] - work_length/2]
            stations_x.extend([round(s[0] - (work_width/2), 2) for s in stations[i]])
            stations_y.extend([round(s[1] - (work_length/2), 2) for s in stations[i]])
            plt.plot(stations_x, stations_y, '^k--')
            plt.plot(stations_x[0], stations_y[0], 'ks', markersize=15)
            for j in range(len(stations[i])):
                temp[i][j] = [round(b_i - w_i, 2) for b_i, w_i in zip(stations[i][j], [work_width/2, work_length/2])]
        stations = temp
    for t in range(len(sequence)):
        plt.plot(*start_pCFs[t], 'rx')
    if charge_type == "Mobile":
        if replan:
            plt.title('Mobile Replan Mission Solution')
        else:
            plt.title('Mobile Preplan Mission Solution')
    else:
        plt.title('Static Mission Solution')
    plt.legend([*cf_names, 'Boundary', 'Charger'], loc='upper right')
    plt.xlabel('Workspace X (m)')
    plt.ylabel('Workspace Y (m)')
    plt.xlim(left=-work_width/2)
    plt.ylim(bottom=-work_length/2)
    plt.grid(True, which='both')
    plt.savefig(file_loc + file_name + '_' + str(len(stations)) + '.jpg')
    plt.savefig(file_loc + file_name + '_' + str(len(stations)) + '.eps')
    plt.show()

    # Ask user to continue with calculated mission plan
    run_solution = zenipy.question(title="Crazyflie Mission Planning", text="Would you like to run this solution?")

    if not run_solution:
        rospy.logwarn("The solution is not acceptable. Please try again or input different parameters.")
        sys.exit(0)

    else:

        # Publish pad_names array to 'pad_names' topic for 'qualisys_charging_pad_stream'
        pad_msg = NameArray()
        pad_msg.names = pad_names
        pad_names_pub.publish(pad_msg)

        # Update class variables
        Crazyflie.Crazyflie.global_update(cf_names, pad_names)

        # Initialize array to hold all threads
        t = []

        # Barriers to synchronize threads
        bt = Barrier(len(cf_names))

        if qualisys_connected:
            # Subscribe to charging pad positions topic(s) output by 'qualisys_charging_pad_stream.py'
            charger_pos_subscribers = []
            for pad in pad_names:
                if not pad:
                    continue

                charger_pos_subscribers.append(rospy.Subscriber("/charger_pos" + str(pad[-1]), PoseStamped,
                                                                charger_callback, pad_names.index(pad)))

                # Check for successful subscription to charger position topic(s)
                try:
                    charger_pos_msg = rospy.wait_for_message("/charger_pos" + str(pad[-1]), PoseStamped, timeout=2)
                    if any([math.isnan(data_entry) for data_entry in [charger_pos_msg.pose.position.x,
                                                                      charger_pos_msg.pose.position.y,
                                                                      charger_pos_msg.pose.orientation.x,
                                                                      charger_pos_msg.pose.orientation.y,
                                                                      charger_pos_msg.pose.orientation.z,
                                                                      charger_pos_msg.pose.orientation.w]]):
                        raise rospy.ROSException
                except rospy.ROSException:
                    rospy.logerr('Could not subscribe to /charger_pos' + str(pad[-1]) + ' message: Timeout')
                    sys.exit()
                except rospy.ROSInterruptException:
                    rospy.logerr('USER INTERRUPTION')
                    sys.exit()

        # Thread just for running rospy.spin()
        spin_thread = Thread(target=rospy_spin, daemon=False)
        spin_thread.start()

        # Used to block execution of code until failed Crazyflie thread is returned
        failed_thread_q = queue.Queue()

        # Used to get starting time for Crazyflies in replanning to determine when to takeoff again
        charge_start_q = queue.Queue()

        for crazy_inst in crazy_instances:

            # Append thread for each Crazyflie to array t (each executes 'flight_path' function)
            t.append(Thread(target=flight_path, args=(qualisys_connected, crazy_inst, cf_nums, num_chargers, bt,
                                                      charging_separated_sequence, charge_seq_indices, stations,
                                                      replan, failed_thread_q, charge_start_q, False),
                            daemon=True))

        stations_idx = [1] * num_chargers  # Share between all threads

        # Start recording actual position data (set parameter for qualisys cf stream to read)
        rospy.set_param('record_position', True)
        mission_start = time.time()

        # Start all threads
        for thread in t:
            thread.start()
            # time.sleep(2)

        # Replan and send new mission points
        if replan:

            replan_count = 0

            # In case more than 1 failure
            failed_seq_ind = []
            failed_cf_nums = []
            failed_cf_ind = []

            # Start calculating new trajectories after first thread finishes
            # Assume cycle of multiple failures is the same (reasonable)
            # cycle starts at 0
            [cycle, idx] = failed_thread_q.get()  # blocks execution until a value is received
            time.sleep(5)
            failed_seq_ind.append(idx)
            failed_cf_nums.append(Crazyflie.Crazyflie.all_cfs_is_tracking.index(False) + 1)

            # Repeat while failed CF and there are still functioning CFs with uncovered mission points
            # last failed_seq_ind != length (instead of length - 1) b/c it represents next index to be reached
            # Only check last element in failed_seq_ind & failed_cf_nums b/c other indices have already been checked
            while any(Crazyflie.Crazyflie.all_cfs_is_tracking) and len(cf_names) > 1 \
                    and not (cycle == (len(charging_separated_sequence)-1) and
                             failed_seq_ind[-1] == (len(charging_separated_sequence[cycle][failed_cf_nums[-1]-1]))):

                # Convert failed_cf_nums into failed_cf_ind because GA output isn't necessarily in numerical order
                failed_cf_ind = []
                for num in failed_cf_nums:
                    failed_cf_ind.append(cf_nums.index(num))

                # Reduce number of Crazyflies and update other variables
                # old_cf_names_ind = []
                # for f in failed_cf_nums:
                #     old_cf_names_ind.append(cf_names.index('CF' + str(failed_cf_nums[f])))
                # print(old_cf_names_ind)
                old_cf_names = cf_names[:]
                for f in range(len(failed_cf_nums)):
                    cf_names.remove('CF' + str(failed_cf_nums[f]))
                old_cf_nums = cf_nums[:]
                cf_nums = [int(name[2:]) for name in cf_names]
                old_crazy_instances = crazy_instances[:]
                failed_cf_ind.sort(reverse=True)
                for failed_idx in failed_cf_ind:
                    del crazy_instances[failed_idx]

                rospy.loginfo(f'CF{failed_cf_nums[-1]} has failed. Replanning new mission...')

                # Handle additional Crazyflie failures before mission planning algorithm
                # Check if failure is in another index
                all_failures = [idx for idx, value in enumerate(Crazyflie.Crazyflie.all_cfs_is_tracking) if not value]
                failed_cf_nums_tracking_arr_idx = [f - 1 for f in failed_cf_nums]
                # new_failures is index in all_cfs_is_tracking
                new_failures = [failure for failure in all_failures if failure not in failed_cf_nums_tracking_arr_idx]
                if len(new_failures) > 0:
                    for i in range(len(new_failures)):
                        try:
                            [cycle, idx] = failed_thread_q.get(timeout=5)
                            failed_seq_ind.append(idx)
                            failed_cf_nums.append(new_failures[i] + 1)
                        except:
                            break
                    cf_names = old_cf_names[:]
                    cf_nums = old_cf_nums[:]
                    crazy_instances = old_crazy_instances[:]
                    continue

                # Run mission planning replan GA with updated parameters
                library_path = os.getenv('LD_LIBRARY_PATH')
                os.environ['LD_LIBRARY_PATH'] = library_path + matlab_runtime_path

                import MobileChargerPkg
                MyMobileCharger = MobileChargerPkg.initialize()

                # MATLAB corrects for difference in starting index (0 vs. 1)
                [seq_x, seq_y, charge_seq_indices, stations, boundary, time_given_charger, solve_time] = \
                    MyMobileCharger.run_replan_mobile(orig_num_cfs, len(cf_names), num_chargers, work_width,
                                                      work_length, res, map_name, cycle, failed_seq_ind, failed_cf_ind,
                                                      replan_count, nargout=7)
                MyMobileCharger.terminate()

                os.environ['LD_LIBRARY_PATH'] = library_path

                rospy.loginfo(f'The algorithm solution time for the re-planned mobile charging case was '
                              f'{solve_time} seconds')

                # New start points
                start_pCFs = [Crazyflie.Crazyflie.cfs_curr_pos[n-1] for n in cf_nums]
                start_pCharger = Crazyflie.Crazyflie.all_charger_pos

                # # Reset tracking array by making index True (do after replanning since other CFs are still flying)
                # # (removing from crazy instances will prevent it from flying again however)
                # for failed_cf in failed_cf_nums:
                #     Crazyflie.Crazyflie.all_cfs_is_tracking[failed_cf - 1] = True

                # Handle additional Crazyflie failures after mission planning algorithm
                # if not all(Crazyflie.Crazyflie.all_cfs_is_tracking):
                #     # Start calculating new trajectories after first thread finishes
                #     [cycle, idx] = failed_thread_q.get()
                #     # Start new arrays b/c previous GA solution already removed other failed CFs
                #     failed_seq_ind = [idx]
                #     failed_cf_nums = [Crazyflie.Crazyflie.all_cfs_is_tracking.index(False) + 1]
                #     continue
                # Handle additional Crazyflie failures before mission planning algorithm
                # Check if failure is in another index
                all_failures = [idx for idx, value in enumerate(Crazyflie.Crazyflie.all_cfs_is_tracking) if not value]
                failed_cf_nums_tracking_arr_idx = [f - 1 for f in failed_cf_nums]
                # new_failures is index in all_cfs_is_tracking
                new_failures = [failure for failure in all_failures if failure not in failed_cf_nums_tracking_arr_idx]
                if len(new_failures) > 0:
                    for i in range(len(new_failures)):
                        try:
                            [cycle, idx] = failed_thread_q.get(timeout=5)
                            failed_seq_ind.append(idx)
                            failed_cf_nums.append(new_failures[i] + 1)
                        except:
                            break
                    cf_names = old_cf_names[:]
                    cf_nums = old_cf_nums[:]
                    crazy_instances = old_crazy_instances[:]
                    continue

                replan_count += 1

                # Convert from MATLAB array and remove extra outer bracket if needed
                time_given_charger = np.asarray(time_given_charger, float).tolist()
                if type(time_given_charger) == float or type(time_given_charger) == int:
                    time_given_charger = [time_given_charger]
                elif type(time_given_charger[0]) == list:
                    time_given_charger = time_given_charger[0]

                # Solution not feasible - more chargers
                if any(value < 0 for value in time_given_charger):
                    rospy.logerr("Not enough mobile chargers for replan. Ending mission... ")
                    sys.exit()

                # Alter outputs to be in correct format
                # Remove extra brackets in output arrays and remove matlab.double type
                for c in range(len(cf_names)):
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
                    if len(stations[k]) == 0:  # Deal with case where there is an empty array
                        temp.append([])
                    else:
                        temp.append(np.asarray(stations[k][0], float).tolist())
                stations = temp
                # Reshape mobile stations array to be 3D
                temp = []
                for k in range(len(stations)):
                    temp.append(np.asarray(stations[k], float).reshape((int(len(stations[k]) / 2), 2)).tolist())
                stations = temp
                # Put x, y seq together to match goToSequence format (and convert to include negative
                # directions - (0,0) in center)
                sequence = seq_x
                for i in range(len(seq_x)):
                    for j in range(len(seq_x[i])):
                        # Sequence: x, y, z, yaw (z and yaw specified later)
                        sequence[i][j] = [round(seq_x[i][j] - (work_width / 2), 2),
                                          round(seq_y[i][j] - (work_length / 2), 2), 0, 0]
                # Find max length of charge_seq_indices to initialize charging_separated_sequence
                max_seq_indices_length = 0
                for i in range(len(charge_seq_indices)):
                    length_i = len(charge_seq_indices[i])
                    if length_i > max_seq_indices_length:
                        max_seq_indices_length = length_i
                    # Make charging_separated_sequence a 4D array with each 3D array broken at the charger_seq_indices
                    charging_separated_sequence = [[]] * (max_seq_indices_length + 1)
                    for j in range(len(charging_separated_sequence)):
                        if max_seq_indices_length == 0:
                            charging_separated_sequence = [sequence]
                            break
                        for k in range(len(sequence)):
                            if j > len(charge_seq_indices[k]) or len(charge_seq_indices[k]) == 0:
                                charging_separated_sequence[j].append([])
                                continue
                            # append had issues for k = 0
                            if k == 0:
                                if j == 0:
                                    charging_separated_sequence[j] = [sequence[k][0:charge_seq_indices[k][0]]]
                                elif j == len(charge_seq_indices[k]):
                                    charging_separated_sequence[j] = [sequence[k][charge_seq_indices[k][-1]:]]
                                else:
                                    charging_separated_sequence[j] = [sequence[k][charge_seq_indices[k][j - 1]:
                                                                                  charge_seq_indices[k][j]]]
                            else:
                                if j == 0:
                                    try:
                                        charging_separated_sequence[j].append(sequence[k][0:charge_seq_indices[k][0]])
                                    except IndexError:  # denotes no charge sequence index for that Crazyflie
                                        charging_separated_sequence[j].append(sequence[k])
                                elif j == len(charge_seq_indices[k]):
                                    try:
                                        charging_separated_sequence[j].append(sequence[k][charge_seq_indices[k][-1]:])
                                    except IndexError:  # denotes no charge sequence index for that Crazyflie
                                        charging_separated_sequence[j].append([])
                                else:
                                    try:
                                        charging_separated_sequence[j].append(sequence[k][charge_seq_indices[k][j - 1]:
                                                                                          charge_seq_indices[k][j]])
                                    except IndexError:  # denotes no charge sequence index for that Crazyflie
                                        charging_separated_sequence[j].append(
                                            sequence[k][charge_seq_indices[k][j - 1]:])

                # Save plot of replanning solution (don't show since user will have to interact)
                # Must convert boundary, stations, and plot limits to include negative directions - (0,0) in center)
                plt.figure(f'Replanning Result {replan_count}')
                # Set color cycle to match original plot when some Crazyflies fail
                # Can only handle up to 10 lines (Crazyflies)
                new_color_cycle = [color_cycle[c] for c in range(len(color_cycle)) if c not in failed_cf_ind]
                plt.gca().set_prop_cycle(color=new_color_cycle)
                color_cycle = new_color_cycle[:]
                for t in range(len(sequence)):
                    x = [point[0] for point in sequence[t]]
                    x = [start_pCFs[t][0], *x]
                    y = [point[1] for point in sequence[t]]
                    y = [start_pCFs[t][1], *y]
                    plt.plot(x, y)
                boundary_x = [round(point[0] - (work_width / 2), 2) for point in boundary]
                boundary_y = [round(point[1] - (work_length / 2), 2) for point in boundary]
                plt.plot(boundary_x, boundary_y, 'k:')

                temp = stations
                for i in range(len(stations)):
                    stations_x = [start_pCharger[i][0]]
                    stations_y = [start_pCharger[i][1]]
                    stations_x.extend([round(s[0] - (work_width / 2), 2) for s in stations[i]])
                    stations_y.extend([round(s[1] - (work_length / 2), 2) for s in stations[i]])
                    plt.plot(stations_x, stations_y, '^k--')
                    plt.plot(stations_x[0], stations_y[0], 'ks', markersize=8)
                    for j in range(len(stations[i])):
                        temp[i][j] = [round(b_i - w_i, 2) for b_i, w_i in
                                      zip(stations[i][j], [work_width / 2, work_length / 2])]
                stations = temp

                for s in range(len(sequence)):
                    plt.plot(start_pCFs[s][0], start_pCFs[s][1], 'rx')

                plt.title(f'Mobile Replan Mission Solution - Replan {replan_count}')
                plt.legend([*cf_names, 'Boundary', 'Charger'], loc='lower right')
                plt.xlabel('Workspace X (m)')
                plt.ylabel('Workspace Y (m)')
                plt.xlim(left=-work_width / 2)
                plt.ylim(bottom=-work_length / 2)
                plt.grid(True, which='both')
                replan_file_name = 'seq_mobile_Replan' + str(replan_count) + '_' + str(work_width) + '_' \
                                   + str(work_length) + '_' + str(res) + '_' + map_name + '_' + str(len(cf_names))
                plt.savefig(file_loc + replan_file_name + '_' + str(len(stations)) + '.jpg')
                plt.savefig(file_loc + replan_file_name + '_' + str(len(stations)) + '.eps')

                # Update other variables after all Crazyflies are done flying (might need to join threads?)
                Crazyflie.Crazyflie.cf_names = cf_names
                Crazyflie.Crazyflie.cfs_curr_pos = [[0] * 3] * int(cf_names[-1][2:])
                Crazyflie.Crazyflie.all_cfs_is_tracking = [True] * int(cf_names[-1][2:])
                for crazy in crazy_instances:
                    crazy.cf_idx = cf_names.index(crazy.prefix)

                # Restart threads individually when fully charged (simulated or real)
                charge_start_time = [0] * len(crazy_instances)
                for i in range(len(cf_nums)):
                    try:
                        [cf_number, start_time] = charge_start_q.get(timeout=10)
                    except:
                        continue
                    charge_start_time[cf_names.index('CF' + str(cf_number))] = start_time

                # Reset just in case
                failed_thread_q = queue.Queue()
                charge_start_q = queue.Queue()
                t = []
                bt = Barrier(len(cf_names))
                while len(t) != len(cf_names):
                    for crazy_inst in crazy_instances:
                        # Wait until battery is above 90% (full charge) to start new sequence
                        # or for specified amount of time (sec of simulated battery charging time)
                        if (time.time() - charge_start_time[crazy_instances.index(crazy_inst)]) >= 60:
                            thread = Thread(target=flight_path, args=(qualisys_connected, crazy_inst, cf_nums,
                                                                      num_chargers, bt, charging_separated_sequence,
                                                                      charge_seq_indices, stations, replan,
                                                                      failed_thread_q, charge_start_q, True),
                                            daemon=True)
                            t.append(thread)
                    # time.sleep(15) - for real battery level check

                stations_idx = [0] * num_chargers  # Reset global variable before starting threads

                # Start all threads
                for thread in t:
                    thread.start()

                # Wait for next time a replan is needed
                [cycle, idx] = failed_thread_q.get()
                if idx == "done":
                    break
                time.sleep(5)
                # Start new arrays b/c previous GA solution already removed other failed CFs
                failed_seq_ind = [idx]
                failed_cf_nums = [Crazyflie.Crazyflie.all_cfs_is_tracking.index(False) + 1]
                failed_cf_ind = []

            # Determine if mobile replan mission succeeded - if all threads put "done" into queue
            if idx == "done":
                for i in range(len(t)-1):
                    try:
                        [cycle, idx] = failed_thread_q.get(timeout=20)  # timeout=30
                        if i == range(len(t)-2):
                            rospy.loginfo('Mission completed successfully!')
                    except:
                        rospy.logwarn('Mission failed. Not all mission points were reached.')
                        break
            else:
                rospy.logwarn('Mission failed. Not all mission points were reached.')

        # Determine if static/mobile preplan mission succeeded - if all CFs are tracking at the end
        else:

            # Wait for all threads to finish
            for thread in t:
                thread.join()

            if all(Crazyflie.Crazyflie.all_cfs_is_tracking):
                rospy.loginfo('Mission completed successfully!')
            else:
                rospy.logwarn('Mission failed. Not all mission points were reached.')

        rospy.loginfo(f'The final mission time (including charging) was {(time.time() - mission_start):.3f} seconds.')
