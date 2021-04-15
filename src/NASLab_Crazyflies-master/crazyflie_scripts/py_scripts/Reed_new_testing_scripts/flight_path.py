#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Mission Planning Flight Path Definition
# Called as thread by 'run_mission.py'
# Uses goToSequence to execute optimal trajectories for
# mobile or static charging mission-planning scenarios
#
# Functions:
#       takeoff(z, sync=False)
#       hover(duration)
#       goTo(x, y, z, yaw, num, tol=0.035, sync=False)  *Note: yaw has no effect at this time
#       goToSequence(sequence, nums, sync=False)    *Note: sequence is 3D array matching order of CFs in nums array
#       move(delta_x, delta_y, delta_z, delta_yaw, nums=None, sync=False)   *Note: delta_yaw has no effect at this time
#       moveSequence(sequence, nums, sync=False)    *Note: sequence is a 3D array matching order of CFs in nums array
#       switchLand(events, add_func=None, add_args=None)   *Note: events is array of threading Events (len = # chargers)
#       land(pad_num=0, readjust=True)  *Note: pad_num = 0 -> land on ground
#       setIsCharging()


import time
import rospy
from matplotlib import pyplot as plt
from geometry_msgs.msg import PointStamped


# Set sync = False if individual Crazyflies should wait on each other completing the task before moving on
# Input: Qualisys boolean, Crazyflie instance, CF nums array, Number of Chargers, Threading barrier,
#        4D sequence array split into charging cycles, Charging station coordinates, Replan boolean, queue
# Output: Send cycle number and sequence index of failed CF to queue to be fetched by 'run_mission'
def flight_path(qualisys_connected, cf, cf_nums, num_chargers, bt, charging_separated_sequence, charge_seq_indices,
                stations, replan, failed_thread_q, charge_start_q, run_replan_mobile):

    global stations_idx

    # print(f'Started new thread for CF{cf.cf_num}')

    # Should the battery usage plot be saved and displayed?
    batt_plot_flag = False

    # Setup publishers for the desired position of the mobile charging cars
    car_des_pos_pubs = []
    for k in range(num_chargers):
        des_pos_pub = rospy.Publisher(f'Car{k+1}/desired_pos', PointStamped, queue_size=2)
        car_des_pos_pubs.append(des_pos_pub)
    time.sleep(1)  # Need to wait before using these publishers

    # Setup message for the desired position of the cars - always keep z = 0 since ground robots
    des_pos_msg = PointStamped()
    des_pos_msg.point.x = None
    des_pos_msg.point.y = None
    des_pos_msg.point.z = 0

    # Don't use for mission planning
    # Determine which Crazyflies are not to fly (on charging pads)
    # if qualisys_connected and num_chargers:
    #     cf.setIsCharging()

    # Update z value in sequence where only x and y values are set
    desired_alt = 0.7  # 0.5
    for i in range(len(charging_separated_sequence)):
        for j in range(len(charging_separated_sequence[i])):
            for k in range(len(charging_separated_sequence[i][j])):
                charging_separated_sequence[i][j][k][2] = desired_alt

    # Wait until all threads have started
    bt_idx = bt.wait()
    bt.reset()

    # Determine if mobile charging based on stations being 3D array instead of 2D
    try:
        inner_dimension = len(stations[0][0])
        is_mobile = True
        # Used to find next station to move car to
        if run_replan_mobile:
            stations_idx = [0]*num_chargers
        else:
            stations_idx = [1]*num_chargers
            # Only do this if preplan
            if bt_idx == 0:
                # Move cars to first charging location
                for k in range(num_chargers):
                    next_charging_location = stations[k][0]
                    rospy.loginfo(f'Car{k+1} moving to next location: {next_charging_location}')
                    des_pos_msg.point.x = next_charging_location[0]
                    des_pos_msg.point.y = next_charging_location[1]
                    car_des_pos_pubs[k].publish(des_pos_msg)
                    # if k != (len(num_chargers) - 1):
                    #     time.sleep(2)
            bt.wait()
            bt.reset()

    except:
        is_mobile = False

    start_fly_batt = cf.battery_percent
    start_fly_time = time.time()

    if qualisys_connected:
        # Find charging pad each CF is on before taking off
        # charger_dist = {}
        # for k in range(len(cf.all_charger_pos)):
        #     charger_dist[cf.pad_names[k]] = cf.dist_2D([cf.ext_x, cf.ext_y], cf.all_charger_pos[k])
        # pad_num = int(min(charger_dist, key=charger_dist.get)[-1])
        # print(pad_num)

        cf.is_charging = False
        cf.takeoff(desired_alt)

        # Move ALL cars after takeoff only in replan
        if run_replan_mobile:
            if bt_idx == 0:
                for pad_number in range(1, num_chargers+1):
                    if stations_idx[pad_number - 1] < len(stations[pad_number - 1]):
                        # print(f'CF{cf.cf_num} pad num: {pad_number}, stations: {stations},
                        # stations_idx: {stations_idx}')
                        next_charging_location = stations[pad_number - 1][stations_idx[pad_number - 1]]

                        # Can use pad_number as index because all mobile chargers are initialized sequentially
                        rospy.loginfo(f'Car{pad_number} moving to next location: {next_charging_location}')
                        des_pos_msg.point.x = next_charging_location[0]
                        des_pos_msg.point.y = next_charging_location[1]
                        car_des_pos_pubs[pad_number - 1].publish(des_pos_msg)

                        stations_idx[pad_number - 1] += 1

    # Iterate over 4D array and sleep for charging duration between loops
    for cycle in range(len(charging_separated_sequence)):
        seq_cycle = charging_separated_sequence[cycle]  # 3D sequence array for goToSequence for ith charging cycle

        if qualisys_connected:
            cf.goToSequence(seq_cycle, cf_nums, tol=0.05)  # Navigate through sequence of points

            # Land only if tracking AND needs recharging
            if cf.is_tracking:  # cf.all_cfs_is_tracking[cf.cf_num-1]:
                if len(charge_seq_indices[cf.cf_idx]) > cycle:
                    # Find closest charging pad and land
                    charger_dist = {}
                    for k in range(len(cf.all_charger_pos)):
                        charger_dist[cf.pad_names[k]] = cf.dist_2D([cf.ext_x, cf.ext_y], cf.all_charger_pos[k])
                    pad_num = int(min(charger_dist, key=charger_dist.get)[-1])
                    cf.land(pad_num)
                    end_fly_batt = cf.battery_percent
                    end_fly_time = time.time()
                    rospy.loginfo(f'Cycle {cycle} - CF{cf.cf_num} Flight Time (s): {end_fly_time-start_fly_time:.3f}, '
                                  f'Battery % Change: {end_fly_batt-start_fly_batt}')
                    time.sleep(3)  # Wait in case lost tracking needs to register after "landing"
                else:
                    pad_num = 0
            else:  # Extra safety measure for CFs not being tracked
                pad_num = 0
                # cf.land(pad_num)
                if not replan:
                    return

            # Replan
            if replan:
                if not all(cf.all_cfs_is_tracking):
                    # Only add to queue when failed CF
                    if not cf.is_tracking:
                        # Give main thread access to cycle and index for failed CF
                        # failed_seq_idx := next index to be reached
                        failed_seq_idx = cf.seq_idx
                        # Send values to 'run_mission.py' (cannot return from thread)
                        failed_thread_q.put([cycle, failed_seq_idx])
                    elif not cf.is_charging:
                        cf.land(0)

                    # Return charge start time to queue to know when to takeoff again after replanning (only if tracked)
                    if cf.is_tracking:
                        if len(charge_seq_indices[cf.cf_idx]) <= cycle:
                            end_fly_time = time.time()
                        charge_start_q.put([cf.cf_num, end_fly_time])

                    # Return to run_mission whether or not this thread is failed
                    # Unfailed CFs will have finished their cycle by this point
                    return

            # Wait for charge and takeoff again only if tracking AND needs recharging
            if cf.is_tracking and len(charge_seq_indices[cf.cf_idx]) > cycle:

                # Wait until battery is >90% (full charge) or specified amount of time to takeoff and run next cycle
                # while cf.battery_percent < 90:
                #     time.sleep(15)
                #     if not all(cf.all_cfs_is_tracking):
                #         return
                rospy.loginfo(f'CF{cf.cf_num} charging before next cycle')
                time.sleep(57)  # Charge at most 2x flight time, current flight time w/ MATLAB batt life of 25 is 30 sec

                # To handle the case where another CF fails while this one is charging
                if replan:
                    if not all(cf.all_cfs_is_tracking):
                        if not (len(charge_seq_indices[cf.cf_idx]) > cycle):
                            end_fly_time = time.time()
                        charge_start_q.put([cf.cf_num, end_fly_time])
                        return

                # Comment if wanting to finish mission regardless if all CFs are tracked
                # if not all(cf.all_cfs_is_tracking):
                #     return

                # is_charging automatically set to true when landing
                cf.is_charging = False

                # End fly time/batt <=> start charge time/batt, start fly time/batt <=> end charge time/batt
                start_fly_batt = cf.battery_percent
                start_fly_time = time.time()
                rospy.loginfo(f'Cycle {cycle} - CF{cf.cf_num} Charge Time (s): {start_fly_time-end_fly_time:.3f}, '
                              f'Battery % Change: {start_fly_batt - end_fly_batt}')

                cf.takeoff(desired_alt)

        else:
            # Assumes sequential CF nums starting at 1
            time.sleep(5 * cf.cf_num)

        # Move car to next charging location (if there is one)
        if is_mobile:
            # Wait for associated CF to be done charging
            if qualisys_connected:
                # Get next mobile charger rendezvous location if not the last one
                if pad_num != 0:
                    if stations_idx[pad_num-1] < len(stations[pad_num-1]):
                        # print(f'CF{cf.cf_num} pad num: {pad_num}, stations: {stations}, stations_idx: {stations_idx}')
                        next_charging_location = stations[pad_num-1][stations_idx[pad_num-1]]

                        # Can use pad_num as index because all mobile chargers are initialized sequentially
                        rospy.loginfo(f'Car{pad_num} moving to next location: {next_charging_location}')
                        des_pos_msg.point.x = next_charging_location[0]
                        des_pos_msg.point.y = next_charging_location[1]
                        car_des_pos_pubs[pad_num-1].publish(des_pos_msg)

                        stations_idx[pad_num-1] += 1
            else:
                try:
                    next_charging_location = stations[min(num_chargers, cf.cf_num-1)][cycle + 1]
                    rospy.loginfo(f'Car{pad_num} moving to next location: {next_charging_location}')
                except:
                    continue

        # if cf.cf_num == 1:
        #     print('CYCLE ' + str(cycle + 1))
        #     for j in range(len(seq_cycle)):
        #         print('CF{}: {}'.format(j+1, seq_cycle[j]))

    # Final land on ground when mission is complete
    if qualisys_connected:
        cf.land(0)
    if replan:
        failed_thread_q.put([cycle, "done"])  # Send "done" for failed sequence index to denote completion of mission

    # Display battery usage plot
    if batt_plot_flag:
        plt.figure(cf.cf_num)
        plt.plot(cf.time_arr, cf.batt_percent_arr)
        plt.ylabel('Battery Percentage (%)')
        plt.xlabel('Time (seconds)')
        plt.title('CF' + cf.cf_num + ' Battery Usage over Time')
        plt.savefig('/home/naslab/crazyflie_ws/src/NASLab_Crazyflies/crazyflie_scripts/battery_graphs/CF'
                    + cf.cf_num + '_batt_usage.png')
        plt.show()

    return
