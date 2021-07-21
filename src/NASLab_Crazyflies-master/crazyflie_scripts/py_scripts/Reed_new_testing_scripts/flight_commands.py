#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Flight Path Definition
# Enter the desired Crazyflie actions under the "ENTER FLIGHT PATH HERE" comment based on the below functions
# or uncomment the appropriate test case
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
from matplotlib import pyplot as plt


# Set sync = False if individual Crazyflies should wait on each other completing the task before moving on
# Input: Crazyflie instance, CF nums array, Number of Chargers, Threading barrier,
#        4D sequence array split into charging cycles, Charging station coordinates, Replan boolean, queue
def flight_commands(cf, num_chargers, bt):

    # Should the battery usage plot be saved and displayed?
    batt_plot_flag = False

    # Determine which Crazyflies are not to fly (on charging pads)    NOT USUALLY COMMENTED OUT
    #if num_chargers:
        #cf.setIsCharging()

    # Wait until all threads have started
    bt.wait()
    bt.reset()

    # ENTER FLIGHT PATH HERE
    # TEST CASES
    #cf.takeoff(0.5)

    # Run until out of battery
    # time.sleep(5)
    # if cf.cf_num == 1:
    # cf.takeoff(0.5)
    # cf.hover(600)
    # cf.land()
    # elif cf.cf_num == 5:
    #     time.sleep(5)
    #     cf.takeoff(0.5)
    #     cf.hover(600)
    #     cf.land()
    # Test takeoff land

    #print('---------- running  correct code')

    #cf.takeoff(0.5)
    #cf.hover(4)

    #cf.hover(3000)
    #cf.goTo(0.4078, 0.7699, 0.5, 0, cf.cf_num)
    #cf.hover(1.5)
    #cf.goTo(0.1, 0.1, 0.4, 20, cf.cf_num)
    #cf.hover(1.5)
    #cf.goTo(0.1, 0.1, 0.4, 40, cf.cf_num)
    #cf.hover(1.5)
    #cf.goTo(0.1, 0.1, 0.4, 60, cf.cf_num)
    #cf.hover(1.5)
    #cf.goTo(0.1, 0.1, 0.4, 80, cf.cf_num)
    #cf.hover(1.5)
    #cf.goTo(0.4078, 0.7699, 0.5, 90, cf.cf_num)
    #cf.hover(1.5)
    #cf.goTo(0.1, 0.1, 0.4, 120, cf.cf_num)
    #cf.hover(1.5)
    #cf.goTo(0.1, 0.1, 0.4, 140, cf.cf_num)
    #cf.hover(1.5)
    #cf.land(0)


    # lawnmower
    #cf.takeoff(0.5)
    #cf.hover(2)
    sequence1 = [[[-1, 1, 0.5, 0], [-1, 1, 0.5, 90]]] #[-1, -1, 0.5, 90]]]
    sequence2 = [[[-1, -1, 0.5, -90], [-1, 1, 0.5, -90]]]
    sequence3 = [[[-1, 1, 0.5, 90], [-1, -1, 0.5, 90]]]
    x = 1

    while True:
        # dont do anything
        x+=1

    #cf.goToSequence(sequence1, [1], sync=False)
    #cf.goToSequence(sequence2, [1], sync=False)
    #cf.goToSequence(sequence3, [1], sync=False)
    #cf.hover(3)
    #cf.land()

    # Basic hovering - Recruiting Script 1
    # cf.takeoff(0.3)
    # cf.hover(5)
    # cf.move(0, 0, 0.3, 0)
    # cf.hover(2)
    # cf.move(0, -0.5, 0, 0)
    # cf.hover(5)
    # cf.land(1)

    # Test move sequence - Recruiting Script 2
    # cf.takeoff(0.5)
    # cf.hover(2)
    # sequence = [[[-cf.ext_x, cf.ext_y, cf.ext_z, 0], [-cf.ext_x, -cf.ext_y, cf.ext_z, 0],
    #              [cf.ext_x, -cf.ext_y, cf.ext_z, 0], [cf.ext_x, cf.ext_y, cf.ext_z, 0]]]*2
    # cf.goToSequence(sequence, [1, 2], sync=True)
    # if cf.cf_num == 2:
    #     cf.land(1)
    # else:
    #     cf.land()

    # Test switchLand - Recruiting Script 3
    # cf.takeoff(0.6)
    # cf.hover(3)
    # cf.switchLand(event_array, add_func='move', add_args=[0.4, 0.3, 0, 0])
    # cf.land()

    # Test land accuracy and wireless charging
    # x = cf.ext_x
    # y = cf.ext_y
    # cf.takeoff(0.5)
    # cf.goTo(1, -0.6, 0.5, 0, cf.cf_num)
    # cf.hover(2)
    # cf.goTo(x, y, 0.5, 0, cf.cf_num, tol=0.025)  # tol=0.028
    # cf.hover(1)
    # cf.land()

    # Test yaw change
    # cf.takeoff(0.5)
    # cf.hover(2)
    # cf.move(0, 0, 0, 180)
    # cf.hover(2)
    # cf.land()

    # Test synchronization
    # cf.takeoff(0.3)
    # # cf.hover(5)
    # # cf.move(-0.5, 0.0, 0.0, 0.0)
    # # cf.move(0.5, -0.5, 0.0, 0.0)
    # # cf.move(0.5, 0.5, 0.0, 0.0)
    # # cf.move(-0.5, 0.5, 0.0, 0.0)
    # # cf.move(0.0, -0.5, 0.0, 0.0)
    # # cf.hover(2)
    # cf.land()

    # Test multiple switchLand
    # cf.takeoff(0.3)
    #
    # flying_nums = [4, 5]
    # dist = [[0]*len(flying_nums)]*len(flying_nums)
    # for j in range(len(flying_nums)):
    #     for k in range(len(flying_nums)):
    #         dist[j][k] = cf.dist_2D(cfs_curr_pos[cf.cf_num - 1][0:2], all_charger_pos[k])
    # [row_idx, col_idx] = linear_sum_assignment(dist)  # Might need package for linear_sum_assignment to be re-added
    # mult_charger_pos = [[]]*len(flying_nums)
    # for i in range(len(row_idx)):
    #     mult_charger_pos[row_idx[i]] = all_charger_pos[col_idx[i]]
    #
    # cf.switchLand(flying_nums, mult_charger_pos, event_array[0:len(mult_charger_pos)])
    # cf.land()

    # Test repeated switchLand
    # cf.takeoff(0.4)
    # switch_land_nums = [2, 3]
    # for i in range(4):
    #     cf.switchLand([switch_land_nums[i % 2]], all_charger_pos[0], event_array)
    #     cf.hover(2)
    # cf.land()

    # Test emergency land lost tracking and bounds
    # cf.takeoff(0.3)
    # cf.goToSequence([[[2, 3, 0.3, 0], [-4, 5, 0.3, 0], [-3, -2, 0.3, 0], [6, -3, 0.3, 0]]], [2])
    # cf.move(0.5, 0, 0, 0, [5])
    # cf.land()

    # Test synchronization to hover while blocking
    # cf.takeoff(0.3)
    # cf.move(0.3, 0.3, 0, 0, [1])
    # cf.land()

    # Test goTo
    # cf.takeoff(0.3)
    # cf.goTo(0.8, 0.8, 0.3, 0, 2)
    # cf.land()

    # Test goToSequence
    # cf.takeoff(0.3)
    # cf.goToSequence([[[0, 0, 0.3, 0], [0, 0, 0.8, 0], [-0.7, -0.7, 0.8, 0], [-0.7, 0.7, 0.8, 0],
    #                   [0.7, 0.7, 0.8, 0], [0.7, -0.7, 0.8, 0], [0.3, 0, 0.8, 0], [0.3, 0, 0.4, 0]]], [2])
    # cf.land()

    # Test moveSequence and switchLand
    # cf.takeoff(0.6)
    # cf.hover(5)
    # sequence = [[[0.5, 0.5, 0, 0], [-1, 0, 0, 0], [1.4, -0.7, 0, 0], [0, 0, 0, 0]],
    #             [[-0.5, 0.5, 0, 0], [0, -0.8, 0, 0], [0, 0, 0.3, 0], [0, 0, 0, 0]]]
    # cf.moveSequence(sequence, [2, 5])
    # cf.switchLand([2], all_charger_pos[0], event_array)
    # cf.land()

    # Test synchronization and switchLand
    # cf.takeoff(0.4)
    # cf.move(0.5*sign(cf.ext_x), 0.7, 0.4, 0)
    # cf.switchLand([5], all_charger_pos[0], event_array)
    # cf.land()

    # Test goToSequence with switching of CFs due to low battery
    # if not cf.is_charging:
    #     cf.takeoff(0.3)
    #     sequence = [[[0.8, 0.8, 0.5, 0], [0.8, -0.8, 0.5, 0]] * 40]
    # [-0.8, 0.8, 0.5, 0], [-0.8, -0.8, 0.5, 0], [0.8, -0.8, 0.5, 0]] * 20]
    #     cf.goToSequence(sequence, nums=[cf.cf_num])
    #     cf.land()
    # else:
    #     # switchLand once updated to handle add_func and add_args (notify charging CF of func/args with queue?
    #     cf.switchLand(event_array)

    # Display battery usage plot
   # if batt_plot_flag:
   #     plt.figure(cf.cf_num)
   #     plt.plot(cf.time_arr, cf.batt_percent_arr)
   #     plt.ylabel('Battery Percentage (%)')
   #     plt.xlabel('Time (seconds)')
   #     plt.title('CF' + cf.cf_num + ' Battery Usage over Time')
   #     plt.savefig('/home/naslab/crazyflie_ws/src/NASLab_Crazyflies/crazyflie_scripts/battery_graphs/CF'
   #                 + cf.cf_num + '_batt_usage.png')
   #     plt.show()

    return
