#!/usr/bin/python3
# Author: Reed, Purdue University, reedlamy@gmail.com
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
import math
from matplotlib import pyplot as plt
import Crazyflie



# Set sync = False if individual Crazyflies should wait on each other completing the task before moving on
# Input: Crazyflie instance, CF nums array, Number of Chargers, Threading barrier,
#        4D sequence array split into charging cycles, Charging station coordinates, Replan boolean, queue

def circle_1_init(cf,cf_num,x,y,z,cf_spotted,stop_threads,ad_flag=False): # ad_flag is advisory flag
    i = 0
    cf.takeoff(0.4)
    cf.hover(3)

    while not stop_threads:
        t = i/4
        rad = math.fmod(t,math.pi)
        rad = (rad+math.pi) % (2*math.pi) - math.pi
        drone = (0.75*math.cos(t)+x,0.75*math.sin(t)+y,z,math.degrees(rad))
        cf.goTo(drone)

    if cf_num == cf_spotted:
        cf.hover(2)

    elif stop_threads:
        cf.land()

def circle_2_init(cf,cf_nums,cf_num,x,y,z,bt,cf_spotted,stop_threads,ad_flag=False):

    i = 0
    bt.wait()
    bt.reset()

    cf.takeoff(0.4)
    cf.hover(1)

    while not stop_threads():  #ad_flag == False:
        t = i/3
        rad = math.fmod(t,math.pi)

        radius = 0.4
        rad1 = (rad+math.pi) % (2*math.pi) - math.pi
        drone1 = [[radius*math.cos(t)+x,radius*math.sin(t)+y,z,math.degrees(rad1)]]

        rad2 = (rad+2*math.pi) % (2*math.pi) - math.pi
        drone2 = [[-radius * math.cos(t)+x, -radius * math.sin(t)+y, z, math.degrees(rad2)]]

        cf.goToSequence([drone1,drone2], [cf_nums[0],cf_nums[1]], sync=False)
        i += 1


    if cf_num == cf_spotted():
        #cf.hover(3)
        cf.track_object_stationary_camera(cf_num)
        # publish CF number and instance or use ros action

    elif stop_threads() and cf_num != cf_spotted():
        cf.land()

def track_object(cf,cf_num, camera_number):
    #if cf_num == camera_number:
        #cf.track_object_stationary_camera(cf_num)
    cf.track_object_stationary_net_test(cf_num)





