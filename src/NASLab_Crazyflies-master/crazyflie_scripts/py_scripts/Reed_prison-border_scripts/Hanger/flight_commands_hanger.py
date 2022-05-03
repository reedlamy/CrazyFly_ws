#!/usr/bin/python3
# Author: Reed, Purdue University, reedlamy@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Flight Path Definition
# Enter the desired Crazyflie actions under the "ENTER FLIGHT PATH HERE" comment based on the below functions
# or uncomment the appropriate test case
#
# Functions that could be used:
#       takeoff(z, sync=False)
#       hover(duration)
#       goTo(x, y, z, yaw, num, tol=0.035, sync=False)  *Note: yaw has no effect at this time
#       goToSequence(sequence, nums, sync=False)    *Note: sequence is 3D array matching order of CFs in nums array
#       move(delta_x, delta_y, delta_z, delta_yaw, nums=None, sync=False)   *Note: delta_yaw has no effect at this time
#       moveSequence(sequence, nums, sync=False)    *Note: sequence is a 3D array matching order of CFs in nums array
#       switchLand(events, add_func=None, add_args=None)   *Note: events is array of threading Events (len = # chargers)
#       land(pad_num=0, readjust=True)  *Note: pad_num = 0 -> land on ground
#       setIsCharging()
#       track_object_stationary_camera(num)
#       track_object_stationary_1(x,y,z)   *Note: basically just go to location. Used during testing and is now useless
#       track_object_stationary_camera(num)   *Note: track detected object and publish info for net
#       track_object_stationary_net(num)  *Note: track detected object (Used for net launcher drones)
#       track_object_stationary_net(num) *Note:  used for testing, not needed

import rospy
import time
import math
from matplotlib import pyplot as plt
import Crazyflie
from std_msgs.msg import Int16


#Mid level control script
# init functions - takes in groups of drones and fly nominal mission using low level control commands until told otherwise
# take net drone info and pass info down to low level control

def circle_1_init(cf,cf_num,x,y,z,cf_spotted,stop_threads,ad_flag=False): # ad_flag is advisory flag
    i = 0 # not needed, used for testing, can be set to 0
    cf.takeoff(0.5,cf_num)
    cf.hover(1) # wait for drone to stabilize, take off can be sketchy occasionally

    #while not stop_threads():
    while i <= 5:
        flag = 0
        pub_flag_callback(flag)
        t = i/4
        rad = math.fmod(t,2*math.pi)
        rad = rad+math.pi/2
        if rad > math.pi:
            rad = rad - 2*math.pi
        #drone = [0.75*math.cos(t)+x,0.75*math.sin(t)+y,z,math.degrees(rad),cf_num]
        cf.goTo(1*math.cos(t)+x,1*math.sin(t)+y,0.5,math.degrees(rad),cf_num)
        cf.hover(0.5)
        i += 1


    #exit conditions below

    #if cf_num == cf_spotted(): # if camera made detection, stay in air
    if cf_num == 1:
        flag = 1
        pub_flag_callback(flag)
        cf.track_object_stationary_camera(cf_num)


    elif stop_threads: # otherwise, land camera drone
        cf.land()

def circle_2_init(cf,cf_nums,cf_num,x,y,z,bt,cf_spotted,stop_threads,ad_flag=False):

    i = 0
    bt.wait()
    bt.reset()

    cf.takeoff(0.4,cf_num)
    cf.hover(1)

    while not stop_threads():  # same thing as above function, but with pair of drones so need two separate radian values pi radians separate from each other (opposite sides of local circle)
        t = i/3
        rad = math.fmod(t,2*math.pi)

        radius = 0.4 # Radius of inner circle
        rad1 = (rad+math.pi) % (2*math.pi) - math.pi
        drone1 = [[radius*math.cos(t)+x,radius*math.sin(t)+y,z,math.degrees(rad1)-90]]

        rad2 = (rad+2*math.pi) % (2*math.pi) - math.pi
        drone2 = [[-radius * math.cos(t)+x, -radius * math.sin(t)+y, z, math.degrees(rad2)-90]]

        cf.goToSequence([drone1,drone2], [cf_nums[0],cf_nums[1]], sync=False)
        i += 1

    # exit conditions

    if cf_num == cf_spotted():
        cf.track_object_stationary_camera(cf_num)


    elif stop_threads() and cf_num != cf_spotted():
        cf.land()

def track_object(cf,cf_num,camera_number):

    time.sleep(3)

    cf.track_object_stationary_net(cf_num)


def testing(cf,cf_num,bt,cf_spotted,stop_threads,ad_flag=False):

    i = 0

    cf.takeoff(0.5,cf_num)
    cf.hover(1)

    start_time = time.time()

    cf.goTo(-1.8,-2.3,0.5,0,cf_num)

    while start_time+37 > time.time():
        cf.hover(1)


    cf.land()

def pub_flag_callback(data1):
    pub_flag = rospy.Publisher("Intruder_flag", Int16, queue_size=1)
    flag1 = Int16()
    rate2 = rospy.Rate(5)

    flag1.data = data1

    pub_flag.publish(flag1)
    rate2.sleep()







