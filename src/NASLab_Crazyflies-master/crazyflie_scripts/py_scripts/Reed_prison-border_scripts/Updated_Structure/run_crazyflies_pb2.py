#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Controller Script
# Set up mission
# using each Crazyflie's functions from the Crazyflie class

import rospy
import Crazyflie
import sys
import signal
import math
import time
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import PoseStamped  # PointStamped
from crazyflie_driver.msg import NameArray
import flight_commands_pb2
from flight_commands_pb2 import circle_1_init, circle_2_init, track_object
from threading import Thread, Barrier
import threading
#from Nominal_Control2 import nominal_control_init

#from threading import Thread, Barrier
#from flight_commands_pb import flight_commands_pb

#s = Mid_Level()

# Charging Pad Position Subscriber Callback
# Input: data = data returned from charger position subscriber (charger_pos_subscriber),
# charger_idx = charging pad index

def nominal_control_init(cf_inst,cf_nums,cf_jobs):

    cf_jobs = [int(cf_job) for cf_job in cf_jobs] # convert list to integers
    # Initialize lists
    Job_1_inst = [] # net launchers
    Job_1_nums = []  # net launchers
    Job_2_inst = [] # cameras
    Job_2_nums = []  # cameras
    Job_3_inst = [] # Misc
    Job_3_nums = []  # Misc

    patterns = 0 # number of circular formations
    group_1_inst = []
    group_1_nums = []
    group_2_inst = []
    group_2_nums = []

    group_cent_pos = []
    cf_flags = []

    # Separate instances by job
    for job in range(0,len(cf_jobs)):
        if cf_jobs[job] == 1:
            Job_1_inst.append(cf_inst[job])
            Job_1_nums.append(cf_nums[job])
        elif cf_jobs[job] == 2:
            Job_2_inst.append(cf_inst[job])
            Job_2_nums.append(cf_nums[job])
        elif cf_jobs[job] == 3:
            Job_3_inst.append(cf_inst[job])
            Job_3_nums.append(cf_nums[job])

    for i in range(0,len(cf_nums)):
        cf_flags.append(0) # 0 means its not chasing drone


    num_patterns = math.ceil(len(Job_2_nums)/2) # up to two drones in each pattern

    if len(Job_2_nums)%2 == 0.5: # there is an odd number of cameras
        group_1_inst.append(Job_2_inst[0]) # singular drone
        group_1_nums.append(Job_2_nums[0])
        for i in range(1,math.floor(len(Job_2_nums)/2)+1):
            group_2_inst.append([Job_2_inst[i * 2 - 1], Job_2_inst[i * 2]]) # pairs of drones
            group_2_nums.append([Job_2_nums[i * 2 - 1], Job_2_nums[i * 2]])  # pairs of drones

    else:
        for i in range(1,len(Job_2_nums)//2+1):
            group_2_inst.append([Job_2_inst[i * 2 - 2], Job_2_inst[i * 2 - 1]]) # pairs of drones
            group_2_nums.append([Job_2_nums[i * 2 - 2], Job_2_nums[i * 2 - 1]]) # pairs of drones


    radius = 0.2
    for i in range(1,num_patterns+1):
        group_cent_pos.append([radius*math.cos((i//num_patterns)*3.14159265*2),radius*math.sin((i//num_patterns)*3.14159265*2)])

    #nominal_flight([Job_1_inst,Job_1_nums],[group_1_inst,group_1_nums],[group_2_inst,group_2_nums],[Job_3_inst,Job_3_nums],group_cent_pos,cf_nums)##########################

    return [Job_1_inst, Job_1_nums]

def nominal_flight(job_1,group_1,group_2,job3,cent_pos,cf_nums):
    t = []
    bt = Barrier(2)

    stop_threads = False
    cf_spotted = 0 # CF that spotted intruder

    if not group_1[1]:
        for i in range(0,len(group_2)-1):
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][0],group_2[1][i], group_2[1][i][0], cent_pos[i][0], cent_pos[i][1], 0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][1],group_2[1][i], group_2[1][i][1], cent_pos[i][0], cent_pos[i][1], 0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))

            #circle_2(group_2[0][i][0],group_2[0][i][1],cent_pos[i][0],cent_pos[i][1],0.5)
    else:
        t.append(Thread(target=circle_1_init, args=(group_1[0][0],group_1[1][0],cent_pos[0][0],cent_pos[0][1],0.5,lambda: cf_spotted,lambda: stop_threads),daemon=True))
        #circle_1(group_1[0][0],cent_pos[0][0],cent_pos[0][1],0.5)
        for i in range(0,len(group_2)-1):
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][0],group_2[1][i],group_2[1][i][0],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][1],group_2[1][i],group_2[1][i][0],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5,bt,lambda: cf_spotted,lambda: stop_threads), daemon=True))
            #circle_2(group_2[0][i][0],group_2[0][i][1],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5)

    for thread in t:
        #thread.start()
        print("hey")

def charger_callback(data, charger_idx):

    ros_hz = 10
    ros_rate = 1/ros_hz  # ROS topic publish rate (10 hz)

    if len(Crazyflie.all_charger_pos[charger_idx]) == 3:
        x_vel = (data.pose.position.x - Crazyflie.all_charger_pos[charger_idx][0]) / ros_rate
        y_vel = (data.pose.position.y - Crazyflie.all_charger_pos[charger_idx][1]) / ros_rate
        Crazyflie.all_charger_vel[charger_idx] = [x_vel, y_vel, 0]

    # Replace previous charger position in all_charger_pos
    Crazyflie.all_charger_pos[charger_idx] = [data.pose.position.x, data.pose.position.y, data.pose.position.z]


# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)


def flag_sub_callback(data1):
    global flag
    flag = data1.data

def cam_id_sub_callback(data2):
    global cam_id
    cam_id = data2.data



# Main function to execute when script is called from launch file
if __name__ == '__main__':

    # Use to test w/o Qualisys
    t1 = []
    qualisys_connected = True
    flag = 0
    cam_id = 0
    flag2 = 0
    # Initialize ROS node for the controller
    rospy.init_node('run_crazyflies', anonymous=True)

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Get list of Crazyflie names from launch file that will be controlled
    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')
    cf_names.sort()
    cf_names.sort(key=len)
    cf_nums = [int(name[2:]) for name in cf_names]

    cf_jobs = rospy.get_param("/cf_jobs")
    cf_jobs = cf_jobs.split(',')

    pad_names = []

    flag_sub = rospy.Subscriber("/Intruder_flag", Int16, flag_sub_callback)

    cam_id_sub = rospy.Subscriber("/CF_CAM_ID", Int16, cam_id_sub_callback)

    # Initialize array so that each Crazyflie's position is available to every other Crazyflie
    Crazyflie.Crazyflie.cfs_curr_pos = [[0] * 3] * int(cf_names[-1][2:])

    # Initialize array to say all cfs are being tracked
    Crazyflie.Crazyflie.all_cfs_is_tracking = [True] * int(cf_names[-1][2:])

    crazy_instances = []
    for name in cf_names:
        if not name:
            rospy.logerr('Please include at least one Crazyflie')
            sys.exit()

        # Create Crazyflie instance based on body name in launch file
        idx = cf_names.index(name)  # Crazyflie's index in 'cf_names' array
        crazy = Crazyflie.Crazyflie(name, idx, qualisys_connected)


        # Subscribe to external_position topic output by 'qualisys_cf_stream.py'

        #crazy.pos_subscriber = rospy.Subscriber("/" + name + "/external_position", PointStamped, crazy.callback)
        crazy.pos_subscriber = rospy.Subscriber("/"+ name + "/external_pose", PoseStamped, crazy.callback)
        #crazy.pos_subscriber = rospy.Subscriber("/external_position", PointStamped, crazy.callback)
        # crazy.pos_subscriber = rospy.Subscriber(name + "/external_pose", PoseStamped, crazy.callback)

        # Check for successful subscription to external position topic
        if qualisys_connected:
            try:
                #rospy.wait_for_message("/" + name + "/external_position", PointStamped, timeout=1)
                rospy.wait_for_message("/" + name + "/external_pose", PoseStamped, timeout=1)
                #rospy.wait_for_message("/external_position", PointStamped, timeout=1)
                # rospy.wait_for_message(name + "/external_pose", PoseStamped, timeout=1)
            except rospy.ROSException:
                print("/" + name + "/external_position")
                #rospy.logerr('Could not subscribe to ' + name + '/external_position message: Timeout')
                rospy.logerr('Could not subscribe to ' + name + '/external_pose message: Timeout')
                sys.exit()
            except rospy.ROSInterruptException:
                rospy.logerr('USER INTERRUPTION')
                sys.exit()

            # Subscribe to battery topic output by Crazyflie driver
            crazy.batt_subscriber = rospy.Subscriber("/" + name + "/battery", Float32, crazy.battery_callback)

            # Check for successful subscription to battery topic
            try:
                rospy.wait_for_message("/" + name + "/battery", Float32, timeout=1)
            except rospy.ROSException:
                rospy.logerr('Could not subscribe to ' + name + '/battery message: Timeout')
                sys.exit()
            except rospy.ROSInterruptException:
                rospy.logerr('USER INTERRUPTION')
                sys.exit()

        # Store instance to use later to start flight path threads
        crazy_instances.append(crazy)


    # Initialize array to hold all threads
    crazy = []

    spotted = False

    # Update class variables if first instance
    crazy_instances[0].global_update(cf_names, pad_names)


    job_1 = nominal_control_init(crazy_instances,cf_nums,cf_jobs) ################################# Call on mid level control
    # RUN detection stuff here (crazy_instances,cf_names)

    rate = rospy.Rate(10)

    #rospy.spin()

    while not rospy.is_shutdown():

        start_time = time.time()

        if flag2 == 0:
            flag2 = 1
            while time.time() < start_time + 100 and spotted == False:
                if flag == 1:
                    spotted = True

                if time.time() > start_time+98:
                    spotted = True # No spotting was actually made but land drones anyways


            if spotted:  # condition to say that CF spotted intruder (or should land mission)
                flag2 = 1
                stop_threads = True  # condition to stop all threads and land drones

                # subscribe to something telling info on drone being spotted
                # ie drone_inst and number
                if flag == 1: # if actual spotting made, assign camera drone to stay in the air
                    #tracker_inst = group_2[0][0][0]
                    #cf_spotted = group_2[1][0][0]
                    cf_spotted = cam_id # published by detection node

                else:
                    cf_spotted = 0 # not actually a drone

                #print(job_1)


                t1.append(Thread(target=track_object, args=(job_1[0][0], job_1[1][0], cf_spotted), daemon=True))

                for thread in t1:
                    thread.start()




        rate.sleep()




    # Keep subscribers listening until script terminates
    #rospy.spin()
