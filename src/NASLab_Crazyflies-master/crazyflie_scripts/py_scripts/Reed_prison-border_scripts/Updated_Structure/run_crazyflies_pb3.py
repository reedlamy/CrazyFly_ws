#!/usr/bin/python3
# Authors: Reed Lamy, Purdue University, reedlamy@gmail.com
# Chris Moneyron, Purdue University, cmoneyron@gmail.com
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
from geometry_msgs.msg import PoseStamped, PointStamped
from crazyflie_driver.msg import NameArray
import flight_commands_pb3
from flight_commands_pb3 import circle_1_init, circle_2_init, track_object
from threading import Thread, Barrier
import threading
#from Nominal_Control2 import nominal_control_init
#from threading import Thread, Barrier
#from flight_commands_pb import flight_commands_pb




# Initializes mission planning algorithm, separating drones by jobs from launch file, and grouping camera drones properly
def nominal_control_init(cf_inst,cf_nums,cf_jobs):

    cf_jobx = [int(cf_job) for cf_job in cf_jobs] # convert list to integers

    # Initialize lists
    Job_1_inst = [] # net launcher instances
    Job_1_nums = []  # net launcher numbers ie CF1 --> 1
    Job_2_inst = [] # camera instances
    Job_2_nums = []  # cameras numbers
    Job_3_inst = [] # Misc instances
    Job_3_nums = []  # Misc numbers

    patterns = 0 # number of circular formations
    group_1_inst = [] # initialize - group 1 is if there are an odd number of camera drone, one camera flies in a pattern alone
    group_1_nums = [] # initialize
    group_2_inst = [] # initialize - Pairs of two of camera drones
    group_2_nums = [] # initialize

    group_cent_pos = [] # initialize - center positions for local circle patterns
    cf_flags = [] # flag not used, but could be useful for further development

    # Separate CF's into instances and number lists by job
    for job in range(0,len(cf_jobx)):
        if cf_jobx[job] == 1:
            Job_1_inst.append(cf_inst[job])
            Job_1_nums.append(cf_nums[job])
        elif cf_jobx[job] == 2:
            Job_2_inst.append(cf_inst[job])
            Job_2_nums.append(cf_nums[job])
        elif cf_jobx[job] == 3:
            Job_3_inst.append(cf_inst[job])
            Job_3_nums.append(cf_nums[job])


    # initialize all cf_flags to 0, signifying all drones are flying nominal mission
    for i in range(0,len(cf_nums)):
        cf_flags.append(0) # 0 means its not chasing drone


    # Calculate the number of separate local patterns the camera drones will fly
    num_patterns = math.ceil(len(Job_2_nums)/2) # up to two drones in each pattern

    # Populate Group 1 and Group 2 information if there is a pattern with a single drone
    if len(Job_2_nums)%2 == 1: # there is an odd number of camera drones, meaning there will be info in group 1
        group_1_inst.append(Job_2_inst[0]) # singular drone
        group_1_nums.append(Job_2_nums[0])
        for i in range(1,math.floor(len(Job_2_nums)/2)+1):
            group_2_inst.append([Job_2_inst[i * 2 - 1], Job_2_inst[i * 2]]) # pairs of drones
            group_2_nums.append([Job_2_nums[i * 2 - 1], Job_2_nums[i * 2]])  # pairs of drones

    # Populate Group 2 information if there is NOT a pattern with a single drone
    else:
        for i in range(1,len(Job_2_nums)//2+1):
            group_2_inst.append([Job_2_inst[i * 2 - 2], Job_2_inst[i * 2 - 1]]) # pairs of drones
            group_2_nums.append([Job_2_nums[i * 2 - 2], Job_2_nums[i * 2 - 1]]) # pairs of drones

    radius = 0.2 #radius of large inner circle to place separate local patterns around
    for i in range(1,num_patterns+1):
        if num_patterns == 1: # if there is only one pattern being flown ie 1 or 2 camera drones, no need to use large inner circle
            group_cent_pos.append([0, 0])  # if there is only one pattern, center around 0,0
        else:
            group_cent_pos.append([radius * math.cos((i // num_patterns) * 3.14159265 * 2), radius * math.sin((i // num_patterns) * 3.14159265 * 2)])  # if there are are multiple patters, center the patterns evenly around a large circle

    # useless code, could delete
    #nominal_flight([Job_1_inst,Job_1_nums],[group_1_inst,group_1_nums],[group_2_inst,group_2_nums],[Job_3_inst,Job_3_nums],group_cent_pos,cf_nums)
    #return [Job_1_inst, Job_1_nums]

    # return all useful information initialization info in an organized fashion
    return [[Job_1_inst, Job_1_nums], [group_1_inst, group_1_nums], [group_2_inst, group_2_nums], [Job_3_inst,  Job_3_nums], group_cent_pos, cf_nums]


# useless - remnant of an older structure
'''
def nominal_flight(job_1,group_1,group_2,job3,cent_pos,cf_nums): # Can delete soon
    bt = Barrier(2)

    stop_threads = False
    cf_spotted = 0 # CF that spotted intruder


    if not group_1[1]:
        for i in range(0,len(group_2[0])-1):
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][0],group_2[1][i], group_2[1][i][0], cent_pos[i][0], cent_pos[i][1], 0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][1],group_2[1][i], group_2[1][i][1], cent_pos[i][0], cent_pos[i][1], 0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))

            #circle_2(group_2[0][i][0],group_2[0][i][1],cent_pos[i][0],cent_pos[i][1],0.5)
    else:
        t.append(Thread(target=circle_1_init, args=(group_1[0][0],group_1[1][0],cent_pos[0][0],cent_pos[0][1],0.5,lambda: cf_spotted,lambda: stop_threads),daemon=True))
        #circle_1(group_1[0][0],cent_pos[0][0],cent_pos[0][1],0.5)
        for i in range(0,len(group_2[0])-1):
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][0],group_2[1][i],group_2[1][i][0],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][1],group_2[1][i],group_2[1][i][0],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5,bt,lambda: cf_spotted,lambda: stop_threads), daemon=True))
            #circle_2(group_2[0][i][0],group_2[0][i][1],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5)
'''

# Will be used for recharging
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

# flag used throughout mission planner
def flag_sub_callback(data1):
    global flag
    flag = data1.data # this flag is from the detection node, 0 = no detected intruder, 1 = intruder has been detected

def cam_id_sub_callback(data2):
    global cam_id
    cam_id = data2.data # CF ID of camera drone that made the spotting, (so it can remain flying)

def adversary_sub_callback(data):
    global intruder_x, intruder_y, intruder_z
    intruder_x = data.point.x
    intruder_y = data.point.y
    intruder_z = data.point.z



# Main function to execute when script is called from launch file
if __name__ == '__main__':

    t = [] # initialize nominal mission camera drone thread
    t1 = [] # initialize reaction thread for net launchers

    # Use to test w/o Qualisys
    qualisys_connected = True
    flag = 0 # flag is from the detection node, 0 = no detected intruder, 1 = intruder has been detected
    cam_id = 1 # place holder for now, hard coded in that CF1 made detection
    flag2 = 0 # used to run nominal mission (I.E. takeoff, begin pattern) only once
    cf_spotted = 0 # (Should rename) Variable used to signify script should move on, (Either mission has been runing for max time or detection has been made)
    net_dist = 1000 #####################################################################################################################################################
    net = 0###############################################################3

    # Location of intruder
    intruder_x = 0
    intruder_y = 0
    intruder_z = 0

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

    # split names into useable numbers
    cf_jobs = rospy.get_param("/cf_jobs")
    cf_jobs = cf_jobs.split(',')

    # not used, but will be
    pad_names = []

    # start subscribers
    flag_sub = rospy.Subscriber("/Intruder_flag", Int16, flag_sub_callback)
    cam_id_sub = rospy.Subscriber("/CF_CAM_ID", Int16, cam_id_sub_callback)
    intruder_sub = rospy.Subscriber("/global_adv", PointStamped, adversary_sub_callback) ###############################################################3

    # Initialize array so that each Crazyflie's position is available to every other Crazyflie
    Crazyflie.Crazyflie.cfs_curr_pos = [[0] * 3] * int(cf_names[-1][2:])

    # Initialize array to say all cfs are being tracked
    Crazyflie.Crazyflie.all_cfs_is_tracking = [True] * int(cf_names[-1][2:])

    # Organize instances for crazyflie functionality
    crazy_instances = []
    for name in cf_names:
        if not name:
            rospy.logerr('Please include at least one Crazyflie')
            sys.exit()

        # Create Crazyflie instance based on body name in launch file
        idx = cf_names.index(name)  # Crazyflie's index in 'cf_names' array
        crazy = Crazyflie.Crazyflie(name, idx, qualisys_connected)


        # Subscribe to external_position topic output by 'qualisys_cf_stream_pb2.py'


        crazy.pos_subscriber = rospy.Subscriber("/"+ name + "/external_pose", PoseStamped, crazy.callback)
        # from trials, could be useful if modifications are made to charge msg type
        #crazy.pos_subscriber = rospy.Subscriber("/external_position", PointStamped, crazy.callback)
        # crazy.pos_subscriber = rospy.Subscriber(name + "/external_pose", PoseStamped, crazy.callback)
        #crazy.pos_subscriber = rospy.Subscriber("/" + name + "/external_position", PointStamped, crazy.callback)

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

    # Initialize array to hold all threads, I dont believe this is used, but may be useful
    crazy = []

    spotted = False

    # Update class variables if first instance
    crazy_instances[0].global_update(cf_names, pad_names)

    # get jobs in organized fashion from function
    jobs = nominal_control_init(crazy_instances,cf_nums,cf_jobs) # Call on mid level control

    # can delete, not used but could be useful for reference
    # nominal_flight([Job_1_inst,Job_1_nums],[group_1_inst,group_1_nums],[group_2_inst,group_2_nums],[Job_3_inst,Job_3_nums],group_cent_pos,cf_nums)
    # (job_1,group_1,group_2,job3,cent_pos,cf_nums)

    # seperate info into separate variables
    job_1 = jobs[0]
    group_1 = jobs[1]
    group_2 = jobs[2]
    job3 = jobs[3]
    cent_pos = jobs[4]
    cf_nums = jobs[5]


    rate = rospy.Rate(10)

    # needed for threading, describes how threading will run (I believe)
    bt = Barrier(2)

    # variable to signify initiate landing of camera drones, whatever the reason
    stop_threads = False
    cf_spotted = 0 # CF that spotted intruder (if there was one, CF0 is not a drone so this is the nominal value)


    # create threads for each local group
    if not group_1[1]:
        for i in range(0,len(group_2[0])-1):
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][0],group_2[1][i], group_2[1][i][0], cent_pos[i][0], cent_pos[i][1], 0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][1],group_2[1][i], group_2[1][i][1], cent_pos[i][0], cent_pos[i][1], 0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))
            #circle_2(group_2[0][i][0],group_2[0][i][1],cent_pos[i][0],cent_pos[i][1],0.5)
    else:
        t.append(Thread(target=circle_1_init, args=(group_1[0][0],group_1[1][0],cent_pos[0][0],cent_pos[0][1],0.5,lambda: cf_spotted,lambda: stop_threads),daemon=True))
        #circle_1(group_1[0][0],cent_pos[0][0],cent_pos[0][1],0.5)
        for i in range(0,len(group_2[0])-1):
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][0],group_2[1][i],group_2[1][i][0],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5,bt,lambda: cf_spotted,lambda: stop_threads),daemon=True))
            t.append(Thread(target=circle_2_init, args=(group_2[0][i][1],group_2[1][i],group_2[1][i][0],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5,bt,lambda: cf_spotted,lambda: stop_threads), daemon=True))
            #circle_2(group_2[0][i][0],group_2[0][i][1],cent_pos[i+1][0],cent_pos[i+1][1],[1],0.5)

    # start nominal threads
    for thread in t:
        thread.start()

    #rospy.spin()
    # fixing bug where drone was getting two sets of commands at once
    takeoff_time = time.time() + 10
    while takeoff_time > time.time():
        flag = 0

    # below is where detection reaction happens
    while not rospy.is_shutdown():
        start_time = time.time() # used to stop mission after some time

        if flag2 == 0: # used to run nominal mission (I.E. takeoff, begin pattern) only once
            flag2 = 1 # used to run nominal mission (I.E. takeoff, begin pattern) only once

            while time.time() < start_time + 100 and spotted == False: # while conditions are meet to carry on with nominal
                if flag == 1: # if detection is actually made
                    spotted = True

                if time.time() > start_time+98: # if time limit is reached
                    spotted = True # No spotting was actually made but land drones anyways, flag = 0 still


            if spotted:  # condition to say script should move on from nominal
                flag2 = 1

                # subscribe to something telling info on drone being spotted, this does not occur but variable is cam_id and is set automatically to 1 for CF1
                # get info like drone_inst and number


                if flag == 1: # if actual spotting made, assign camera drone to stay in the air
                    #tracker_inst = group_2[0][0][0]
                    #cf_spotted = group_2[1][0][0]
                    cf_spotted = cam_id # published by detection node
                    stop_threads = True  # condition to stop all threads and land drones

                    #Find closest net launcher ###########################################################################33
                    #for i in len(job_1[1]):
                    #    Crazyflie.Crazyflie.cfs_curr_pos # get xyz of drone
                    #    dist = abs(xyz-xyz)

                    #    if dist < net_dist: #this net is closer
                    #        net_dist = dist
                    net = 0  # will be assigned right above


                    #Check to see if net laucher is under camera drone, if it is, choose different net #########################################################################3
                    #if Crazyflie.Crazyflie.cfs_curr_pos camera - Crazyflie.Crazyflie.cfs_curr_pos net < value
                    #    if net == 0:
                    #        net = net+1
                    #    else:
                    #        net = net-1


                    #t1.append(Thread(target=track_object, args=(job_1[0][0], job_1[1][0], cf_spotted), daemon=True)) # launch net drone
                    t1.append(Thread(target=track_object, args=(job_1[0][net], job_1[1][net], cf_spotted), daemon=True)) ############################################

                else: # no spotting made but all cameras should land due to reaching time limit
                    cf_spotted = 0 # not actually a drone
                    stop_threads = True  # condition to stop all threads and land drones


                for thread in t1: # start threads for net drones
                    thread.start()


        rate.sleep()




    # Keep subscribers listening until script terminates
    #rospy.spin()
