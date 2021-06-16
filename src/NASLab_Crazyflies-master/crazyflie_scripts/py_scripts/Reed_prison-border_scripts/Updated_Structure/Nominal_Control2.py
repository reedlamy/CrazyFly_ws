#!/usr/bin/python3
# Author: Reed Lamy, Purdue University, rlamy@purdue.edu
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Nominal Controller Script
# Execute sequence of actions for the mission defined in crazyflie library
# using each Crazyflie's functions from the Crazyflie class

import rospy
import sys
import signal
import math
import time
import flight_commands_pb2
from flight_commands_pb2 import circle_1_init, circle_2_init, track_object
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped  # PointStamped
from crazyflie_driver.msg import NameArray
from threading import Thread, Barrier
import threading
import Crazyflie
#from threading import Thread, Barrier
#from flight_commands_pb import flight_commands_pb


# Charging Pad Position Subscriber Callback
# Input: data = data returned from charger position subscriber (charger_pos_subscriber),
# charger_idx = charging pad index

#class Mid_level:

#def __init__(self):



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

    nominal_flight([Job_1_inst,Job_1_nums],[group_1_inst,group_1_nums],[group_2_inst,group_2_nums],[Job_3_inst,Job_3_nums],group_cent_pos,cf_nums)

def nominal_flight(job_1,group_1,group_2,job3,cent_pos,cf_nums):
    t = []
    t1 = []
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
        thread.start()

    start_time = time.time()

    while time.time() < start_time + 11:
        spotted = False

    else:
        spotted = True


    if spotted: # condition to say that CF spotted intruder
        stop_threads = True # condition to stop all threads and land drones

        # subscribe to something telling info on drone being spotted
        # ie drone_inst and number
        tracker_inst = group_2[0][0][0]
        cf_spotted = group_2[1][0][0]

        t1.append(Thread(target=track_object, args=(job_1[0][0], job_1[1][0], cf_spotted), daemon=True))

        for thread in t1:
            thread.start()


        #emergency_response(job_1,tracker_inst,cf_spotted)



#def emergency_response(job_1,tracker_inst,tracker_num): # will take all net crazyflies and the drone that made the detection

    # generate function that finds closest net launcher available, running through all drones under job 1
    # for now, just say net is first drone in job1


    #t.append(Thread(target=track_object, args=(tracker_inst,tracker_num,tracker_num),daemon=True))
    #t.append(Thread(target=track_object, args=(job_1[0][0],job_1[1][0],tracker_num),daemon=True))

    #for thread in t:
        #thread.join()









