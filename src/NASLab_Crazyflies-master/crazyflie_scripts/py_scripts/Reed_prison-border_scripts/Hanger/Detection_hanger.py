#!/usr/bin/python3
# Author: Reed Lamy, Purdue University, reedlamy@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Controller Script
# Set up mission
# using each Crazyflie's functions from the Crazyflie class

# USE FOR CF 1 CAMERA SYS


import rospy
import Crazyflie
import sys
import signal
from std_msgs.msg import Float32, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from crazyflie_driver.msg import NameArray
import math
from scipy.spatial.transform import Rotation as R


def tracks_callback(data):
    # get camera relative x,y,z
    global rel_x,rel_y,rel_z,timer,timer_2

    rel_x = data.pose.pose.position.x
    rel_y = data.pose.pose.position.y
    rel_z = data.pose.pose.position.z

    timer = 0
    timer_2 += 1

def position_pub_callback(data):
    adv_info = PointStamped()
    rate1 = rospy.Rate(10)

    adv_info.point.x = data[0]
    adv_info.point.y = data[1]
    adv_info.point.z = data[2]
    adv_info.header.seq = data[3]

    pub_global_adv.publish(adv_info)
    rate1.sleep()

def pub_flag_callback(data1):
    flag1 = Int16()
    rate2 = rospy.Rate(5)

    flag1.data = data1

    pub_flag.publish(flag1)
    rate2.sleep()

def cf_position_callback(data):
    # Reset current external position variables
    global cf_x, cf_y, cf_z, cf_yaw
    cf_x = data.pose.position.x
    cf_y = data.pose.position.y
    cf_z = data.pose.position.z

    r = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    rot = r.as_euler('xyz', degrees=True)
    cf_yaw = rot[2]

def cf_check(pos): # check for other CF's and if its within mission area
    global check_cfs_flag, flag, cf_id

    # drawing a rectangle around drone in x-y plane with proper orientation
    x_1 = pos[0]+ math.cos(math.radians(cf_yaw+16))
    x_2 = pos[0]+ math.cos(math.radians(cf_yaw-16))
    x_3 = pos[0]+ math.cos(math.radians(cf_yaw+196))
    x_4 = pos[0]+ math.cos(math.radians(cf_yaw+148))

    y_1 = pos[1]+ math.sin(math.radians(cf_yaw+16))
    y_2 = pos[1]+ math.sin(math.radians(cf_yaw-16))
    y_3 = pos[1]+ math.sin(math.radians(cf_yaw+196))
    y_4 = pos[1]+ math.sin(math.radians(cf_yaw+148))

    # getting vectors around the box
    v_1 = [(x_2 - x_1), (y_2 - y_1)]
    v_2 = [(x_3 - x_2), (y_3 - y_2)]
    v_3 = [(x_4 - x_3), (y_4 - y_3)]
    v_4 = [(x_1 - x_4), (y_1 - y_4)]


    for i in range(len(Crazyflie.Crazyflie.cfs_curr_pos)):
        # creating vectors from corner points of rectangle to drone in question
        d_1 = [(Crazyflie.Crazyflie.cfs_curr_pos[i][0] - x_1), (Crazyflie.Crazyflie.cfs_curr_pos[i][1] - y_1)]
        d_2 = [(Crazyflie.Crazyflie.cfs_curr_pos[i][0] - x_2), (Crazyflie.Crazyflie.cfs_curr_pos[i][1] - y_2)]
        d_3 = [(Crazyflie.Crazyflie.cfs_curr_pos[i][0] - x_3), (Crazyflie.Crazyflie.cfs_curr_pos[i][1] - y_3)]
        d_4 = [(Crazyflie.Crazyflie.cfs_curr_pos[i][0] - x_4), (Crazyflie.Crazyflie.cfs_curr_pos[i][1] - y_4)]


        # cross product
        c1 = d_1[0]*v_1[1]-v_1[0]*d_1[1]
        c2 = d_2[0]*v_2[1]-v_2[0]*d_2[1]
        c3 = d_3[0]*v_3[1]-v_3[0]*d_3[1]
        c4 = d_4[0]*v_4[1]-v_4[0]*d_4[1]


        if c1 > 0 and c2 > 0 and c3 > 0 and c4 > 0 and abs(pos[2]-Crazyflie.Crazyflie.cfs_curr_pos[i][2])<0.27: # drone is within 3d rectangle of error
            check_cfs_flag = 1
            break
        elif abs(pos[0])>1.5 or abs(pos[1])>2 or pos[3]>2 or pos[3]<0.2: # check if its within mission area
            check_cfs_flag = 1
            break
        else:
            check_cfs_flag = 0 # it is good to go ahead and check other drones then capture

    if check_cfs_flag == 0: # proper detection has been made
        flag = 1
        pub_flag_callback(flag)
        pub_cf_id_callback(cf_id)
        position_pub_callback(pos)
        print("Adversary Spotted")

    else: # no detection, still publish flag
        flag = 0
        pub_flag_callback(flag)
        print("CF Spotted")


# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)

def pub_cf_id_callback(data2):
    cfid = Int16()
    rate2 = rospy.Rate(5)

    cfid.data = data2

    pub_cf_id.publish(cfid)
    rate2.sleep()


if __name__ == '__main__':

    check_cfs_flag = 1 # flag for when a detection is made, check once to see if its the other CF's
    flag = 0 # flag for detection being made to be published
    cf_x = 0
    cf_y = 0
    cf_z = 0
    cf_yaw = 0
    rel_x = 0
    rel_y = 0
    rel_z = 0
    timer = 21 # counter that goes to 0 when a detection is made/updated, if too high, detection is old and is lost
    timer_2 = 0 # counter that is increased when a detection is made and maintained, used to reject quick false detections
    tack_id = 0
    cf_id = 1
    potential_global_int = [0, 0, 0]

    qualisys_connected = True

    rospy.init_node('detection', anonymous=True)

    pub_global_adv = rospy.Publisher("global_adv", PointStamped, queue_size=1)

    pub_flag = rospy.Publisher("Intruder_flag", Int16, queue_size=1)

    pub_cf_id = rospy.Publisher("CF_CAM_ID", Int16, queue_size=1)

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Get list of Crazyflie names from launch file that will be controlled
    cf_names = rospy.get_param("/cf_names2")
    cf_names = cf_names.split(',')

    Crazyflie.Crazyflie.cfs_curr_pos = [[0] * 3] * int(cf_names[-1][2:])
    Crazyflie.Crazyflie.all_cfs_is_tracking = [True] * int(cf_names[-1][2:])
    crazy_instances = []

    for name in cf_names:
        if not name:
            rospy.logerr('Please include at least one Crazyflie')
            sys.exit()

        # Create Crazyflie instance based on body name in launch file
        idx = cf_names.index(name)  # Crazyflie's index in 'cf_names' array
        crazy = Crazyflie.Crazyflie(name, idx, qualisys_connected)

        crazy.pos_subscriber = rospy.Subscriber("/" + name + "/external_pose", PoseStamped, crazy.callback)

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

        crazy_instances.append(crazy)


    track_sub = rospy.Subscriber("/tracks", Odometry, tracks_callback)

    cf_sub = rospy.Subscriber("/CF1/external_pose", PoseStamped, cf_position_callback)

    #cam_sub = rospy.Subscriber("/" + name + "/external_point", PointStamped, coord_callback)

    rate = rospy.Rate(10)

    #rospy.spin()

    while not rospy.is_shutdown():

        # Something that waits for camera detection to be made
        # if camera detection is true, get cf idx number
            #[x,y,z] = Crazyflie.Crazyflie.cfs_curr_pos[i]
            #[x1,y1,z1] = coord_callback()

            #potential_global_int = [x+x1,y+y1,z+z1]

            # Check if its within tolerance of all other drones
                # for drone in SOME DIRECTORY WITH ALL DRONE POSITIONS
                    # if MAG(potential_global_int - drone) < tolerance
                        # some sort of flag
                        # break
                    # if flag blah blah
                        # publish position

        timer += 1

        if timer >= 7 : # no/lost detection
            rel_x = 0
            rel_y = 0
            rel_z = 0
            timer_2 = 0
            potential_global_int=[0,0,0]
            check_cfs_flag = 0
            flag = 0
            pub_flag_callback(flag)

        else: # Detection has been made and is not just noise
            if timer_2>=5:
                #camera = 0 # create a way to know which drone made the detection
                #camera_pos = Crazyflie.Crazyflie.cfs_curr_pos[camera]

                # turn drone position and info from camera into a global position
                potential_global_int = [cf_x+math.cos(math.radians(cf_yaw))*rel_x-math.sin(math.radians(cf_yaw))*rel_y,cf_y+math.sin(math.radians(cf_yaw))*rel_x+math.cos(math.radians(cf_yaw))*rel_y,cf_z+rel_z,cf_id]

                # Check fiesability of detection
                cf_check(potential_global_int)

            else:
                flag = 0
                pub_flag_callback(flag)

        rate.sleep()