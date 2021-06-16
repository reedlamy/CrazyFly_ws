#!/usr/bin/python3
# Author: Reed Lamy, Purdue University, reedlamy@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Controller Script
# Set up mission
# using each Crazyflie's functions from the Crazyflie class

import rospy
import Crazyflie
import sys
import signal
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from crazyflie_driver.msg import NameArray


def tracks_callback(data):
    # get camera relative x,y,z
    global rel_x,rel_y,rel_z,timer

    rel_x = data.pose.pose.position.x
    rel_y = data.pose.pose.position.y
    rel_z = data.pose.pose.position.z

    timer = 0



# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == '__main__':

    rel_x = 0
    rel_y = 0
    rel_z = 0
    timer = 20

    qualisys_connected = True

    rospy.init_node('detection', anonymous=True)

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Get list of Crazyflie names from launch file that will be controlled
    cf_names = rospy.get_param("/cf_names")
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
        #rospy.spin()

    track_sub = rospy.Subscriber("/tracks", Odometry, tracks_callback)

    #cam_sub = rospy.Subscriber("/" + name + "/external_point", PointStamped, coord_callback)

    rate = rospy.Rate(10)

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

        #print(Crazyflie.Crazyflie.cfs_curr_pos)
        timer += 1

        if timer >= 20 : # no/lost detection
            rel_x = 0
            rel_y = 0
            rel_z = 0

        else: # Detection has been made
            camera = 1 # create a way to know which drone made the detection
            [x, y, z] = Crazyflie.Crazyflie.cfs_curr_pos[camera]

            potential_global_int




        print(' - ')
        print(' - ')
        #print(Crazyflie.Crazyflie.cfs_curr_pos)
        rate.sleep()