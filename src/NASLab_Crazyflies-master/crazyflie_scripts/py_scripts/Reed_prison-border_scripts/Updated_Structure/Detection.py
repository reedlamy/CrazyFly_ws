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
from geometry_msgs.msg import PoseStamped  # PointStamped
from crazyflie_driver.msg import NameArray

#from threading import Thread, Barrier
#from flight_commands_pb import flight_commands_pb

#s = Mid_Level()

# Charging Pad Position Subscriber Callback
# Input: data = data returned from charger position subscriber (charger_pos_subscriber),
# charger_idx = charging pad index

class Detection:

    def __init__(self, prefix, idx, qualisys_connected):

    def detection():

            for name in cf_names:
                if not name:
                    rospy.logerr('Please include at least one Crazyflie')
                    sys.exit()

                # Create Crazyflie instance based on body name in launch file
                idx = cf_names.index(name)  # Crazyflie's index in 'cf_names' array
                crazy = Crazyflie.Crazyflie(name, idx, qualisys_connected)


                # Subscribe to external_position topic output by 'qualisys_cf_stream.py'

                #crazy.pos_subscriber = rospy.Subscriber("/" + name + "/external_position", PointStamped, crazy.callback)
                crazy.pos_subscriber = rospy.Subscriber("/"+ name + "/external_pose", PoseStamped, cord_callback)
                #crazy.pos_subscriber = rospy.Subscriber("/external_position", PointStamped, crazy.callback)
                # crazy.pos_subscriber = rospy.Subscriber(name + "/external_pose", PoseStamped, crazy.callback)

                # Check for successful subscription to external position topic


                # Store instance to use later to start flight path threads
                crazy.pose.position

            while not rospy.is_shutdown():




    def cord_callback(self,data):




        # Keep subscribers listening until script terminates
        rospy.spin()



##################################################
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
from geometry_msgs.msg import PoseStamped, PointStamped
from crazyflie_driver.msg import NameArray


def camera_callback(data):
    # get camera relative x,y,z


def coord_callback(data):
    x = data.pose.position.x
    y = data.pose.position.x
    z = data.pose.position.x

# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == '__main__':

    qualisys_connected = True

    rospy.init_node('detection')

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Get list of Crazyflie names from launch file that will be controlled
    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')

    for name in cf_names:
        if not name:
            rospy.logerr('Please include at least one Crazyflie')
            sys.exit()

        # Create Crazyflie instance based on body name in launch file
        idx = cf_names.index(name)  # Crazyflie's index in 'cf_names' array
        crazy = Crazyflie.Crazyflie(name, idx, qualisys_connected)

        crazy.pos_subscriber = rospy.Subscriber("/" + name + "/external_pose", PoseStamped, coord_callback)

        rospy.spin()

    cam_sub = rospy.Subscriber("/" + name + "/external_point", PointStamped, coord_callback)

    while not rospy.is_shutdown():

        # Something that waits for camera dectection to be made

        # if camera detection is true, get cf number
            [x,y,z] = camera_callback()
            [x1,y1,z1] = coord_callback()

            potential_global_int = [x+x1,y+y1,z+z1]

            # Check if its within tolorance of all other drones
                # for drone in SOME DIRECTORY WITH ALL DRONE POSITIONS
                    # if MAG(potential_global_int - drone) < tolerance
                        # some sort of flag
                        # break
                    # if flag blah blah
                        # publish position


        rate.sleep()



