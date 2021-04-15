#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Controller Script
# Execute sequence of actions for the mission defined in flight path
# using each Crazyflie's functions from the Crazyflie class

import rospy
import Crazyflie
import sys
import signal
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped  # PoseStamped
from crazyflie_driver.msg import NameArray
from threading import Thread, Barrier
from flight_commands import flight_commands


# Charging Pad Position Subscriber Callback
# Input: data = data returned from charger position subscriber (charger_pos_subscriber),
# charger_idx = charging pad index
def charger_callback(data, charger_idx):

    ros_hz = 10
    ros_rate = 1/ros_hz  # ROS topic publish rate (10 hz)

    if len(Crazyflie.all_charger_pos[charger_idx]) == 3:
        x_vel = (data.point.x - Crazyflie.all_charger_pos[charger_idx][0]) / ros_rate
        y_vel = (data.point.y - Crazyflie.all_charger_pos[charger_idx][1]) / ros_rate
        Crazyflie.all_charger_vel[charger_idx] = [x_vel, y_vel, 0]

    # Replace previous charger position in all_charger_pos
    Crazyflie.all_charger_pos[charger_idx] = [data.point.x, data.point.y, data.point.z]


# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)


# Main function to execute when script is called from launch file
if __name__ == '__main__':

    # Use to test w/o Qualisys
    qualisys_connected = True

    # Initialize ROS node for the controller
    rospy.init_node('run_crazyflies', anonymous=True)

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Get list of Crazyflie names from launch file that will be controlled
    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')

    pad_names = rospy.get_param("/pad_names")
    pad_names = pad_names.split(',')

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


        crazy.pos_subscriber = rospy.Subscriber("/"+ name + "/external_position", PointStamped, crazy.callback)
        #crazy.pos_subscriber = rospy.Subscriber("/external_position", PointStamped, crazy.callback)
        # crazy.pos_subscriber = rospy.Subscriber(name + "/external_pose", PoseStamped, crazy.callback)

        # Check for successful subscription to external position topic
        if qualisys_connected:
            try:
                rospy.wait_for_message("/" + name + "/external_position", PointStamped, timeout=1)
                #rospy.wait_for_message("/external_position", PointStamped, timeout=1)
                # rospy.wait_for_message(name + "/external_pose", PoseStamped, timeout=1)
            except rospy.ROSException:
                print("/" + name + "/external_position")
                rospy.logerr('Could not subscribe to ' + name + '/external_position message: Timeout')
                # rospy.logerr('Could not subscribe to ' + name + '/external_pose message: Timeout')
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
    t = []

    # Barriers to synchronize threads
    bt = Barrier(len(cf_names))


    # Update class variables if first instance
    crazy_instances[0].global_update(cf_names, pad_names)


    for crazy_inst in crazy_instances:
        # Append thread for each Crazyflie to array t (each executes 'flight_path' function)
        t.append(Thread(target=flight_commands, args=(crazy_inst, 0, bt), daemon=True))

    # Start all threads
    for thread in t:
        thread.start()

    # Keep subscribers listening until script terminates
    rospy.spin()
