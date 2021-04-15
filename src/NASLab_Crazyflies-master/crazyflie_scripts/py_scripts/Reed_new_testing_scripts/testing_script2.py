#!/usr/bin/python3

#
# Reed Lamy
#

import math
import rospy
from std_msgs.msg import Empty
import Crazyflie
import signal
from crazyflie_driver.msg import NameArray
from crazyflie_driver.msg import Position
from crazyflie_driver.srv import UpdateParams
from flight_path import flight_path
from geometry_msgs.msg import PointStamped


def flight_commands(cf, num_chargers, bt):
    batt_plot_flag = False


# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('test_code', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    qualisys_connected = False

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Get list of Crazyflie names from launch file that will be controlled
    cf_names = rospy.get_param("cf_names")
    cf_names = cf_names.split(',')

    # Initialize array so that each Crazyflie's position is available to every other Crazyflie
    Crazyflie.Crazyflie.cfs_curr_pos = [[0] * 3] * int(cf_names[-1][2:])

    # Initialize array to say all cfs are being tracked
    Crazyflie.Crazyflie.all_cfs_is_tracking = [True] * int(cf_names[-1][2:])

    for name in cf_names:
        if not name:
            rospy.logerr('Please include at least one Crazyflie')
            sys.exit()

        # Create Crazyflie instance based on body name in launch file
        idx = cf_names.index(name)  # Crazyflie's index in 'cf_names' array
        crazy = Crazyflie.Crazyflie(name, idx, qualisys_connected)

        # Subscribe to external_position topic output by 'qualisys_cf_stream.py'
        crazy.pos_subscriber = rospy.Subscriber(name + "/external_position", PointStamped, crazy.callback)
        # crazy.pos_subscriber = rospy.Subscriber(name + "/external_pose", PoseStamped, crazy.callback)

        # Check for successful subscription to external position topic
        if qualisys_connected:
            try:
                rospy.wait_for_message(name + "/external_position", PointStamped, timeout=1)
                # rospy.wait_for_message(name + "/external_pose", PoseStamped, timeout=1)
            except rospy.ROSException:
                rospy.logerr('Could not subscribe to ' + name + '/external_position message: Timeout')
                # rospy.logerr('Could not subscribe to ' + name + '/external_pose message: Timeout')
                sys.exit()
            except rospy.ROSInterruptException:
                rospy.logerr('USER INTERRUPTION')
                sys.exit()

            # Subscribe to battery topic output by Crazyflie driver
            crazy.batt_subscriber = rospy.Subscriber(name + "/battery", Float32, crazy.battery_callback)

            # Check for successful subscription to battery topic
            try:
                rospy.wait_for_message(name + "/battery", Float32, timeout=1)
            except rospy.ROSException:
                rospy.logerr('Could not subscribe to ' + name + '/battery message: Timeout')
                sys.exit()
            except rospy.ROSInterruptException:
                rospy.logerr('USER INTERRUPTION')
                sys.exit()


    rate = rospy.Rate(10)  # 10 hz


    #pub = rospy.Publisher("cmd_position", Position, queue_size=1)

    #stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    #stop_msg = Empty()

    #rospy.wait_for_service('update_params')
    #rospy.loginfo("found update_params service")
    #update_params = rospy.ServiceProxy('update_params', UpdateParams)

    #rospy.set_param("stabilizer/estimator", 2)
    #update_params(["stabilizer/estimator"])
    #rospy.set_param("kalman/resetEstimation", 1)
    #update_params(["kalman/resetEstimation"])
    #rospy.sleep(0.1)
    #rospy.set_param("kalman/resetEstimation", 0)
    #update_params(["kalman/resetEstimation"])
    #rospy.set_param("flightmode/posSet", 1)
    #update_params(["flightmode/posSet"])




    # take off
    while not rospy.is_shutdown():

        for i in range(300):

            takeoff(0.5)
            #goTo(0, 0, 0.5, 0, cf.cf_num)

    while not rospy.is_shutdown():
        while zDistance > 0:
            msg.x = 0  # xPoint
            msg.y = 0  # yPoint
            msg.z = zDistance
            msg.yaw = 0.0
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            rospy.loginfo("Landing")
            rospy.loginfo(msg.x)
            rospy.loginfo(msg.y)
            rospy.loginfo(msg.z)
            rospy.loginfo(msg.yaw)
            pub.publish(msg)
            rate.sleep()
            zDistance -= 0.05
        break
        stop_pub.publish(stop_msg)

