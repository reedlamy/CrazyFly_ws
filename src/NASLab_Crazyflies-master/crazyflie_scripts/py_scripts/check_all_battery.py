#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Check instantaneous battery of all Crazyflies
# Make sure all Crazyflies being checked are on and launch file is correct

import rospy
import os
import sys
import signal
from std_msgs.msg import Float32
from crazyflie_driver.srv import Stop
from sys import exit


# Handling CTRL+C from keyboard
def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('all_battery', anonymous=True)

    signal.signal(signal.SIGINT, signal_handler)

    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')

    stopServices = []
    for name in cf_names:
        rospy.wait_for_service(name + '/stop')
        stopServices.append(rospy.ServiceProxy(name + '/stop', Stop))

    min_vbat = 3.0
    max_vbat = 4.23
    for name in cf_names:
        try:
            msg = rospy.wait_for_message(name + "/battery", Float32, timeout=3)
            vbat = msg.data
            battery_percent = ((vbat - min_vbat) / (max_vbat - min_vbat)) * 100
            print(name + " Battery Percentage: %.2f%%" % battery_percent)
        except rospy.ROSException:
            print(name + ": could not get battery percentage")

    rospy.spin()
