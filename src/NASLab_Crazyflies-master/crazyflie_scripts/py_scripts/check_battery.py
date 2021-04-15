#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Check battery of individual Crazyflie at frequency of battery topic publish rate

import rospy
import time
import sys
import signal
from matplotlib import pyplot as plt
from std_msgs.msg import Float32


# Handling CTRL+C from keyboard
def signal_handler(sig, frame):

    # Display battery charging plot
    plt.plot(time_arr, batt_percent_arr)
    plt.ylabel('Battery Percentage (%)')
    plt.xlabel('Time (seconds)')
    plt.title('Crazyflie Battery Charging over Time')
    plt.savefig('/home/naslab/crazyflie_ws/src/NASLab_Crazyflies/crazyflie_scripts/battery_graphs/CF'
                + cf_num + '_batt_charge.png')
    plt.show()
    sys.exit(0)


def battery_callback(msg):
    global batt_percent_arr

    # Get voltage from ROS msg
    vbat = msg.data
    min_vbat = 3.0
    max_vbat = 4.23
    battery_percent = ((vbat - min_vbat)/(max_vbat - min_vbat)) * 100
    batt_percent_arr.append(battery_percent)
    time_arr.append(time.time() - start)
    print("Battery Percentage: %.2f%%" % battery_percent)
    if battery_percent < 30:
        print("LOW BATTERY!")


if __name__ == '__main__':
    # Initialize ROS node 'battery
    rospy.init_node('battery', anonymous=True)

    uri = rospy.get_param("~uri")
    cf_num = uri[-2:]

    # Initialize array of battery charge to plot
    batt_percent_arr = []
    time_arr = []
    start = time.time()

    # Exit when CTRL+C is pressed
    signal.signal(signal.SIGINT, signal_handler)

    # Subscribe to Crazyflie's battery topic
    rospy.Subscriber("battery", Float32, battery_callback)
    rospy.spin()
