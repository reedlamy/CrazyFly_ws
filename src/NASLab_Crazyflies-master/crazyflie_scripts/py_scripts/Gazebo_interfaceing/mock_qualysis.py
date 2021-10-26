#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Streaming 6Dof from QTM
# Make sure that new QTM Measurement is open before running
#
# This code is adapted from the code provided by Qualisys for 6DOF tracking
# Link: https://github.com/qualisys/qualisys_python_sdk/blob/master/examples/stream_6dof_example.py

import asyncio
import xml.etree.ElementTree as ET
import rospy
import qtm
import csv
import math
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


def callback(data,i):
    global ext_x, ext_y, ext_z, ext_rotx, ext_roty, ext_rotz, ext_rotw
    # Quaternians

    # Reset current external position variables
    ext_x = data.pose.position.x
    ext_y = data.pose.position.y
    ext_z = data.pos.position.z
    ext_rotx = data.pose.orientation.x
    ext_roty = data.pose.orientation.y
    ext_rotz = data.pose.orientation.z
    ext_rotw = data.pose.orientation.w

# Main function to run until node is killed

async def main():
    wanted_bodies = cf_names

    # PoseStamped
    pos_msg = PoseStamped()
    pos_msg.header.frame_id = worldFrame
    pos_msg.header.stamp = rospy.Time.now()
    pos_msg.header.seq += 1
    pos_msg.pose.position.x = ext_x
    pos_msg.pose.position.y = ext_y
    pos_msg.pose.position.z = ext_z
    pos_msg.pose.orientation.x = ext_rotx
    pos_msg.pose.orientation.y = ext_roty
    pos_msg.pose.orientation.z = ext_rotz
    pos_msg.pose.orientation.w = ext_rotw


    # Initialize messages & publishers to 'external_position' topic (subscribed by Crazyflie)
    messages = []
    publishers = []




    # Iterate over all CFs in launch file
    for j in range(len(wanted_bodies)):

        msg = PoseStamped()  # PointStamped: Header header; Point point (float64 x; float64 y; float64 z; float64 yaw)
        # msg = PoseStamped()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = worldFrame
        messages.append(msg)  # Array containing a message for each Crazyflie

        print(wanted_bodies)

        # Publisher name preceded by 'CF#/' due to grouping in launch file
        pub = rospy.Publisher(wanted_bodies[j] + "/external_position", PoseStamped, queue_size=3)
        #pub = rospy.Publisher("external_position", PointStamped, queue_size=3)
        # pub = rospy.Publisher(wanted_bodies[j] + "/external_pose", PoseStamped, queue_size=3)
        publishers.append(pub)  # Array containing a publisher for each Crazyflie

    
    # Start streaming frames #####################################################################################################################################3
    #await connection.stream_frames(components=["6deuler"], on_packet=on_packet)

    # Wait asynchronously until shutdown
    while not rospy.is_shutdown():
        await asyncio.sleep(0.1)

    # Stop streaming################################################################################################################################
    #await connection.stream_frames_stop()


# Function to run when script is run as main script
if __name__ == "__main__":

    # Initialize ROS node for Qualisys stream with worldframe and IP address
    rospy.init_node('qualisys_cf_stream', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    firstPacket = True
    start_record = False

    # Automatically compiles Crazyflie rigid body names from launch file into array
    # Note: Names must be equal to names in Qualisys (normally CF#)
    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')

    for i in range(len(cf_names)):
        rospy.Subscriber("firefly/odometry_sensor/pose", PoseStamped,i, callback)
        # Assign world position components to msg (after converting to meters)


    #log_location = '/home/reed/Crazyfly_ws/src/NASLab_Crazyflies/crazyflie_scripts/static_cf_actual_pos_logs/'

    now = datetime.now()
    dt_string = now.strftime("%m-%d-%Y_%H:%M")

    csv_xheader = [name + ' X' for name in cf_names]
    csv_yheader = [name + ' Y' for name in cf_names]
    csv_header = [None]*(len(csv_xheader)+len(csv_yheader))
    csv_header[::2] = csv_xheader
    csv_header[1::2] = csv_yheader
    #with open(log_location + 'cf_xy_pos_' + dt_string + '.csv', 'w', newline='') as file:
        #writer = csv.writer(file, quoting=csv.QUOTE_NONNUMERIC)
        #writer.writerow(csv_header)

    # Run asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
