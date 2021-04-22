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
from geometry_msgs.msg import PoseStamped  # PointStamped
from scipy.spatial.transform import Rotation as R


# Build dictionary with rigid body labels from QTM and their indices in the output stream
# Input: xml_string = output from QTM measurements
# Output: body_to_index = dictionary of body names and their indices in the QTM output
def create_body_index(xml_string):

    # Extract a name to index dictionary from 6dof settings xml
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


# Main function to run until node is killed
async def main():

    # Connect to qtm
    connection = await qtm.connect(ip)

    # Connection failed?
    if connection is None:

        rospy.logwarn("QTM failed to connect")
        return

    else:
        rospy.loginfo("QTM connected")

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    # Name of 6DOF Body in QTM
    wanted_bodies = cf_names

    # Callback function for each packet received from QTM
    def on_packet(packet):

        global firstPacket
        global start_record
        global tracking_frames_lost

        # Extract packet data in 6d euler form
        info, bodies = packet.get_6d_euler()

        #print(wanted_bodies)
        #print(all([cf in body_index for cf in wanted_bodies]))


        # Is every wanted body in the body index, the transformed output from QTM?
        if wanted_bodies is not None and all([cf in body_index for cf in wanted_bodies]):

            # Extract all bodies
            wanted_indices = [body_index[cf] for cf in wanted_bodies]


            # Check if actual position should start to be recorded
            #if start_record:
            if False:
                # Store actual position for each Crazyflie (open and close in case of abrupt ending (CTRL+C)
                x_pos = [bodies[wanted_indices[i]][0][0] / 1000 for i in range(len(wanted_indices))]
                y_pos = [bodies[wanted_indices[i]][0][1] / 1000 for i in range(len(wanted_indices))]
                xy_pos = [None]*(len(x_pos)+len(y_pos))
                xy_pos[::2] = x_pos
                xy_pos[1::2] = y_pos
                with open(log_location + 'cf_xy_pos_' + dt_string + '.csv', 'a', newline='') as file:
                    writer = csv.writer(file, quoting=csv.QUOTE_NONNUMERIC)
                    writer.writerow(xy_pos)

            #else:
            #    if rospy.get_param('record_position'):
            #        start_record = True



            # Iterate over every Crazyflie, sending them their individual positions
            for i in range(len(wanted_indices)):

                position = bodies[wanted_indices[i]][0]  # x, y, z (millimeters)
                rotation = bodies[wanted_indices[i]][1]  # roll (x axis), pitch (y axis), yaw (z axis) (degrees)

                # Did coordinate system initialize correctly in QTM for each Crazyflie?
                if firstPacket:

                    firstPacket = False

                    # Crazyflie coordinates incorrect if x axis is too large or small
                    if rotation[0] > 10 or rotation[0] < -10:

                        rospy.logerr("One of the Crazyflies' coordinate systems is incorrect")
                        rospy.signal_shutdown("One of the Crazyflies' coordinate systems is incorrect")
                        return

                # Assign world position components to msg (after converting to meters)

                #Point Stamped
                #pos_msg = PointStamped()
                #pos_msg.header.frame_id = worldFrame
                #pos_msg.header.stamp = rospy.Time.now()
                #pos_msg.header.seq += 1
                #pos_msg.point.x = position[0] / 1000
                #pos_msg.point.y = position[1] / 1000
                #pos_msg.point.z = position[2] / 1000

                # PoseStamped
                pos_msg = PoseStamped()
                pos_msg.header.frame_id = worldFrame
                pos_msg.header.stamp = rospy.Time.now()
                pos_msg.header.seq += 1
                pos_msg.pose.position.x = position[0]/1000
                pos_msg.pose.position.y = position[1]/1000
                pos_msg.pose.position.z = position[2]/1000


                # Assign orientations to msg (after converting from euler to quaternion)
                eul_rotation = R.from_euler('xyz', rotation, degrees=True)
                quat_rotation = eul_rotation.as_quat()

                pos_msg.pose.orientation.x = quat_rotation[0]
                pos_msg.pose.orientation.y = quat_rotation[1]
                pos_msg.pose.orientation.z = quat_rotation[2]
                pos_msg.pose.orientation.w = quat_rotation[3]


                # Don't publish NaN or Inf value in coordinate data from Qualisys
                if not any([math.isnan(coord) + math.isinf(coord) for coord in
                            [pos_msg.pose.position.x, pos_msg.pose.position.y, pos_msg.pose.position.z]]):
                    publishers[i].publish(pos_msg)
                    messages[i] = pos_msg

                    # Reset counter only if tracking was not already lost (ensure untracked CFs do not restart flying)
                    if tracking_frames_lost[i] <= tracking_frames_thresh:
                        tracking_frames_lost[i] = 0
                else:
                    publishers[i].publish(messages[i])  # Send last pos_msg command to keep Crazyflie in the air
                    tracking_frames_lost[i] += 1
                    if tracking_frames_lost[i] > tracking_frames_thresh:
                        # Could try to remove from wanted bodies but would mess up indices for publishers
                        # Notify run_mission of lost tracking so that Crazyflies can be stopped
                        # Any message on this topic commands run_mission to stop the CFs so use PointStamped for ease
                        lost_frames_publishers[i].publish(messages[i])
                # print("{} - Pos: {} - Rot: {}".format(wanted_bodies, position, rotation))

        # Possible error: Launch file and QTM names likely don't match
        else:

            rospy.logwarn("Not all bodies found")
            return

    # Initialize messages & publishers to 'external_position' topic (subscribed by Crazyflie)
    messages = []
    publishers = []
    lost_frames_publishers = []



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
        # pub = rospy.Publisher(wanted_bodies[j] + "/external_position", PointStamped, queue_size=3)
        # pub = rospy.Publisher("external_position", PointStamped, queue_size=3)
        pub = rospy.Publisher(wanted_bodies[j] + "/external_pose", PoseStamped, queue_size=3)
        publishers.append(pub)  # Array containing a publisher for each Crazyflie

        lost_frames_pub = rospy.Publisher(wanted_bodies[j] + "/lost_frames", PoseStamped, queue_size=1)
        #lost_frames_pub = rospy.Publisher(wanted_bodies[j] + "/lost_frames", PointStamped, queue_size=1)
        lost_frames_publishers.append(lost_frames_pub)
    
    # Start streaming frames
    await connection.stream_frames(components=["6deuler"], on_packet=on_packet)

    # Wait asynchronously until shutdown
    while not rospy.is_shutdown():
        await asyncio.sleep(0.1)

    # Stop streaming
    await connection.stream_frames_stop()


# Function to run when script is run as main script
if __name__ == "__main__":

    # Initialize ROS node for Qualisys stream with worldframe and IP address
    rospy.init_node('qualisys_cf_stream', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    ip = rospy.get_param("~ip")

    firstPacket = True
    start_record = False

    # Automatically compiles Crazyflie rigid body names from launch file into array
    # Note: Names must be equal to names in Qualisys (normally CF#)
    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')


    tracking_frames_lost = [0]*len(cf_names)
    tracking_frames_thresh = 10  # 4  # Need to send Crazyflie a position update within every 0.5 seconds


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
