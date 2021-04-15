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
import sys
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import NameArray
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

    # Name of charging pad bodies in QTM
    wanted_bodies = pad_names

    # Callback function for each packet received from QTM
    def on_packet(packet):

        # Extract packet data in 6d euler form
        info, bodies = packet.get_6d_euler()

        # Is every wanted body in the body index, the transformed output from QTM?
        if wanted_bodies is not None and all([pad in body_index for pad in wanted_bodies]):

            # Extract all bodies
            wanted_indices = [body_index[pad] for pad in wanted_bodies]

            # Iterate over every charging pad, sending them their individual positions
            for i in range(len(wanted_indices)):

                position = bodies[wanted_indices[i]][0]  # x, y, z (millimeters)
                rotation = bodies[wanted_indices[i]][1]  # roll (x axis), pitch (y axis), yaw (z axis) (degrees)

                # Assign world position components to msg (after converting to meters)
                messages[i].header.frame_id = worldFrame
                messages[i].header.stamp = rospy.Time.now()
                messages[i].header.seq += 1
                messages[i].pose.position.x = position[0] / 1000
                messages[i].pose.position.y = position[1] / 1000
                messages[i].pose.position.z = position[2] / 1000
                eul_rotation = R.from_euler('xyz', rotation, degrees=True)
                quat_rotation = eul_rotation.as_quat()
                messages[i].pose.orientation.x = quat_rotation[0]
                messages[i].pose.orientation.y = quat_rotation[1]
                messages[i].pose.orientation.z = quat_rotation[2]
                messages[i].pose.orientation.w = quat_rotation[3]
                publishers[i].publish(messages[i])
                # print("{} - Pos: {}".format(wanted_bodies[i], position))

        # Possible error: Launch file and QTM names likely don't match
        else:

            rospy.logerr('Not all charging pad bodies found.')
            rospy.signal_shutdown('Not all charging pad bodies found.')
            return

    # Initialize messages & publishers to 'charger_pos#' topic (subscribed in hover script)
    messages = []
    publishers = []

    # Iterate over all charging pad names in launch file
    for j in range(len(wanted_bodies)):

        msg = PoseStamped()  # PointStamped: Header header; Point point (float64 x; float64 y; float64 z)
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = worldFrame
        messages.append(msg)  # Array containing a message for each charging pad

        # Publisher name followed by charging pad number
        pub = rospy.Publisher("charger_pos" + str(wanted_bodies[j][-1]), PoseStamped, queue_size=3)
        publishers.append(pub)  # Array containing a publisher for each charging pad

    # Start streaming frames
    await connection.stream_frames(components=["6deuler"], on_packet=on_packet)

    # Wait asynchronously until shutdown
    while not rospy.is_shutdown():
        await asyncio.sleep(0.1)

    # Stop streaming
    await connection.stream_frames_stop()


# Function to run when script is run as main script
if __name__ == "__main__":

    # Initialize ROS node for charging pad Qualisys stream with worldframe and IP address
    rospy.init_node('qualisys_charging_pad_stream', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    ip = rospy.get_param("~ip")

    # Automatically compiles charging pad body names from launch file into array
    # Note: Names must be equal to names in Qualisys (normally Pad#)
    # Comment for 'crazyflie_scripts' package, uncomment for 'car' package
    # pad_names = rospy.get_param("/pad_names")
    # pad_names = pad_names.split(',')

    # Wait until message is received from run_mission with pad_names array (topic = pad_names)
    #Uncomment for 'crazyflie_scripts' package, comment for 'car' package
    pad_names_msg = rospy.wait_for_message("pad_names", NameArray, timeout=None)
    pad_names = pad_names_msg.names

     #if not pad_names:
     #    rospy.loginfo('No charging pads specified')
     #    sys.exit()

    # Run asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
