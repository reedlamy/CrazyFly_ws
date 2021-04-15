#!/usr/bin/python3
# Streaming 6Dof from QTM
# Make sure that new QTM Measurement is open before running

import asyncio
import xml.etree.ElementTree as ET
import rospy
import qtm
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import UpdateParams


def create_body_index(xml_string):
    # Extract a name to index dictionary from 6dof settings xml
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


async def main():
    global firstTransform
    firstTransform = True

    # Connect to qtm
    connection = await qtm.connect(ip)

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return
    else:
        print("QTM connected")

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    # Name of 6DOF Body in QTM
    wanted_body = "CF2"

    def on_packet(packet):
        global firstTransform

        info, bodies = packet.get_6d_euler()

        # Set header for new msg to be published
        msg.header.frame_id = worldFrame
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1

        if wanted_body is not None and wanted_body in body_index:
            # Extract one specific body
            wanted_index = body_index[wanted_body]
            position, rotation = bodies[wanted_index]

            if firstTransform:
                # Initialize kalman filter
                rospy.set_param("kalman/initialX", position[0])
                rospy.set_param("kalman/initialY", position[1])
                rospy.set_param("kalman/initialZ", position[2])
                update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
                rospy.set_param("kalman/resetEstimation", 1)
                update_params(["kalman/resetEstimation"])
                firstTransform = False
            else:
                # Assign position components to msg and publish msg
                msg.point.x = position[0]/1000
                msg.point.y = position[1]/1000
                msg.point.z = position[2]/1000
                pub.publish(msg)
                # print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))

        else:
            print("Not all bodies found")
            return

    # Initialize msg
    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame

    # Initialize publisher to 'external_position' topic (subscribed by Crazyflie)
    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)

    # Start streaming frames
    await connection.stream_frames(components=["6deuler"], on_packet=on_packet)

    # Wait asynchronously 15 seconds
    await asyncio.sleep(15)

    # Stop streaming
    await connection.stream_frames_stop()


if __name__ == "__main__":
    rospy.init_node('stream_6dof_example', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    ip = rospy.get_param("~ip")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params servcie")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)
    
    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
