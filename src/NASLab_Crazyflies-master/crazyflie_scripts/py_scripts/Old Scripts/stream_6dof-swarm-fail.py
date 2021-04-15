#!/usr/bin/python3
# Streaming 6Dof from QTM
# Make sure that new QTM Measurement is open before running

import asyncio
import xml.etree.ElementTree as ET
import rospy
import qtm
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import UpdateParams
import numpy as np

def updateParams(prefix):
    rospy.wait_for_service(prefix + '/update_params')
    rospy.loginfo("Found " + prefix + " update_params service")

def create_body_index(xml_string):
    # Extract a name to index dictionary from 6dof settings xml
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


async def main():
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
##    wanted_body = "CF1"

    def on_packet(packet):
        global firstTransform
        
        info, bodies = packet.get_6d_euler()

        # Set header for new msg to be published
##        msg.header.frame_id = worldFrame
##        msg.header.stamp = rospy.Time.now()
##        msg.header.seq += 1

##        if wanted_body is not None and wanted_body in body_index:
        if cf_names is not None and all(np.isin(cf_names, np.array(list(body_index.keys())))):
            # Extract one specific body
##            wanted_index = body_index[wanted_body]
##            position, rotation = bodies[wanted_index]

            # Extract bodies
            cf_indices = [body_index[name] for name in cfnames]
            position, rotation = bodies[cf_indices]

            if firstTransform:
                # Initialize kalman filter
                rospy.set_param("kalman/initialX", position[0])
                rospy.set_param("kalman/initialY", position[1])
                rospy.set_param("kalman/initialZ", position[2])
                update_params1(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
                update_params2(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
                update_params3(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
##                update_params4(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
                update_params5(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
                rospy.set_param("kalman/resetEstimation", 1)
                update_params1(["kalman/resetEstimation"])
                update_params2(["kalman/resetEstimation"])
                update_params3(["kalman/resetEstimation"])
##                update_params4(["kalman/resetEstimation"])
                update_params5(["kalman/resetEstimation"])
                firstTransform = False
            else:
                # PROBABLY CAUSE ERROR WITH 2+ CFs
                # Assign position components to msg and publish msg
                for i in range(len(msg)):
                    msg1.point.x = position[0]/1000
                    msg1.point.y = position[1]/1000
                    msg1.point.z = position[2]/1000
                    pub1.publish(msg1)
                    msg2.point.x = position[0]/1000
                    msg2.point.y = position[1]/1000
                    msg2.point.z = position[2]/1000
                    pub2.publish(msg2)
                    msg3.point.x = position[0]/1000
                    msg3.point.y = position[1]/1000
                    msg3.point.z = position[2]/1000
                    pub3.publish(msg3)
##                    msg4.point.x = position[0]/1000
##                    msg4.point.y = position[1]/1000
##                    msg4.point.z = position[2]/1000
##                    pub4.publish(msg4)
                    msg5.point.x = position[0]/1000
                    msg5.point.y = position[1]/1000
                    msg5.point.z = position[2]/1000
                    pub5.publish(msg5)
##                print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))

        else:
            print("Not all bodies found")
            return

    # Initialize msg
    msg1 = PointStamped()
    msg2 = PointStamped()
    msg3 = PointStamped()
##    msg4 = PointStamped()
    msg5 = PointStamped()
    msg1.header.seq = 0
    msg1.header.stamp = rospy.Time.now()
    msg1.header.frame_id = worldFrame
    msg2.header.seq = 0
    msg2.header.stamp = rospy.Time.now()
    msg2.header.frame_id = worldFrame
    msg3.header.seq = 0
    msg3.header.stamp = rospy.Time.now()
    msg3.header.frame_id = worldFrame
##    msg4.header.seq = 0
##    msg4.header.stamp = rospy.Time.now()
##    msg4.header.frame_id = worldFrame
    msg5.header.seq = 0
    msg5.header.stamp = rospy.Time.now()
    msg5.header.frame_id = worldFrame

    # Initialize publisher to 'external_position' topic (subscribed by Crazyflie)
##    pub = rospy.Publisher("CF1/external_position", PointStamped, queue_size=1)
    pub1 = rospy.Publisher("CF1/external_position", PointStamped, queue_size=1)
    pub2 = rospy.Publisher("CF2/external_position", PointStamped, queue_size=1)
    pub3 = rospy.Publisher("CF3/external_position", PointStamped, queue_size=1)
##    pub4 = rospy.Publisher("CF4/external_position", PointStamped, queue_size=1)
    pub5 = rospy.Publisher("CF5/external_position", PointStamped, queue_size=1)

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

##    rospy.wait_for_service('CF1/update_params')
##    rospy.loginfo("found update_params servcie")
##    update_params = rospy.ServiceProxy('CF1/update_params', UpdateParams)

    # Name of 6DOF Bodies in QTM
    cf_names = np.array(["CF1", "CF2", "CF3", "CF5"])
    for name in cf_names:
        updateParams(name)
    
    update_params1 = rospy.ServiceProxy('CF1/update_params', UpdateParams)
    update_params2 = rospy.ServiceProxy('CF2/update_params', UpdateParams)
    update_params3 = rospy.ServiceProxy('CF3/update_params', UpdateParams)
##    update_params4 = rospy.ServiceProxy('CF4/update_params', UpdateParams)
    update_params5 = rospy.ServiceProxy('CF5/update_params', UpdateParams)
    
    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
