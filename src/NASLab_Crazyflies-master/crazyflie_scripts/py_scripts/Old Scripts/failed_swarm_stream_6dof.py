#!/usr/bin/python3
# Streaming 6Dof from QTM
# Make sure that new QTM Measurement is open before running

import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import rospy
import qtm
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import UpdateParams
from threading import Thread


class CrazyflieStream:
    def __init__(self, prefix):
        self.prefix = prefix
        self.ip = rospy.get_param("~ip")

        worldFrame = rospy.get_param("~worldFrame", "/world")

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found " + prefix + " update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)

        # Initialize publisher to 'external_position' topic (subscribed by Crazyflie)
        self.pub = rospy.Publisher(prefix + "external_position", PointStamped, queue_size=1)

        # Initialize msg
        self.msg = PointStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame


    def create_body_index(xml_string):
        # Extract a name to index dictionary from 6dof settings xml
        xml = ET.fromstring(xml_string)

        body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_to_index[body.text.strip()] = index

        return body_to_index


    async def stream_position():
        def on_packet(packet):
            self.info, self.bodies = packet.get_6d_euler()

            # Set header for new msg to be published
            self.msg.header.frame_id = worldFrame
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.seq += 1

            if self.wanted_body is not None and self.wanted_body in self.body_index:
                # Extract one specific body
                self.wanted_index = self.body_index[self.wanted_body]
                self.position, self.rotation = self.bodies[self.wanted_index]

                if self.firstTransform:
                    # Initialize kalman filter
                    rospy.set_param("kalman/initialX", position[0])
                    rospy.set_param("kalman/initialY", position[1])
                    rospy.set_param("kalman/initialZ", position[2])
                    self.update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
                    rospy.set_param("kalman/resetEstimation", 1)
                    self.update_params(["kalman/resetEstimation"])
                    firstTransform = False
                else:
                    # Assign position components to msg and publish msg
                    self.msg.point.x = self.position[0]/1000
                    self.msg.point.y = self.position[1]/1000
                    self.msg.point.z = self.position[2]/1000
                    self.pub.publish(self.msg)
                    # print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))

            else:
                print("Not all bodies found")
                return


        # ERROR b/c not in async func def!
        # Connect to qtm
        self.connection = await qtm.connect(self.ip)

        # Connection failed?
        if self.connection is None:
            print("Failed to connect")
            sys.exit()
        else:
            print("QTM connected")
        self.firstTransform = True

        # Get 6dof settings from qtm
        self.xml_string = await connection.get_parameters(parameters=["6d"])
        self.body_index = create_body_index(self.xml_string)

        # Name of 6DOF Body in QTM
        self.wanted_body = prefix

        # Start streaming frames
        await connection.stream_frames(components=["6deuler"], on_packet=on_packet)

        # Wait asynchronously 15 seconds
        await asyncio.sleep(15)
        
        # Stop streaming
        await connection.stream_frames_stop()


def run(cf):
    # Run asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(cf.stream_position())

    # Stop streaming
    #await connection.stream_frames_stop()


if __name__ == "__main__":
    rospy.init_node('stream_6dof_example', anonymous=True)

    # cf_names = ['CF1', 'CF2', 'CF3', 'CF4', 'CF5']
    cf_names = ['CF1']
    for name in cf_names:
        cf = CrazyflieStream(name)
        t = Thread(target=run, args=(cf,))
        t.start()
