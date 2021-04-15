#!/usr/bin/python3

import rospy
import math
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams


if __name__ == '__main__':
    rospy.init_node('qualisys_hover', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    rate = rospy.Rate(10)  # 10 hz

    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = 0.0
    msg.y = 0.0
    msg.z = 0.0
    msg.yaw = 0.0

    pub = rospy.Publisher("cmd_position", Position, queue_size=1)

    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)
        
    rospy.set_param("stabilizer/estimator", 2)
    update_params(["stabilizer/estimator"])
    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])
    rospy.set_param("flightmode/posSet", 1)
    update_params(["flightmode/posSet"])

    # take off
    while not rospy.is_shutdown():
        zDistance = 0.5  # meters
        takeoff_time = math.ceil(20 * zDistance)  # 2 seconds/meter
        for i in range(takeoff_time):
            msg.x = 0.0
            msg.y = 0.0
            msg.yaw = 0.0
            msg.z = (i/takeoff_time)*zDistance
            rospy.loginfo("Taking off")
            rospy.loginfo(msg.x)
            rospy.loginfo(msg.y)
            rospy.loginfo(msg.z)
            rospy.loginfo(msg.yaw)
            now = rospy.get_time()
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
            
        hover_time = 30  # 3 seconds
        for j in range(hover_time):
            msg.x = 0.0
            msg.y = 0.0
            msg.yaw = 0.0
            msg.z = zDistance
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            rospy.loginfo("Hovering")
            rospy.loginfo(msg.x)
            rospy.loginfo(msg.y)
            rospy.loginfo(msg.z)
            rospy.loginfo(msg.yaw)
            now = rospy.get_time()
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
        break

    # go to x: -0.5 y: 0.5
    # xPoint = -0.5
    # yPoint = -0.5
    # start = rospy.get_time()
    # while not rospy.is_shutdown():
    #     msg.x = xPoint
    #     msg.y = yPoint
    #     msg.yaw = 0.0
    #     msg.z = zDistance
    #     now = rospy.get_time()
    #     if (now - start > 3.0):
    #         break
    #     msg.header.seq += 1
    #     msg.header.stamp = rospy.Time.now()
    #     rospy.loginfo("Going to Position")
    #     rospy.loginfo(msg.x)
    #     rospy.loginfo(msg.y)
    #     rospy.loginfo(msg.z)
    #     rospy.loginfo(msg.yaw)
    #     pub.publish(msg)
    #     rate.sleep()

    # Land
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

    


