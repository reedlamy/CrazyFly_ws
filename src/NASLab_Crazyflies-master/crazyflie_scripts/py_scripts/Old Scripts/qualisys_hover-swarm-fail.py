#!/usr/bin/python3

import rospy
import math
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from geometry_msgs.msg import PointStamped
import numpy as np


def updateParams(prefix):
    rospy.wait_for_service(prefix + '/update_params')
    rospy.loginfo("Found " + prefix + " update_params service")
    
##    rospy.set_param(prefix + "/stabilizer/estimator", 2)
##    update_params([prefix + "/stabilizer/estimator"])
##    rospy.set_param(prefix + "/kalman/resetEstimation", 1)
##    update_params([prefix + "/kalman/resetEstimation"])
##    rospy.sleep(0.1)
##    rospy.set_param(prefix + "/kalman/resetEstimation", 0)
##    update_params([prefix + "/kalman/resetEstimation"])
##    rospy.set_param(prefix + "/flightmode/posSet", 1)
##    update_params([prefix + "/flightmode/posSet"])
    

def callback(data):
    global msg
    
    cf_name_index = cf_names.index(name)
    msg[cf_name_index] = data
    

if __name__ == '__main__':
    rospy.init_node('qualisys_hover', anonymous=True)
##    cf1 = Crazyflie("CF1")
    worldFrame = rospy.get_param("~worldFrame", "/world")
    cf_names = np.array(["CF1", "CF2", "CF3", "CF5"])

    rate = rospy.Rate(10) # 10 hz

##    msg = Position()
##    msg.header.seq = 0
##    msg.header.stamp = rospy.Time.now()
##    msg.header.frame_id = worldFrame
##    msg.x = 0.0
##    msg.y = 0.0
##    msg.z = 0.0
##    msg.yaw = 0.0
##
##    pub = rospy.Publisher("CF1/cmd_position", Position, queue_size=1)
##
##    stop_pub = rospy.Publisher("CF1/cmd_stop", Empty, queue_size=1)
##    stop_msg = Empty()

##    rospy.wait_for_service('CF1/update_params')
##    rospy.loginfo("found update_params service")
##    update_params = rospy.ServiceProxy('CF1/update_params', UpdateParams)

    for name in cf_names:
        updateParams(name)
        rospy.Subscriber(name + "/external_position", PointStamped, callback)

    update_params1 = rospy.ServiceProxy('CF1/update_params', UpdateParams)
    update_params2 = rospy.ServiceProxy('CF2/update_params', UpdateParams)
    update_params3 = rospy.ServiceProxy('CF3/update_params', UpdateParams)
##    update_params4 = rospy.ServiceProxy('CF4/update_params', UpdateParams)
    update_params5 = rospy.ServiceProxy('CF5/update_params', UpdateParams)


    rospy.set_param("stabilizer/estimator", 2)
    rospy.set_param("kalman/resetEstimation", 1)
    update_params1(["stabilizer/estimator", "kalman/resetEstimation"])
    update_params2(["stabilizer/estimator", "kalman/resetEstimation"])
    update_params3(["stabilizer/estimator", "kalman/resetEstimation"])
##    update_params4(["stabilizer/estimator", "kalman/resetEstimation"])
    update_params5(["stabilizer/estimator", "kalman/resetEstimation"])
    rospy.set_param("kalman/resetEstimation", 0)
    rospy.set_param("flightmode/posSet", 1)
    update_params1(["kalman/resetEstimation", "flightmode/posSet"])
    update_params2(["kalman/resetEstimation", "flightmode/posSet"])
    update_params3(["kalman/resetEstimation", "flightmode/posSet"])
##    update_params4(["kalman/resetEstimation", "flightmode/posSet"])
    update_params5(["kalman/resetEstimation", "flightmode/posSet"])
        
##    rospy.set_param("stabilizer/estimator", 2)
##    update_params(["stabilizer/estimator"])
##    rospy.set_param("kalman/resetEstimation", 1)
##    update_params(["kalman/resetEstimation"])
##    rospy.sleep(0.1)
##    rospy.set_param("kalman/resetEstimation", 0)
##    update_params(["kalman/resetEstimation"])
##    rospy.set_param("flightmode/posSet", 1)
##    update_params(["flightmode/posSet"])

##    rospy.Subscriber("CF1/external_position", PointStamped, callback)
    while not rospy.is_shutdown():
        # Do something
        msg_array = 1
    rospy.spin()

    # take off
##    while not rospy.is_shutdown():
##        zDistance = 1.0 # meters
##        takeoff_time = math.ceil(20 * zDistance) # 2 seconds/meter
##        for i in range(takeoff_time):
##            msg.x = 0.0
##            msg.y = 0.0
##            msg.yaw = 0.0
##            msg.z = (i/takeoff_time)*zDistance
##            rospy.loginfo("Taking off")
##            rospy.loginfo(msg.x)
##            rospy.loginfo(msg.y)
##            rospy.loginfo(msg.z)
##            rospy.loginfo(msg.yaw)
##            now = rospy.get_time()
##            msg.header.seq += 1
##            msg.header.stamp = rospy.Time.now()
##            pub.publish(msg)
##            rate.sleep()
##            
##        hover_time = 30 # 3 seconds
##        for j in range(hover_time):
##            msg.x = 0.0
##            msg.y = 0.0
##            msg.yaw = 0.0
##            msg.z = zDistance
##            msg.header.seq += 1
##            msg.header.stamp = rospy.Time.now()
##            rospy.loginfo("Hovering")
##            rospy.loginfo(msg.x)
##            rospy.loginfo(msg.y)
##            rospy.loginfo(msg.z)
##            rospy.loginfo(msg.yaw)
##            now = rospy.get_time()
##            msg.header.seq += 1
##            msg.header.stamp = rospy.Time.now()
##            pub.publish(msg)
##            rate.sleep()
##        break

    # go to x: 1.0 y: 1.0
##    xPoint = 1.0
##    yPoint = 1.0
##    start = rospy.get_time()
##    while not rospy.is_shutdown():
##        msg.x = xPoint
##        msg.y = yPoint
##        msg.yaw = 0.0
##        msg.z = zDistance
##        now = rospy.get_time()
##        if (now - start > 3.0):
##            break
##        msg.header.seq += 1
##        msg.header.stamp = rospy.Time.now()
##        rospy.loginfo("Going to Position")
##        rospy.loginfo(msg.x)
##        rospy.loginfo(msg.y)
##        rospy.loginfo(msg.z)
##        rospy.loginfo(msg.yaw)
##        pub.publish(msg)
##        rate.sleep()

    # Land
##    while not rospy.is_shutdown():
##        while zDistance > 0:
##            msg.x = 0 #xPoint
##            msg.y = 0 #yPoint
##            msg.z = zDistance
##            msg.yaw = 0.0
##            msg.header.seq += 1
##            msg.header.stamp = rospy.Time.now()
##            rospy.loginfo("Landing")
##            rospy.loginfo(msg.x)
##            rospy.loginfo(msg.y)
##            rospy.loginfo(msg.z)
##            rospy.loginfo(msg.yaw)
##            pub.publish(msg)
##            rate.sleep()
##            zDistance -= 0.05
##        break
    
##    stop_pub.publish(stop_msg)

    


