#!/usr/bin/python3

# Test sending position to car

import rospy
from geometry_msgs.msg import PointStamped

rospy.init_node("test_desired_pos", anonymous=False)

des_publisher1 = rospy.Publisher("Car1/desired_pos", PointStamped, queue_size=2)
des_publisher2 = rospy.Publisher("Car2/desired_pos", PointStamped, queue_size=2)
des_publisher3 = rospy.Publisher("Car3/desired_pos", PointStamped, queue_size=2)

rospy.sleep(2)

msg1 = PointStamped()
msg1.point.x = -1
msg1.point.y = -1
msg1.point.z = 0

des_publisher1.publish(msg1)

rospy.sleep(2)

msg2 = PointStamped()
msg2.point.x = 1
msg2.point.y = -1
msg2.point.z = 0

des_publisher2.publish(msg2)

rospy.sleep(2)

msg3 = PointStamped()
msg3.point.x = -1
msg3.point.y = -0.4
msg3.point.z = 0

des_publisher3.publish(msg3)
