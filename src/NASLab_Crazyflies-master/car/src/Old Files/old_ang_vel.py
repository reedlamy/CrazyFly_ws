#!/usr/bin/env python3

# CALCULATE ANGULAR VELOCITY NODE

import rospy
import sys
import math
from geometry_msgs.msg import PoseStamped, PointStamped
from msg import pid

rospy.init_node('ang_vel', anonymous=False)


def cur_callback(data):

	# Get current(cur) position
	rospy.loginfo(rospy.get_caller_id() + " I heard %f", data.data)
	cur_x = data.pose.position.x
	cur_y = data.pose.position.y
	cur_yaw = data.pose.orientation.z  # Check that (z) represents yaw


def des_callback(data):
	# Get desired(des) position
	rospy.loginfo(rospy.get_caller_id() + " I heard %f", data.data)
	des_x = data.point.x
	des_y = data.point.y


def ang_update():
	pad_names = rospy.get_param("/pad_names")
	pad_names = pad_names.split(',')

	messages = []
	publishers = []

	charger_pos_subscribers = []

	for pad in pad_names:
		if not pad:
			continue

		# Subscribe to get current position
		rospy.Subscriber("/charger_pos" + str(pad[-1]), PoseStamped, cur_callback, pad_names.index(pad))

		# Subscribe to get desired position
		rospy.Subscriber("/desired_pos" + str(pad[-1]), PointStamped, des_callback, pad_names.index(pad))

		# Check for successful subscription to charger position topic
		try:
			rospy.wait_for_message("/charger_pos" + str(pad[-1]), PoseStamped, timeout=1)
		except rospy.ROSException:
			rospy.logerr('Could not subscribe to /charger_pos' + str(pad[-1]) + ' message: Timeout')
			sys.exit()
		except rospy.ROSInterruptException:
			rospy.logerr('USER INTERRUPTION')
			sys.exit()

		# Create PID Message: Feedback
		msg = pid()
		angle = math.atan2(des_pos.y - data.y, 2, des_pos.x - data.x)
		msg.fdbck = angle - cur_yaw
		messages.append(msg)

		# Publish to PID_rot node
		pubang = rospy.Publisher("pidr_ret" + str(pad[-1]), pid, queue_size=10)
		pubang.publish(msg)
		publishers.append(pubang)

		rospy.spin()


if __name__ == '__main__':
	try:
		ang_update()
	except rospy.ROSInteruptException:
		pass
