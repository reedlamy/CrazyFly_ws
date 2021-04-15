#!/usr/bin/python3
# CALCULATE LINEAR VELOCITY NODE

import rospy
import sys
import math
from geometry_msgs.msg import PoseStamped, PointStamped
from car.msg import pid
from scipy.spatial.transform import Rotation as R
from crazyflie_driver.msg import NameArray


def cur_callback(data, pad_idx):
	global cur_x
	global cur_y
	global cur_yaw
	# Get current(cur) position
	# Error: rospy.loginfo(rospy.get_caller_id() + " I heard %f", data)
	cur_x[pad_idx] = data.pose.position.x
	cur_y[pad_idx] = data.pose.position.y

	# Convert quaternion angles back to euler to command yaw (z-axis rotation)
	rotations = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
	if not any([math.isnan(orient) + math.isinf(orient) for orient in rotations]):
		quat_orientation = R.from_quat(rotations)
		eul_orientation = quat_orientation.as_euler('xyz', degrees=True)
		cur_yaw[pad_idx] = eul_orientation[2]  # Degrees


def des_callback(data, pad_idx):
	global des_x
	global des_y
	# Get desired(des) position
	# Error: rospy.loginfo(rospy.get_caller_id() + " I heard %f", data)
	des_x[pad_idx] = data.point.x
	des_y[pad_idx] = data.point.y


def lin_ang_update():
	lin_messages = [None]*len(pad_names)
	ang_messages = [None]*len(pad_names)

	# Need to move these to individual cars
	ang_kp = 0.015  # 0.12, 0.1
	ang_ki = 0.01  # 0.1, 0.01
	ang_kd = 0  # 0, 0
	lin_kp = 2  # 1.5, 1.5
	lin_ki = 1  # 0.1, 0.1
	lin_kd = 0.01  # 0.1, 0.05

	while not rospy.is_shutdown():
		for i in range(len(pad_names)):
			# Create Linear Velocity PID Message: Feedback
			lin_msg = pid()
			if des_x[i] is None or des_y[i] is None:
				lin_msg.fdbck = 0
			else:
				lin_msg.fdbck = math.sqrt(pow(des_y[i] - cur_y[i], 2) + pow(des_x[i] - cur_x[i], 2))
			lin_msg.setpoint = 0  # 0 difference between end position and goal position
			lin_msg.kp = lin_kp
			lin_msg.ki = lin_ki
			lin_msg.kd = lin_kd

			# Create Angular Velocity PID Message: Feedback
			ang_msg = pid()
			if des_x[i] is None or des_y[i] is None:
				ang_msg.fdbck = 0
			else:
				# Make error 0 when close
				if (lin_msg.fdbck - lin_msg.setpoint) < 0.15:
					lin_msg.fdbck = 0
					ang_msg.fdbck = 0
				else:
					# atan2 returns value between -pi and pi
					angle = math.degrees(math.atan2(des_y[i] - cur_y[i], des_x[i] - cur_x[i]))  # degrees
					angle_error = (angle - cur_yaw[i] + 180) % 360 - 180
					# print(f'Car{i+1} angle error = {angle_error} deg')
					if abs(angle_error) >= 20:
						lin_msg.fdbck = 0
						ang_msg.fdbck = angle_error
					else:
						ang_msg.fdbck = 0
			ang_msg.setpoint = 0  # 0 difference between end position and goal position
			ang_msg.kp = ang_kp
			ang_msg.ki = ang_ki
			ang_msg.kd = ang_kd
			ang_messages[i] = ang_msg
			lin_messages[i] = lin_msg
			# Publish to PID_vel node
			lin_publishers[i].publish(lin_msg)
			# Publish to PID_rot node
			ang_publishers[i].publish(ang_msg)
		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('lin_ang_vel', anonymous=False)
		rate = rospy.Rate(10)  # refresh rate

		# Wait until pad names have been formulated from user input and published to the pad_names topic
		# Uncomment if mission plan, comment if just cars
		pad_names_msg = rospy.wait_for_message("pad_names", NameArray, timeout=None)
		pad_names = pad_names_msg.names

		# Get pad names list from launch file
		# Comment if mission plan, uncomment if just cars
		# pad_names = rospy.get_param("/pad_names")
		# pad_names = pad_names.split(',')

		cur_x = [None] * len(pad_names)
		cur_y = [None] * len(pad_names)
		cur_yaw = [None] * len(pad_names)
		des_x = [None] * len(pad_names)
		des_y = [None] * len(pad_names)

		lin_publishers = []
		ang_publishers = []

		for pad in pad_names:
			# Check if first index of pad names is emtpy string ('') -> means no pad names defined
			if not pad:
				rospy.logwarn('No charging pads defined for mobile charger mission planning.')
				sys.exit()

			# Subscribe to get current position
			rospy.Subscriber("/charger_pos" + str(pad[-1]), PoseStamped, cur_callback, pad_names.index(pad))

			# Subscribe to get desired position
			rospy.Subscriber("Car" + str(pad[-1]) + "/desired_pos", PointStamped, des_callback, pad_names.index(pad))

			# Check for successful subscription to charger position topic
			try:
				first_charger_pos_msg = rospy.wait_for_message("/charger_pos" + str(pad[-1]), PoseStamped, timeout=1)
				cur_x[pad_names.index(pad)] = first_charger_pos_msg.pose.position.x
				cur_y[pad_names.index(pad)] = first_charger_pos_msg.pose.position.y

				# Convert quaternion angles back to euler to command yaw (z-axis rotation)
				first_quat_orientation = R.from_quat([first_charger_pos_msg.pose.orientation.x,
													  first_charger_pos_msg.pose.orientation.y,
													  first_charger_pos_msg.pose.orientation.z,
													  first_charger_pos_msg.pose.orientation.w])
				first_eul_orientation = first_quat_orientation.as_euler('xyz', degrees=True)
				cur_yaw[pad_names.index(pad)] = first_eul_orientation[2]
			except rospy.ROSException:
				rospy.logerr('Could not subscribe to /charger_pos' + str(pad[-1]) + ' message: Timeout')
				sys.exit()
			except rospy.ROSInterruptException:
				rospy.logerr('USER INTERRUPTION')
				sys.exit()

			# Create publishers
			publin = rospy.Publisher("Car" + str(pad[-1]) + "/pidv_ret", pid, queue_size=10)
			pubang = rospy.Publisher("Car" + str(pad[-1]) + "/pidr_ret", pid, queue_size=10)
			lin_publishers.append(publin)
			ang_publishers.append(pubang)

		lin_ang_update()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
