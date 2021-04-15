#!/usr/bin/env python

# For Charging Pad 1
# Implement on Car

import rospy
import sys
from mobileCharger_motors import *
from autonomy.msg import pid


# SET MOTOR SPEED AND MOVE CAR
def lin_callback(data):
	global lin_speed
	# Error: rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
	lin_speed = data.out


def ang_callback(data):
	global ang_speed
	# Error: rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
	ang_speed = data.out


def move():
	while not rospy.is_shutdown():
		# Input motor voltage
		l_speed = lin_speed + ang_speed
		r_speed = lin_speed - ang_speed

		if l_speed > 0:
			M3A = 1  # FL forward
			M3B = 0
			M2B = 1  # BL forward
			M2A = 0
		else:
			M3B = 1  # FL backward
			M3A = 0
			M2A = 1  # BL backward
			M2B = 0
		if r_speed > 0:
			M4A = 1  # FR forward
			M4B = 0
			M1B = 1  # BR forward
			M1A = 0
		else:
			M4B = 1  # FR backward
			M4A = 0
			M1A = 1  # BR backward
			M1B = 0

		motor_pins = [M4B, M3B, M4A, M2B, M1B, M1A, M2A, M3A]

		# Bound to positive number
		l_speed = abs(l_speed)
		r_speed = abs(r_speed)

		# Upper bound is 1 but limit to 0.75 because battery current draw can exceed the limit and turn off the pi
		l_speed = min(0.6, l_speed)
		r_speed = min(0.6, r_speed)

		if l_speed < 0.2:
			l_speed = 0
		if r_speed < 0.2:
			r_speed = 0
	
		setSpeed(l_speed, r_speed)
		shiftout(motor_pins, 1/1000000)  # 1/hz)


if __name__ == '__main__':
	try:
		rospy.init_node('car_move', anonymous=True)

		rospy.Subscriber("pidv", pid, lin_callback)
		rospy.Subscriber("pidr", pid, ang_callback)
		first_pidv_msg = rospy.wait_for_message("pidv", pid, timeout=None)
		lin_speed = first_pidv_msg.out
		rospy.loginfo(lin_speed)
		first_pidr_msg = rospy.wait_for_message("pidr", pid, timeout=1)
		ang_speed = first_pidr_msg.out
		rospy.loginfo(ang_speed)

		hz = 10
		rate = rospy.Rate(hz)

		move()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
