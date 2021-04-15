#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Thread to keep rospy spinning

import rospy


def rospy_spin():
    # Keep subscribers listening until script terminates
    rospy.spin()
