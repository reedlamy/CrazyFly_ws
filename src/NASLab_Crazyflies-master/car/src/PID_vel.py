#!/usr/bin/python3
# title           :PID.py
# description     :python pid controller
# author          :Reeve Lambert & Caner Durmusoglu(https://github.com/ivmech/ivPID/blob/master/PID.py)
# Rev date        :6/12/2019
# version         :0.1
# notes           :
# python_version  :3.6
# ==============================================================================

"""This PID Controller is largely based upon the PID Controller developed by
Caner Durmusoglu(https://github.com/ivmech/ivPID/blob/master/PID.py) with """
import time
import rospy
from car.msg import pid
from crazyflie_driver.msg import NameArray

global pidv


class PID:
    """PID Controller"""

    def __init__(self, p=0.2, i=0.0, d=0.0):
        """Intialization/Constructor function that sets the proportional, integral, and derivative gains as well as the
        current time"""
        self.Kp = p  # set proportional gain
        self.Ki = i  # set the integral gain
        self.Kd = d  # set the derivative gain
        self.sample_time = 0.00  # set a default sample time for user to overwrite later if needed (real time update)
        self.windup_guard = 0.00  # Initialize windup guard (max value ITerm can be)
        self.current_time = time.time()  # set the current time as the current time
        self.last_time = self.current_time  # set the initial last time to the current time
        self.last_error = 0.00  # initialize a variable to track previous error values
        self.error = 0.0  # initialize a variable to track current PID error
        self.ITerm = 0.0  # Initialize the term for integral gan
        self.PTerm = 0.0  # Initialize the term for Proportional gain
        self.DTerm = 0.0  # Initialize the term for derivative gain
        self.SetPoint = 0.0  # Initialize the desired value
        self.int_error = 0.0  # output the integration error
        self.output = 0.0  # Initialize the output variable
        self.feedback_value = 0.0  # Intialize the feedback value that will be updated by the ROS subscription service

    def clear(self):
        """Clears PID computations and coefficients"""
        # Clear outputs and goals
        self.SetPoint = 0.0
        self.last_error = 0.0
        # Clear Terms
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        # Clear Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        # Clear Output
        self.output = 0.0

    def update(self):
        """Calculates PID value for given reference feedback
         math: u(t) = K_p e(t) + K_i/int_{0}^{t} e(t)dt + K_d {de}/{dt}
        """
        # Error Calc
        self.error = self.feedback_value - self.SetPoint  # calculate error as difference between goal val & current val
        # if self.error < 0.1:
        #     self.error = 0

        delta_error = self.error - self.last_error

        # Dt Calc
        self.current_time = time.time()  # Set the current time
        delta_time = 0.1  # Find change in time in seconds (1/rospy.rate)

        # Calculate Output
        if delta_time >= self.sample_time:  # If enough time has elapsed (sample time) then update pid output
            # Calculate Terms
            self.PTerm = self.Kp * self.error  # Calculate Proportional term
            self.ITerm += self.error * delta_time   # Calculate Integral Term

            # Determine if the integral term is within the range of -windup_guard to + wind_guard
            if self.ITerm < -self.windup_guard:  # If it is less than negative guard
                self.ITerm = -self.windup_guard * self.Ki  # set integral term to  negative guard
            elif self.ITerm > self.windup_guard:  # IF it is more than guard
                self.ITerm = self.windup_guard * self.Ki  # set to guard
            else:
                tempval = self.ITerm  # temp store Iterm
                self.ITerm = tempval * self.Ki  # set to guard

            # Calculate Differential term if time has passed
            self.DTerm = 0.0
            if delta_time > 0:  # If any time has passed
                self.DTerm = delta_error / delta_time  # calculate derivative term

            # Remember last time and last error for next calculation
            self.last_time = self.current_time  # Set last time as the time at which update last ran (i.e. now)
            self.last_error = self.error  # Save the error for use upon the next update

            # Compile full output from terms and gain coefficients
            if abs(self.error) < 750:  # If we are close to the target
                self.output = self.PTerm + self.ITerm + (self.Kd * self.DTerm)
            else:
                self.output = -1.0  # full speed

# =================================================================================================
# The following is ROS Code


def callback(data, pad_idx):
    """This function will run anytime that data is received from the pidv topic  The function shall:
    1. log the feedback data that was recieved
    2. update all values of the controller as per the message data (i.e. kp, setpoint, etc)
    3. update the pid controller
    4. reformulate the output pid message for the pidv topic
    5. re-publish the message"""
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.fdbck)  # log heard feedback values

    # Update PID constants
    pidv[pad_idx].SetPoint = data.setpoint  # Update(if any) the setpoint (goal value) for the pid controller
    pidv[pad_idx].windup_guard = data.wndup  # update (if any) the windup guard value for the pid controller
    pidv[pad_idx].Kd = data.kd  # Update (if any) the derivative gain constant
    pidv[pad_idx].Ki = data.ki  # Update (if any) the integral gain constant
    pidv[pad_idx].Kp = data.kp  # Update (if any) the proportional gain constant
    pidv[pad_idx].feedback_value = data.fdbck  # Update the distance to the waypoint
    pidv[pad_idx].ITerm = data.iterm  # take in the reported iterm

    # Update the PID output
    pidv[pad_idx].update()  # Update the velocity pid controller


def pidupdate():
    """This script will subscribe to the velocity PID topic and updat ethe PID controller at the rate it is published"""
    # Set rosnodes up and listener value

    messages = []

    while not rospy.is_shutdown():
        for i in range(len(pad_names)):

            # Assemble the response message
            msg = pid()  # Create a message as defined by the pid message file
            msg.setpoint = pidv[i].SetPoint  # report on the current setpoint value
            msg.laster = pidv[i].last_error  # report the last error the pid controller had
            msg.err = pidv[i].error  # report on the error the PID controller calculated
            msg.wndup = pidv[i].windup_guard  # Put the windup guard into the message
            msg.kp = pidv[i].Kp  # put the commanded proportional gain constant back into the message
            msg.kd = pidv[i].Kd  # put the commanded derivative gain constant back into the message
            msg.ki = pidv[i].Ki  # put the commanded integral gain constant back into the message
            msg.out = pidv[i].output  # Put the PID controller output command into the message
            msg.fdbck = pidv[i].feedback_value  # Put the recieved feedback value back into the message
            msg.iterm = pidv[i].ITerm  # update the iterm value in the msg
            msg.pterm = pidv[i].PTerm  # update the pterm value in the msg
            msg.dterm = pidv[i].DTerm  # update the dterm value in the msg

            messages.append(msg)

            # Publish the message
            publishers[i].publish(msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        # Define ROS Publishing parameters
        rospy.init_node('PID_vel', anonymous=False)  # Establish the node
        rate = rospy.Rate(10)  # refresh rate of GPS data is slow at 10 Hz

        # Wait until pad names have been formulated from user input and published to the pad_names topic
        # Uncomment if mission plan, comment if just cars
        pad_names_msg = rospy.wait_for_message("pad_names", NameArray, timeout=None)
        pad_names = pad_names_msg.names

        # Get pad names list from launch file
        # Comment if mission plan, uncomment if just cars
        # pad_names = rospy.get_param("/pad_names")
        # pad_names = pad_names.split(',')

        publishers = []
        pidv = []

        for pad in pad_names:
            # Check if first index of pad names is emtpy string ('') -> means no pad names defined
            if not pad:
                rospy.logwarn('No charging pads defined for mobile charger mission planning.')
                sys.exit()

            # Subscribe to get PID linear input from lin_ang_vel.py
            rospy.Subscriber("Car" + str(pad[-1]) + "/pidv_ret", pid, callback, pad_names.index(pad))

            # Setup publisher of pid via pidv topic
            pubpid = rospy.Publisher("Car" + str(pad[-1]) + "/pidv", pid, queue_size=10)
            publishers.append(pubpid)

            # Set controllers
            pidv.append(PID())  # Set the PID controller up

        pidupdate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
