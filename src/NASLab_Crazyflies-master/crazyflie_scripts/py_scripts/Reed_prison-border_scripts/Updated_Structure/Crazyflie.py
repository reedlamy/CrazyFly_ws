#!/usr/bin/python3
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Crazyflie Swarm Controller Module
# Define actions such as takeoff, hover, goTo, move, goToSequence, moveSequence, land, switchLand
# Publishes positions at 10 Hz to each Crazyflie and achieves closed loop control
#
# This code implements some features that are included in Whoenig's scripts on GitHub,
# especially the Crazyflie ROS driver
# Link: https://github.com/whoenig/crazyflie_ros

import rospy
import time
import sys
import math
import queue
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Barrier, BrokenBarrierError, Event
from scipy.spatial.transform import Rotation as R
from crazyflie_scripts.msg import camera_msg
# from scipy.optimize import linear_sum_assignment  # Used in testing multiple switchLand
avoid_ros_namespace_conventions = True

# Class with properties describing each Crazyflie and functions to control each Crazyflie
class Crazyflie:

    # Initialize variables to be updated later
    # These variables must be called with 'Crazyflie.variable' instead of 'self.variable' to change the value for
    # all objects
    cf_names = []
    pad_names = []
    num_chargers = 0
    all_charger_pos = [[0, 0, 0]]
    all_charger_vel = [[0, 0, 0]]
    chargers_occupied = {}
    finished_command = []
    all_cfs_is_tracking = []
    cfs_curr_pos = [[0]*3]
    event_array = []
    num_threads = 0
    bt = Barrier(1)

    # FIFO Queue to pass add_func and add_args between threads ([func, args], [func, args], etc)
    q = queue.Queue()

    # Parameters to set for each new Crazyflie instance
    def __init__(self, prefix, idx, qualisys_connected):



        self.prefix = prefix  # Body name from launch file, i.e. CF1
        self.cf_idx = idx  # Crazyflie's index in 'cf_names' array

        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.hz = 10
        self.rate = rospy.Rate(self.hz)  # ROS topic publish rate (10 hz)
        self.vz = .5  # Vertical velocity (m/s)
        self.vy = 30 # Yaw angular velocity (deg/s)
        self.vh = 0.2 # Horizontal velocity (m/s)

        # used for tracking objects
        self.y_dir_tt = 0
        self.x_dir_tt = 0
        self.t_y = 0
        self.t_x = 0
        self.tg_yaw_t = 0
        self.tracker_flag = 0

        # Space limits (avoid leaving Qualisys MoCap System FOV)
        bound_tol = 0.5  # To account for slight overshoots
        self.zbounds = [0.0, 2.5]
        #work_width = rospy.get_param("work_width")
        #work_length = rospy.get_param("work_length")
        work_width = 10
        work_length = 10

        self.xbounds = [-work_width/2 - bound_tol, work_width/2 + bound_tol]
        self.ybounds = [-work_length/2 - bound_tol, work_length/2 + bound_tol]

        service_count = 0
        update_params_count = 0

        if qualisys_connected:
            while service_count <= 1:
                # Find ROS service 'update params' for CF instance


                try:
                    rospy.wait_for_service('/'+ prefix + '/update_params', timeout=3)
                    rospy.loginfo("Found " + prefix + " update_params service")
                    self.update_params = rospy.ServiceProxy('/'+ prefix + '/update_params', UpdateParams, persistent=True)
                    break
                except:
                    rospy.logwarn('Could not find update_params service')
                    service_count += 1
                    continue
                    # sys.exit()

            if service_count >= 2:
                sys.exit()

            # Reset controllers
            rospy.set_param(prefix + "/stabilizer/estimator", 2)
            # rospy.sleep(0.1)
            rospy.set_param(prefix + "/kalman/resetEstimation", 1)
            # rospy.sleep(0.5)
            rospy.set_param(prefix + "/locSrv/extQuatStdDev", 0.04)

            while update_params_count <= 2:
                # Try to send update of parameters
                try:
                    self.update_params(["stabilizer/estimator", "kalman/resetEstimation","locSrv/extQuatStdDev"])
                    break
                except:
                    rospy.logwarn("Could not update 1st parameters")
                    update_params_count += 1
                    continue
                    # sys.exit()

            if update_params_count >= 3:
                sys.exit()

            rospy.sleep(0.1)  # System delay
            update_params_count = 0

            # Finish reset of controllers and set flight mode to position hold
            rospy.set_param(prefix + "/kalman/resetEstimation", 0)
            # rospy.sleep(0.1)
            rospy.set_param(prefix + "/flightmode/posSet", 1)
            # rospy.sleep(0.5)

            while update_params_count <= 2:
                # Try to send update of parameters
                try:
                    self.update_params(["kalman/resetEstimation", "flightmode/posSet"])
                    break
                except:
                    rospy.logwarn("Could not update 2nd parameters")
                    update_params_count += 1
                    continue
                    # sys.exit()

            if update_params_count >= 3:
                sys.exit()

            ## NEW Update of std dev

            #rospy.set_param(prefix + "/locSrv/extQuatStdDev", 0.04)

            #while update_params_count <= 2:
                # Try to send update of parameters
            #    try:
            #        self.update_params("locSrv/extQuatStdDev")
            #        break
            #    except:
            #        rospy.logwarn("Could not update 3rd parameter")
            #        update_params_count += 1
            #        continue
                    # sys.exit()

            #if update_params_count >= 3:
            #    sys.exit()

            # close persistent service connection
            self.update_params.close()

        # Initialize publisher to command position to Crazyflies
        self.pub = rospy.Publisher(prefix+"/cmd_position", Position, queue_size=2)
        #self.pub = rospy.Publisher("cmd_position", Position, queue_size=2)

        # Initial msg
        # Position (crazyflie_driver): Header header; float32 x; float32 y; float32 z; float32 yaw
        self.msg = Position()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.worldFrame
        self.msg.x = 0.0
        self.msg.y = 0.0
        self.msg.z = 0.0
        self.msg.yaw = 0.0

        # Initialize publisher to send stop command to Crazyflie (turns off all motors)
        self.stop_pub = rospy.Publisher(prefix+"/cmd_stop", Empty, queue_size=1)
        #self.stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)

        self.stop_msg = Empty()
        self.pos_subscriber = None
        self.batt_subscriber = None

        # Initialize position variables
        self.ext_x = 0
        self.ext_y = 0
        self.ext_z = 0
        self.ext_yaw = 0 # was commented out
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.cf_seq = [self.x, self.y, self.z, self.yaw]  # Initial goTo sequence array
        self.seq_idx = 0  # Index to know when Crazyflie failed

        self.in_takeoff = False  # In state of takeoff?
        self.land_rate = 1  # Initial ratio to decrease altitude during landing
        self.emergency_land = False  # In state of emergency land?
        self.battery_percent = 100  # Battery percentage
        self.low_battery_thresh = 0  # 10-20  # Low battery percentage threshold
        self.low_battery = False  # Is CF in low battery state?

        # Initialize array of battery charge to plot
        self.batt_percent_arr = []
        self.time_arr = []
        self.batt_start = time.time()

        # Initialize elapsed time variables
        self.start = 0.0
        self.elap_time = 0.0

        # self.tracking_frames_lost = 0  # Count of frames where external position is not tracked
        # self.tracking_frames_thresh = 4  # Threshold of untracked frames (need position sent every 0.5 seconds)
        self.is_tracking = True  # Determine if tracking based on count and threshold
        self.readjust_count = 0  # Count of landing readjustments

        self.cf_num = int(self.prefix[2:])  # Crazyflie's number
        self.is_charging = False  # In state of charging?
        self.desired_switch_pos = [0, 0]  # Initialize desired switch position for switchLand function
        self.start_land_time = 0

    # Update global Crazyflie parameters
    @classmethod
    def global_update(cls, cf_name_array, pad_name_array):

        cls.cf_names = cf_name_array
        cls.pad_names = pad_name_array

        # Number of charging pads in space
        if not cls.pad_names:
            cls.num_chargers = 0
        else:
            cls.num_chargers = len(cls.pad_names)

        cls.all_charger_pos = [[0, 0, 0]] * cls.num_chargers  # Initialize empty chargers' positions until tracked
        cls.all_charger_vel = [[0, 0, 0]] * cls.num_chargers  # Initialize empty chargers' velocities until tracked

        # Initialize dictionary to describe which chargers are occupied by which CF {'Pad#': CF#}
        # 0 = unoccupied, # = CF number, -# = CF currently landing or not charged long enough (2 min)
        cls.chargers_occupied = {pad: 0 for pad in cls.pad_names}

        # Array to track which Crazyflies have finished to prevent overwriting of data
        # Size = # of CFs
        cls.finished_command = [False] * len(cls.cf_names)

        # Condition for threads to wait on each other (includes locks)
        # Used for flying CF to notify the correct charging CF to take off during switchLand
        # Size = # of charging pads
        # Index = index of pad number in pad_names
        cls.event_array = [Event()] * len(cls.pad_names)

        # Barriers to synchronize threads
        cls.num_threads = len(cls.cf_names)
        cls.bt = Barrier(cls.num_threads)

    # Take off
    # Input: z = takeoff height (meters), sync = wait on all CFs (default False)
    def takeoff(self, z, sync=False):

        self.in_takeoff = True

        # Reset charger occupied value if takeoff commanded for Crazyflie and is done charging (is_charging = False)
        if not self.is_charging:
            for p in range(len(self.all_charger_pos)):
                pos = self.all_charger_pos[p]
                if Crazyflie.dist_2D(pos, [self.ext_x, self.ext_y]) <= 0.1:
                    Crazyflie.chargers_occupied[self.pad_names[p]] = 0  # Reset since taking off will make the pad free
                    break

        x = self.ext_x
        y = self.ext_y
        yaw = self.ext_yaw
        self.elap_time = 0.0
        self.start = time.time()

        while not rospy.is_shutdown():
            counter = 0

            # Repeat action to increase altitude while z position is less than commanded z
            while self.ext_z < z and self.is_tracking and not self.is_charging:



                # Set commanded positions to take off directly above current position
                self.msg.x = x
                self.msg.y = y
                self.msg.yaw = yaw # was set = 0.0
                # self.msg.yaw = 0.0

                # Increase z if less than commanded z
                if counter*self.vz <= z:
                    self.msg.z = counter*self.vz

                # Command same z if should have achieved commanded z based on count and velocity
                else:
                    self.msg.z = z

                # Log info
                #rospy.loginfo("Taking off")
                #rospy.loginfo(self.msg.x)
                #rospy.loginfo(self.msg.y)
                #rospy.loginfo(self.msg.z)
                #rospy.loginfo(self.msg.yaw)



                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
                counter += 1/self.hz  # Increase counter for next loop
                self.elap_time = time.time() - self.start

                # Set state to emergency land if taking too long to reach position
                if self.elap_time > 10:
                    print(f'CF{self.cf_num}: Takeoff')
                    self.emergency_land = True
                    self.emerg_land('goal')
                    break

                # Don't use for mission planning
                # Set state to emergency land if not all CFs tracked
                # elif not all(self.all_cfs_is_tracking):
                #     self.emergency_land = True
                #     self.emerg_land('tracking')

            # If required to wait on all other CFs
            if sync:

                # Current CF has finished command
                Crazyflie.finished_command[self.cf_idx] = True
                self.in_takeoff = False

                # Hover while waiting for other CFs
                while not all(self.finished_command):
                    self.hover(1/self.hz)

                # Barrier to wait for all CFs
                bt_idx = 0
                try:
                    bt_idx = self.bt.wait()
                    self.bt.reset()
                except BrokenBarrierError:
                    pass

                # One CF resets finished_command array values
                if bt_idx == 0:
                    Crazyflie.finished_command = [False] * self.num_threads

                # Barrier to wait for CFs again (prevents finished_command from being overwritten)
                try:
                    self.bt.wait()
                    self.bt.reset()
                except BrokenBarrierError:
                    pass

            break

    # Hover
    # Input: duration = hovering time (seconds)
    def hover(self, duration):

        # Skip if in emergency land state
        if self.emergency_land:
            return

        # Set hover position
        x = self.msg.x
        y = self.msg.y
        yaw = self.msg.yaw
        z = self.msg.z

        while not rospy.is_shutdown():
            hover_time = duration
            start = time.time()
            # Repeat command to stay in same position for hover time
            while (time.time() - start) < hover_time and not self.is_charging:

                # Display positions
                # for i in range(len(self.cfs_curr_pos)):
                #     print('CF{}: {}, {}, {}'.format(i+1, self.cfs_curr_pos[i][0], self.cfs_curr_pos[i][1],
                #                                     self.cfs_curr_pos[i][2]))

                # Set position to command
                self.msg.x = x
                self.msg.y = y
                self.msg.yaw = yaw
                self.msg.z = z
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()

                # Log info
                # rospy.loginfo("Hovering")
                # rospy.loginfo(self.msg.x)
                # rospy.loginfo(self.msg.y)
                # rospy.loginfo(self.msg.z)
                # rospy.loginfo(self.msg.yaw)

                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()

                # Don't use for mission planning
                # Set state to emergency land if not tracked
                # if not all(self.all_cfs_is_tracking):
                #     self.emergency_land = True
                #     self.emerg_land('tracking')
                #     break

                # Don't use for mission planning
                # Go into low battery land which will switchland if necessary with the nearest charger
                # if self.battery_percent < self.low_battery_thresh and not self.low_battery:
                #     rospy.loginfo('CF' + str(self.cf_num) + ' is ending hover command due to low battery.')
                #
                #     # Change to low battery state
                #     self.low_battery = True
                #
                #     # switchLand to finish hover command
                #     residual_time = hover_time - (time.time() - start)
                #     self.switchLand(self.event_array, 'hover', [residual_time])
                #     break

            break

    # Go to Point
    # Input: x = goal x coord (meters), y = goal y coord (meters), z = goal z coord (meters)
    # yaw = goal yaw (degrees) - NOT working yet, num = CF num to command, tol = goal position error (meters),
    # sync = wait on all CFs (default False)
    def goTo(self, x, y, z, yaw, num, tol=0.035, sync=False):

        # Skip if in emergency land state
        if self.emergency_land:
            return

        # Current thread is CF specified by num in goTo arguments
        elif self.cf_num == num:

            while not rospy.is_shutdown():
                counter = 0
                self.msg.yaw = self.ext_yaw

                self.elap_time = 0.0
                self.start = time.time()

                # Keep commanded position within bounds
                if x > 0:
                    x = min(x, self.xbounds[1])
                else:
                    x = max(x, self.xbounds[0])
                if y > 0:
                    y = min(y, self.ybounds[1])
                else:
                    y = max(y, self.ybounds[0])
                if z < self.zbounds[0]:
                    z = self.zbounds[0]
                elif z > self.zbounds[1]:
                    z = self.zbounds[1]


                # Initialize direction needed to turn
                a_x = math.cos(math.radians(self.ext_yaw))
                a_y = math.sin(math.radians(self.ext_yaw))
                b_x = math.cos(math.radians(yaw))
                b_y = math.sin(math.radians(yaw))

                nz = (a_x*b_y)-(a_y*b_x)

                if nz > 0:
                    dir = 1 # CCW
                else:
                    dir = -1 # CW

                update_dir = 1 # variable used in case crazyflie passes over yaw

                # Command position until within specified tolerance (form of closed loop control)
                while (not self.emergency_land) and ((abs(x - self.ext_x) > tol) or (abs(y - self.ext_y) > tol)
                                                     or (abs(z - self.ext_z) > (2 * tol)) or (abs(yaw - self.ext_yaw) > 4)) and self.is_tracking \
                        and not self.is_charging:

                    # Yawing stuff
                    if abs(yaw - self.ext_yaw) < self.vy:
                        self.msg.yaw = yaw
                        update_dir = 0

                    else:
                        if update_dir == 0:
                            a_x = math.cos(math.radians(self.ext_yaw))
                            a_y = math.sin(math.radians(self.ext_yaw))
                            b_x = math.cos(math.radians(yaw))
                            b_y = math.sin(math.radians(yaw))

                            nz = (a_x * b_y) - (a_y * b_x)

                            if nz > 0:
                                dir = 1  # CCW
                            else:
                                dir = -1  # CW

                            update_dir = 1

                        self.msg.yaw = self.msg.yaw + dir*(1/self.hz)*self.vy

                    if self.msg.yaw > 180:
                        self.msg.yaw = self.msg.yaw-360

                    elif self.msg.yaw < -180:
                        self.msg.yaw = self.msg.yaw+360

                    ## XY stuff
                    if abs(self.ext_x - x) < self.vh:
                        self.msg.x = x

                    else:
                        if x > self.ext_x:
                            dir_x = 1
                        else:
                            dir_x = -1
                        self.msg.x = dir_x*(1/self.hz) * self.vh + self.msg.x

                    if abs(self.ext_y - y) < self.vh:
                        self.msg.y = y

                    else:
                        if y > self.ext_y:
                            dir_y = 1
                        else:
                            dir_y = -1
                        self.msg.y = dir_y * (1 / self.hz) * self.vh + self.msg.y

                    # Set position to publish
                    #counter += 1/self.hz
                    #self.msg.x = x
                    #self.msg.y = y
                    #self.msg.yaw = yaw
                    self.msg.z = z
                    self.msg.header.seq += 1
                    self.msg.header.stamp = rospy.Time.now()
                    # Log info
                    # rospy.loginfo("Going to Position")
                    # rospy.loginfo(self.msg.x)
                    # rospy.loginfo(self.msg.y)
                    # rospy.loginfo(self.msg.z)
                    # rospy.loginfo(self.msg.yaw)
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                    self.elap_time = time.time() - self.start

                    # Display position
                    # print('X: {}, Y: {}, Z: {}'.format(self.ext_x, self.ext_y, self.ext_z))

                    # Set state to emergency land if taking too long to reach position
                    if self.elap_time > 20:
                        print(f'CF{self.cf_num}: GoTo {x},{y},{z}, Actual {self.ext_x},{self.ext_y},'
                              f'{self.ext_z}')
                        self.emergency_land = True
                        self.land()
                        self.emerg_land('goal')
                        break

                    # Don't use for mission planning
                    # Set state to emergency land if not all CFs tracked
                    # elif not all(self.all_cfs_is_tracking):
                    #     self.emergency_land = True
                    #     self.emerg_land('tracking')

                    # Don't use for mission planning
                    # Return to parent function to perform switchLand
                    # Only check if not change in height (consumes more instantaneous battery)
                    # elif self.battery_percent < self.low_battery_thresh and not self.low_battery \
                    #         and abs(z - self.ext_z) < (3 * tol):
                    #     self.low_battery = True
                    #     rospy.loginfo('CF' + str(self.cf_num) + ' is ending goTo command due to low battery.')
                    #     return

                break

        # If required to wait on all other CFs
        if sync:

            # Current CF has finished command
            Crazyflie.finished_command[self.cf_idx] = True

            # Hover while waiting for other CFs
            while not all(self.finished_command):
                if not self.is_charging:
                    self.hover(1 / self.hz)

            # Barrier to wait for all CFs
            bt_idx = 0
            try:
                bt_idx = self.bt.wait()
                self.bt.reset()
            except BrokenBarrierError:
                pass

            # One CF resets finished_command array values
            if bt_idx == 0:
                Crazyflie.finished_command = [False] * self.num_threads

            # Barrier to wait for CFs again (prevents finished_command from being overwritten)
            try:
                self.bt.wait()
                self.bt.reset()
            except BrokenBarrierError:
                pass

    # Go to sequence of points
    # Input: sequence (3D array) = array of sequences of points for each CF to go to (meters, degrees),
    # nums = array of CF nums to command (match sequence 3D array indices), tol = goal position error (meters),
    # sync = wait on all CFs (default False)
    def goToSequence(self, sequence, nums, tol=0.035, sync=False):

        # Exit command if sequence 2D array does not have enough indices to match nums array
        # or in emergency land state
        if len(nums) != len(sequence) or self.emergency_land:
            return

        # Current thread is CF specified in nums array
        elif self.cf_num in nums:

            self.cf_seq = sequence[nums.index(self.cf_num)]  # Get individual sequence of points

            # Run goTo command for each set of points in the sequence
            for self.seq_idx in range(len(self.cf_seq)):
                self.goTo(self.cf_seq[self.seq_idx][0], self.cf_seq[self.seq_idx][1], self.cf_seq[self.seq_idx][2],
                          self.cf_seq[self.seq_idx][3], self.cf_num, tol, sync)

                # Break if not tracking to get correct seq idx where it failed
                if not self.is_tracking:
                    break

                # Don't use for mission planning
                # Low battery
                # if self.battery_percent < self.low_battery_thresh:
                #
                #     rospy.loginfo('CF' + str(self.cf_num) + ' is ending goToSequence command due to low battery.')
                #     self.low_battery = True
                #
                #     # Leftover mission sequence to complete
                #     residual_sequence = [self.cf_seq[i:len(self.cf_seq)]]
                #
                #     # Go into low battery land which will switchLand if necessary with the nearest charge
                #     self.switchLand(self.event_array, 'goToSequence', [residual_sequence])
                #     return

        # If required to wait on all other CFs
        elif sync:

            for i in range(len(sequence[0])):

                # Current CF has finished command
                Crazyflie.finished_command[self.cf_idx] = True

                # Hover while waiting for other CFs
                while not all(self.finished_command):
                    if not self.is_charging:
                        self.hover(1 / self.hz)

                if not self.emergency_land:

                    # Barrier to wait for all CFs
                    try:
                        self.bt.wait()
                        self.bt.reset()
                    except BrokenBarrierError:
                        pass

                    # Barrier to wait for all CFs
                    try:
                        self.bt.wait()
                        self.bt.reset()
                    except BrokenBarrierError:
                        pass

    # Move Distance
    # Input: delta_x = change in x pos (meters), delta_y = change in y pos (meters),
    # delta_z = change in z pos (meters), delta_yaw = change in z pos (degrees) - NOT working yet,
    # nums = array of CF nums to command (default None = all), sync = wait on all CFs (default False)
    def move(self, delta_x, delta_y, delta_z, delta_yaw, nums=None, sync=False):

        # Skip if in emergency land state
        if self.emergency_land:
            return

        # Command all CFs if nums is None or only CFs specified in nums array
        elif nums is None or self.cf_num in nums:

            # Set goTo position based on current position and commanded change in position
            self.x = self.ext_x + delta_x
            self.y = self.ext_y + delta_y
            self.z = self.ext_z + delta_z
            self.yaw = self.msg.yaw + delta_yaw

            # Call goTo command on new positions
            self.goTo(self.x, self.y, self.z, self.yaw, self.cf_num)

        # If required to wait on all other CFs
        if sync:

            # Current CF has finished command
            Crazyflie.finished_command[self.cf_idx] = True

            # Hover while waiting for other CFs
            while not all(self.finished_command):
                if not self.is_charging:
                    self.hover(1 / self.hz)

            # Barrier to wait for all CFs
            bt_idx = 0
            try:
                bt_idx = self.bt.wait()
                self.bt.reset()
            except BrokenBarrierError:
                pass

            # One CF resets finished_command array values
            if bt_idx == 0:
                Crazyflie.finished_command = [False] * self.num_threads

            # Barrier to wait for CFs again (prevents finished_command from being overwritten)
            try:
                self.bt.wait()
                self.bt.reset()
            except BrokenBarrierError:
                pass

    # Move Sequence of Distances
    # Input: sequence (3D array): array of sequences of position/rotation changes for each CF (meters, degrees),
    # nums = array of CF nums to command (match sequence 3D array indices), sync = wait on all CFs (default False)
    def moveSequence(self, sequence, nums, sync=False):

        # Exit command if sequence 2D array does not have enough indices to match nums array
        # or in emergency land state
        if len(nums) != len(sequence) or self.emergency_land:
            return

        # Run sequence if CF thread number in nums array
        elif self.cf_num in nums:

            self.cf_seq = sequence[nums.index(self.cf_num)]  # Get individual sequence of points

            # Run move command for each set of points in the sequence
            for i in range(len(self.cf_seq)):
                self.move(self.cf_seq[i][0], self.cf_seq[i][1], self.cf_seq[i][2], self.cf_seq[i][3])

                # Don't use for mission planning
                # Low battery
                # if self.battery_percent < self.low_battery_thresh:
                #
                #     rospy.loginfo('CF' + str(self.cf_num) + ' is ending moveSequence command due to low battery.')
                #     self.low_battery = True
                #
                #     # Leftover mission sequence to complete
                #     residual_sequence = self.cf_seq[i:len(self.cf_seq)]
                #
                #     # Go into low battery land which will switchland if necessary with the nearest charge
                #     self.switchLand(self.event_array, 'moveSequence', residual_sequence)
                #     return

        # If required to wait on all other CFs
        elif sync:

            for i in range(len(sequence[0])):

                # Current CF has finished command
                Crazyflie.finished_command[self.cf_idx] = True

                # Hover while waiting for other CFs
                while not all(self.finished_command):
                    if not self.is_charging:
                        self.hover(1 / self.hz)

                # Barrier to wait for all CFs
                try:
                    self.bt.wait()
                    self.bt.reset()
                except BrokenBarrierError:
                    pass

                # Barrier to wait for all CFs
                try:
                    self.bt.wait()
                    self.bt.reset()
                except BrokenBarrierError:
                    pass

    # Switch flying Crazyflie with one that is wirelessly charging
    # Input: events = array of initialized events equal to length of charging pads (thread synchronization technique)
    # add_func = string with function name where new CF will pick up,
    # add_args = array of ALL arguments required for the new CF to pickup the actions of the old CF
    def switchLand(self, events, add_func=None, add_args=None):

        # Skip if in emergency land state or not tracked
        if self.emergency_land:  # and not all(self.all_cfs_is_tracking): - Don't use for mission planning
            return

        else:

            # Run if flying CF
            if not self.is_charging:

                # At least 1 charger
                if self.num_chargers:

                    # Find closest feasible charger to land on {'Pad#': dist}
                    charger_dist = {}
                    for k in range(len(self.all_charger_pos)):
                        charger_dist[self.pad_names[k]] = Crazyflie.dist_2D([self.ext_x, self.ext_y],
                                                                            self.all_charger_pos[k])

                    # Sort dictionary by values
                    charger_dist = {k: v for k, v in sorted(charger_dist.items(), key=lambda item: item[1])}

                    # Find first charging pad that is either unoccupied or feasible to switch with
                    for key in charger_dist.keys():

                        # Charging pad not occupied
                        if self.chargers_occupied[key] == 0:

                            self.land(key[-1])  # Go to charger XY position and land
                            Crazyflie.finished_command[self.cf_idx] = True

                            # Wait for CF to charge about halfway (~15%)
                            while self.battery_percent <= 75:
                                time.sleep(1)

                            # After charge time, set new value in chargers occupied
                            Crazyflie.chargers_occupied[key] = self.cf_num  # Assign CF num to pad it's charging on
                            self.low_battery = False  # No longer in low battery state
                            return

                        # CF currently landing or has not been charging long enough
                        elif self.chargers_occupied[key] < 0:
                            continue

                        # Charging pad occupied - requires switch
                        else:

                            # Put add_func and add_args in queue
                            self.q.put([add_func, add_args])

                            # Find switch position for flying CF
                            charger_pos = self.all_charger_pos[self.pad_names.index(key)]

                            self.desired_switch_pos = self.findSwitchPos(0.3, key)

                            # Check next charging pad if infeasible to switch
                            if not any(self.desired_switch_pos):
                                continue

                            # Perform switch with charging CF
                            else:

                                # Go to switch position and decrease altitude to 0.3 meters
                                self.goTo(charger_pos[0] + self.desired_switch_pos[0],
                                          charger_pos[1] + self.desired_switch_pos[1], self.ext_z, 0, self.cf_num)
                                self.goTo(charger_pos[0] + self.desired_switch_pos[0],
                                          charger_pos[1] + self.desired_switch_pos[1], 0.3, 0, self.cf_num)

                                # Notify charging CF to takeoff
                                events[self.pad_names.index(key)].set()

                                # Hover while waiting for charging CF to finish takeoff/move sequence
                                while events[self.pad_names.index(key)].is_set():
                                    self.hover(1 / self.hz)

                                self.land(key[-1])  # Go to charger XY position and land
                                self.is_charging = True
                                Crazyflie.finished_command[self.cf_idx] = True

                                # Wait for CF to charge about halfway (~15%)
                                while self.battery_percent <= 75:
                                    time.sleep(1)

                                # After charge time, set new value in chargers occupied
                                Crazyflie.chargers_occupied[key] = self.cf_num  # Assign CF num to pad it's charging on
                                self.low_battery = False  # No longer in low battery state

                                self.switchLand(self.event_array)  # Get ready for another switch

                                return

                # Did not find feasible charging pad to land at or there are no charging pads
                if not self.is_charging:

                    self.land(0, readjust=False)  # Land on ground directly below

                    # Set parameters to True so that it has no chance of taking off again
                    self.is_charging = True
                    self.emergency_land = True

                    Crazyflie.finished_command[self.cf_idx] = True

            # Run if CF is charging
            else:

                # Find CF's charging pad
                pad_name = None
                for pad, charging_num in self.chargers_occupied.items():
                    if self.cf_num == charging_num:
                        pad_name = pad
                        break

                # Wait for corresponding flying CF to reach switch position
                events[self.pad_names.index(pad_name)].wait()

                # Find desired switch position for charging CF
                self.desired_switch_pos = self.findSwitchPos(0.3, pad_name)

                # Do nothing if all switch positions are infeasible
                if self.desired_switch_pos == [0, 0]:
                    Crazyflie.finished_command[self.cf_idx] = True
                    return

                # Set CF to not charging, takeoff, and move to switch position
                self.is_charging = False
                self.takeoff(0.6)
                self.move(self.desired_switch_pos[0], self.desired_switch_pos[1], 0.0, 0.0, [self.cf_num])
                Crazyflie.chargers_occupied[pad_name] = 0  # Reset chargers_occupied value

                # Notify flying CF that charging CF has reached switch position
                events[self.pad_names.index(pad_name)].clear()

                Crazyflie.finished_command[self.cf_idx] = True

                # No add_func passed in switchLand call
                if not self.q.empty():

                    # Get add_func and add_args from queue
                    [func_name, args] = self.q.get()

                    # Extra parameter if sequence vs just hover
                    if func_name in ['goToSequence', 'moveSequence']:
                        args.append([self.cf_num])

                    # Get function call ready
                    rospy.loginfo('CF' + str(self.cf_num) + ' is continuing previous ' + func_name + ' command.')
                    func_call_str = 'self.' + func_name

                    # Wait briefly for other CF to land
                    self.hover(8)

                    # Continue mission of CF that ran out of battery
                    eval(func_call_str)(*args)

                else:
                    self.land()

    # Determine which Crazyflies do not takeoff at initial time (needs improvement - height different depending on x,y)
    # Input: none
    def setIsCharging(self):

        for p in range(len(self.all_charger_pos)):
            pos = self.all_charger_pos[p]
            # if (pos[0] - 0.1 <= self.ext_x <= pos[0] + 0.1) and (pos[1] - 0.1 <= self.ext_y <= pos[1] + 0.1):
            if Crazyflie.dist_2D(pos, [self.ext_x, self.ext_y]) <= 0.1:
                self.is_charging = True
                Crazyflie.chargers_occupied[self.pad_names[p]] = self.cf_num
                return

    # Determine switch position (north, east, south, west) for charging/flying Crazyflies
    # Input: switch_dist = distance from charger to move during switchLand (meters),
    # pos = charger position to find switch positions for (meters)
    # Output: switch = ideal switch position to move CF to (meters)
    def findSwitchPos(self, switch_dist, pad_name) -> list:

        charger_pos = self.all_charger_pos[self.pad_names.index(pad_name)]
        homeX = charger_pos[0]  # Charger x position
        homeY = charger_pos[1]  # Charger y position

        # Possible switch distances for charger
        d = [[switch_dist, 0], [0, switch_dist], [-switch_dist, 0], [0, -switch_dist]]

        # Initialize array to track total distance from charging CF to all other CFs
        switch_pos = [0, 0, 0, 0]

        # Iterate over each switch position (north, east, south, west)
        for i in range(4):

            # Is switch position outside bounds?
            if (homeX + d[i][0]) > self.xbounds[1] or (homeX + d[i][0]) < self.xbounds[0] or \
                    (homeY + d[i][1]) > self.ybounds[1] or (homeY + d[i][1]) < self.ybounds[0]:

                switch_pos[i] = 0  # Eliminates option since selecting max from switch_pos array

            # Switch position within bounds
            else:

                # Iterate over all CFs
                for j in range(len(self.cf_names)):

                    cf_pos = self.cfs_curr_pos[int(self.cf_names[j][2:]) - 1]  # Crazyflie position

                    # Don't count self or charging CF
                    if int(self.cf_names[j][2:]) != self.cf_num \
                            and int(self.cf_names[j][2:]) not in self.chargers_occupied.values() \
                            or int(self.cf_names[j][2:]) == self.chargers_occupied[pad_name]:

                        # Is another CF within 0.2 meters of x or y position of this CF?
                        if ((homeX + d[i][0] - 0.2) <= cf_pos[0] <= (homeX + d[i][0] + 0.2)) \
                                and ((homeY + d[i][1] - 0.2) <= cf_pos[1] <= (homeY + d[i][1] + 0.2)):

                            switch_pos[i] = 0  # Eliminates option since selecting max from switch_pos array

                            break  # Move to next switch position

                        # Sum 2D distances from proposed switch position to each Crazyflie
                        switch_pos[i] += Crazyflie.dist_2D([homeX + d[i][0], homeY + d[i][1]], cf_pos[0:2])

        # All switch positions set to 0? (infeasible)
        if not any(switch_pos):

            rospy.logwarn('Cannot find feasible switch position')
            switch = [0, 0]

        # One switch position not 0
        else:

            switch = d[switch_pos.index(max(switch_pos))]  # Select switch position with max distance from all CFs

        return switch

    # Land Function
    # Input: pad_num = charging pad number to land at (0 = ground landing),
    # readjust = boolean to takeoff and land up to 3 times if not charging on charging pad (default True)
    def land(self, pad_num=0, readjust=True):

        # Land on ground below if emergency
        if self.emergency_land:
            pad_num = 0

        self.start_land_time = time.time()

        # Land directly below on ground if no pad number specified or charger occupied by CF other than itself
        # or emergency land
        if pad_num == 0 or ('Pad' + str(pad_num)) not in self.pad_names \
                or self.chargers_occupied['Pad' + str(pad_num)] not in [0, -self.cf_num]:

            if pad_num == 0:
                rospy.loginfo(f'No charging pad specified. CF{self.cf_num} landing on ground directly below.')
            elif ('Pad' + str(pad_num)) not in self.pad_names:
                rospy.loginfo(f'That charging pad number was not found. '
                              f'CF{self.cf_num} landing on ground directly below.')
            elif self.chargers_occupied['Pad' + str(pad_num)] not in [0, -self.cf_num]:
                rospy.logwarn(f'Charging pad {pad_num} is already occupied. '
                              f'CF{self.cf_num} landing on ground directly below.')

            self.x = self.ext_x
            self.y = self.ext_y
            self.z = self.ext_z
            self.yaw = self.msg.yaw
            land_height = 0.1
            readjust = False

        # Set land position to above specified charging pad position
        else:

            Crazyflie.chargers_occupied['Pad' + str(pad_num)] = -self.cf_num  # Charging pad not ready for switch

            self.z = self.all_charger_pos[self.pad_names.index('Pad' + str(pad_num))][2]
            self.yaw = self.msg.yaw

            if self.readjust_count == 0:
                self.z += 0.3

            # Climb to altitude 0.3 meters above charger
            self.move(0, 0, self.z - self.ext_z, 0, [self.cf_num])

            # Set goal to charger's XY position
            self.x = self.all_charger_pos[self.pad_names.index('Pad' + str(pad_num))][0]
            self.y = self.all_charger_pos[self.pad_names.index('Pad' + str(pad_num))][1]
            self.goTo(self.x, self.y, self.z, self.yaw, self.cf_num, tol=0.025)
            self.hover(2)  # Ensure stability

            # Charger is moving (magnitude of velocity greater than 0.1
            # vel_mag = math.sqrt(self.all_charger_vel[pad_names.index('Pad' + str(pad_num))][0]**2
            #                     + self.all_charger_vel[pad_names.index('Pad' + str(pad_num))][1]**2)
            # if vel_mag >= 0.1:
            #     self.goTo(self.x, self.y, self.z, self.yaw, self.cf_num, tol=1)
            # else:
            #     self.goTo(self.x, self.y, self.z, self.yaw, self.cf_num, tol=0.025)
            #     self.hover(2)  # Ensure stability

            # Altitude (meters) to stop motors (based on charger z position)
            land_height = self.z - 0.2

        while not rospy.is_shutdown():

            # Repeat until altitude is less than land height or loses tracking
            while self.ext_z > land_height and self.is_tracking and not self.is_charging:

                # Go to charger's XY position
                # self.x = self.all_charger_pos[pad_names.index('Pad' + str(pad_num))][0] \
                #     + (1/self.hz)*self.all_charger_vel[pad_names.index('Pad' + str(pad_num))][0]
                # self.y = self.all_charger_pos[pad_names.index('Pad' + str(pad_num))][1] \
                #     + (1/self.hz)*self.all_charger_vel[pad_names.index('Pad' + str(pad_num))][1]
                self.msg.x = self.x
                self.msg.y = self.y
                self.msg.z = self.z/self.land_rate  # Decrease commanded z position by land rate
                self.msg.yaw = self.yaw
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                # rospy.loginfo("Landing")
                # rospy.loginfo(self.msg.x)
                # rospy.loginfo(self.msg.y)
                # rospy.loginfo(self.ext_z)
                # rospy.loginfo(self.msg.yaw)
                self.pub.publish(self.msg)
                self.rate.sleep()
                self.land_rate *= 1.05  # Increase land rate for each iteration

                # Update land height for each iteration
                if pad_num != 0 and ('Pad' + str(pad_num)) in self.pad_names:
                    land_height = self.all_charger_pos[self.pad_names.index('Pad' + str(pad_num))][2] + 0.1

            self.msg.x = self.x
            self.msg.y = self.y
            self.msg.z = 0  # Signifies landing
            self.msg.yaw = self.yaw
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            self.stop_pub.publish(self.stop_msg)  # Stop motors

            # Takeoff & Landing Readjustment (if not charging) - max 3 times
            if readjust and self.readjust_count < 2 and self.is_tracking:

                self.readjust_count += 1
                time.sleep(3)  # Wait to check if charging

                # Accuracy of 0.015 wasn't working with rail system on 5/5/20
                if Crazyflie.dist_2D([self.x, self.y], [self.ext_x, self.ext_y]) > 0.03:
                    rospy.loginfo('CF{} readjusting'.format(self.cf_num))

                    # Takeoff to low altitude for readjustment
                    self.takeoff(0.2 + self.all_charger_pos[self.pad_names.index('Pad' + str(pad_num))][2])

                    self.land(pad_num)  # Land again

            self.is_charging = True

            self.readjust_count = 0
            print('CF' + str(self.cf_num) + ' land time: ' + str(time.time() - self.start_land_time) + ' seconds')
            break

    # Emergency Land Function
    # Emergency: tracking lost or did not achieve goal position
    def emerg_land(self, fault):

        # Reset number of threads to synchronize
        self.bt.reset()  # Results in BrokenBarrierError exception for all waiting threads
        Crazyflie.num_threads -= 1
        self.bt = Barrier(self.num_threads)

        if fault == 'goal':
            rospy.logwarn(self.prefix + ' Emergency Landing - Did Not Achieve Goal Position')
            self.is_tracking = False
            self.all_cfs_is_tracking[self.cf_num - 1] = False
        else:
            rospy.logwarn(self.prefix + ' Emergency Landing - Tracking')

        # Land on ground
        self.land()

    # Crazyflie Position Subscriber Callback
    # Input: data = data returned from external position subscriber (pos_subscriber)
    def callback(self, data):
        # Crazyflies always tracked correctly if this callback function is invoked

        # else:
        # self.tracking_frames_lost = 0  # Reset so only reaches threshold for consecutive lost frames

        # Position

        #self.ext_x = data.point.x
        #self.ext_y = data.point.y
        #self.ext_z = data.point.z


        #Quaternians

        # Reset current external position variables
        self.ext_x = data.pose.position.x
        self.ext_y = data.pose.position.y
        self.ext_z = data.pose.position.z

        r = R.from_quat([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        rot = r.as_euler('xyz', degrees=True)
        self.ext_yaw = rot[2]
        # self.ext_x = data.pose.position.x
        # self.ext_y = data.pose.position.y
        # self.ext_z = data.pose.position.z

        # CF outside x bounds
        if self.ext_x < (self.xbounds[0]) or self.ext_x > (self.xbounds[1]):
            rospy.logwarn('CF' + str(self.cf_num) + ' is outside x bounds')
            self.stop_pub.publish(self.stop_msg)  # Stop motors
            self.pos_subscriber.unregister()  # unable to receive future position commands
            self.batt_subscriber.unregister()  # No need to get battery data anymore

        # CF outside y bounds
        if self.ext_y < (self.ybounds[0]) or self.ext_y > (self.ybounds[1]):
            rospy.logwarn('CF' + str(self.cf_num) + ' is outside y bounds')
            self.stop_pub.publish(self.stop_msg)  # Stop motors
            self.pos_subscriber.unregister()  # unable to receive future position commands
            self.batt_subscriber.unregister()  # No need to get battery data anymore

        # CF outside z bounds
        if self.ext_z < (self.zbounds[0]) or self.ext_z > (self.zbounds[1]):
            rospy.logwarn('CF' + str(self.cf_num) + ' is outside z bounds')
            self.stop_pub.publish(self.stop_msg)  # Stop motors
            self.pos_subscriber.unregister()  # unable to receive future position commands
            self.batt_subscriber.unregister()  # No need to get battery data anymore

        # Convert quaternion angles back to euler to command yaw (z-axis rotation)
        # quat_orientation = R.from_quat([data.pose.orientation.x, data.pose.orientation.y,
        #                                data.pose.orientation.z, data.pose.orientation.w])
        # eul_orientation = quat_orientation.as_euler('xyz', degrees=True)
        # self.ext_yaw = eul_orientation[2]

        # Update array of all Crazyflies' external positions
        Crazyflie.cfs_curr_pos[self.cf_num - 1] = [self.ext_x, self.ext_y, self.ext_z]


    # track unknown object with only 1 drone
    def track_object_stationary_1(self,target_x,target_y,target_z):

        track_x = 2
        track_y = 2
        track_z = 0.5

        # crazyflie that tracked object

        # finding x,y,and z position to go to
        opt_dist = 2  # optimal distance for continuous tracking
        # find vector from tracked object to tracking drone
        x_dist = self.ext_x - track_x
        y_dist = self.ext_y - track_y
        mag = math.sqrt(x_dist ** 2 + y_dist ** 2)
        # distance from drone for optimal tracking
        x_d_opt = (x_dist / mag) * opt_dist
        y_d_opt = (y_dist / mag) * opt_dist

        # find target position
        tg_x = track_x + x_d_opt  #########################3
        tg_y = track_y + y_d_opt ########################
        tg_z = track_z ###############################

        # finding yaw to maintain
        x_dir = -x_dist
        y_dir = -y_dist

        # find quadrant
        if x_dir >= 0:
            yaw_mod = 0
        else:
            if y_dir >= 0:
                yaw_mod = 180
            else:
                yaw_mod = -180

        tg_yaw = math.degrees(math.atan(y_dir / x_dir)) + yaw_mode  # yaw to move towards target with   #############################

    #track unknown object with 2 drones, one with net launcher, one with camera (first position in nums is tracker, 2nd is net)
    def track_object_stationary_camera(self,num, tol=0.035):

            # if tracking has been made, get global x, y and z, and drone that tracked it (potentially x y and z of drone the tracked it)
            # could also pass relative and get rid of some of this code for tracker drone
            track_x = 0
            track_y = 0
            track_z = 0.5

            # finding x,y,and z position to go to
            opt_dist_t = 0.5  # optimal distance for continuous tracking

            # find vector from tracked object to tracking drone

            # This is used later with other code
            self.t_x = self.ext_x
            self.t_y = self.ext_y

            x_dist = self.ext_x - track_x
            y_dist = self.ext_y - track_y
            mag = math.sqrt(x_dist ** 2 + y_dist ** 2)

            # distance from drone for optimal tracking
            x_d_opt = (x_dist / mag) * opt_dist_t
            y_d_opt = (y_dist / mag) * opt_dist_t

            # find target position for tracker drone
            tg_x_t = track_x + x_d_opt #########################
            tg_y_t = track_y + y_d_opt #########################
            tg_z_t = track_z ##########################

            # finding yaw to maintain
            self.x_dir_tt = -x_dist
            self.y_dir_tt = -y_dist

            # find quadrant
            if self.x_dir_tt >= 0:
                yaw_mod = 0
            else:
                if self.y_dir_tt >= 0:
                    yaw_mod = 180
                else:
                    yaw_mod = -180

            self.tg_yaw_t = math.degrees(math.atan(self.y_dir_tt / self.x_dir_tt)) + yaw_mod  # yaw to move towards target with tracker drone #######################
            self.tracker_flag = 1

            self.goTo(tg_x_t, tg_y_t, tg_z_t,self.tg_yaw_t, num, tol, sync=False)

            # publish this info
            #try:
            #    self.camera_pub_callback()
            #except rospy.ROSInterruptException:
            #    print('publish error')
            #    self.hover(3)
            #    self.land()
                #pass
            #return [self.ext_x,self.ext_y,self.t_x,self.t_y,self.x_dir_tt,self.y_dir_tt]

            self.camera_pub_callback()

            self.hover(4)
            self.land()

    def camera_pub_callback(self):
        pub_adv = rospy.Publisher("camera_data",camera_msg, queue_size = 1)
        location_info = camera_msg()
        rate = self.rate
        #while not rospy.is_shutdown():
        location_info.ext_x = int(self.ext_x)
        location_info.ext_y = int(self.ext_y)
        location_info.t_x = int(self.t_x)
        location_info.t_y = int(self.t_y)
        location_info.x_dir_tt = int(self.x_dir_tt)
        location_info.y_dir_tt = int(self.y_dir_tt)

        pub_adv.publish(location_info)
        rate.sleep()

    def camera_sub_callback(self,data):
        self.ext_x = data.ext_x
        self.ext_y = data.ext_y
        self.t_x = data.t_x
        self.t_y = data.t_y
        self.x_dir_tt = data.x_dir_tt
        self.y_dir_tt = data.y_dir_tt

        #self.ext_x = track_data[0]
        #self.ext_y = track_data[1]
        #self.t_x = track_data[2]
        #self.t_y = track_data[3]
        #self.x_dir_tt = track_data[4]
        #self.y_dir_tt = track_data[5]



    def track_object_stationary_net(self,num, tol=0.035):

        # subscribe to publisher from crazyflie camera
        rospy.Subscriber("camera_data",camera_msg,self.camera_sub_callback)

        track_x = 0
        track_y = 0
        track_z = 0.5


        self.takeoff(0.5)
        self.hover(1)

        #self.ext_x = track_data[0]
        #self.ext_y = track_data[1]
        #self.t_x = track_data[2]
        #self.t_y = track_data[3]
        #self.x_dir_tt = track_data[4]
        #self.y_dir_tt = track_data[5]

        # get vectors to target and net from tracker (to decide orientation of net launcher target)
        # already have from tracker to target - x_dir_tt and y_dir_tt, just need tracker to net
        x_dir_tn = self.ext_x - self.t_x
        y_dir_tn = self.ext_y - self.t_y

        opt_dist_n_h = 0.75  # optimal horizontal distance for net launcher
        opt_dist_n_v = 0.2  # optimal vertical distance for net launcher
        net_offset = 45  # offset (degrees) of direction of net launcher from crazyflie defined forward direction

        #take cross product to understand orientation
        nz_2 = (self.x_dir_tt * y_dir_tn) - (self.y_dir_tt * x_dir_tn)

        # orient net launcher 90 degrees off of tracker drone, the side depends on initial orientation
        if nz_2 > 0:
            angle_mod = -90
        else:
            angle_mod = 90

        n_pos_angle = -(180 - self.tg_yaw_t) + angle_mod

        n_xmod = opt_dist_n_h * math.cos(math.radians(n_pos_angle))
        n_ymod = opt_dist_n_h * math.sin(math.radians(n_pos_angle))

        # find target position for net drone
        tg_x_n = track_x + n_xmod   #################
        tg_y_n = track_y + n_ymod   ##################
        tg_z_n = track_z + opt_dist_n_v   #############


        # finding yaw for net
        # find quadrant
        if -n_xmod >= 0:
            yaw_mod2 = 0
        else:
            if -n_ymod >= 0:
                yaw_mod2 = 180
            else:
                yaw_mod2 = -180

        tg_yaw_n = math.degrees(math.atan(-n_ymod / -n_xmod)) + yaw_mod2 + net_offset #################

        self.goTo(tg_x_n, tg_y_n, tg_z_n, tg_yaw_n, num, tol, sync=False)

        self.hover(9)
        self.land()


    # Crazyflie Battery Subscriber Callback
    # Input: msg = messages received by battery subscriber (batt_subscriber)
    def battery_callback(self, msg):

        vbat = msg.data  # Battery voltage
        min_vbat = 3.0  # Battery voltage lower limit
        max_vbat = 4.23 # Battery voltage upper limit
        #min_vbat = 8  # Battery voltage lower limit
        #max_vbat = 10.8 # Battery voltage upper limit
        self.battery_percent = ((vbat - min_vbat) / (max_vbat - min_vbat)) * 100

        self.batt_percent_arr.append(self.battery_percent)
        self.time_arr.append(time.time() - self.batt_start)

        # rospy.loginfo("Battery Percentage: %.2f%%" % self.battery_percent)

        # Check if battery below threshold outside of takeoff
        if not self.in_takeoff and self.battery_percent < self.low_battery_thresh:
            rospy.logwarn("LOW BATTERY!")

    # Crazyflie Lost Tracking Subscriber Callback - Sent from Qualisys since cannot publish NaNs to Crazyflies
    # and therefore cannot receive that information
    def lost_frames_callback(self, nan_msg):
        # # NaN value in coordinate data from Qualisys shows not tracking

        # if any([math.isnan(coord) for coord in [data.point.x, data.point.y, data.point.z]]):
        #     self.tracking_frames_lost += 1
        #     rospy.loginfo(f'CF{self.cf_num} tracking frames lost: {self.tracking_frames_lost}')
        #
        #     # if self.tracking_frames_lost > self.tracking_frames_thresh:
        #     #     self.hover(1)

        # if self.tracking_frames_lost > self.tracking_frames_thresh2:

        # Too many frames not tracked?
        if self.is_tracking:
            rospy.logwarn('Lost tracking for ' + self.prefix)  # Output only first time
            self.is_tracking = False
            Crazyflie.all_cfs_is_tracking[self.cf_num - 1] = False
        self.stop_pub.publish(self.stop_msg)  # Stop motors

        # Set position to land straight down
        # self.msg.x = self.ext_x
        # self.msg.y = self.ext_y
        # self.msg.yaw = 0
        # self.msg.z = 0
        # self.msg.header.seq += 1
        # self.msg.header.stamp = rospy.Time.now()
        # self.pub.publish(self.msg)

        self.pos_subscriber.unregister()  # unable to receive future position commands
        self.batt_subscriber.unregister()  # No need to get battery data anymore

    # Return the distance between 2 positions in 2D
    # Input: pos1 = first position array of coordinates, pos2 = second position array of coordinates
    # Output: 2D distance between points (XY plane) (units^2)
    @staticmethod
    def dist_2D(pos1, pos2):

        [x1, y1] = pos1[0:2]
        [x2, y2] = pos2[0:2]
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
