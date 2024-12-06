#!/usr/bin/env python3


import sys
import rospy

from new_driver import Driver

from math import atan2, sqrt


class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self, threshold=0.3):
		super().__init__('odom')
		# Set the threshold to a reasonable number
		self._threshold = threshold

	def close_enough_to_waypoint(self, distance, target, lidar):
		'''
		This function is called perioidically if there is a waypoint set.  This is where you should put any code that
		has a smarter stopping criteria then just checking the distance. See get_twist for the parameters; distance
		is the current distance to the target.
		'''
		# Default behavior.
		if distance < self._threshold:
			return True
		return False

	def get_twist(self, target, lidar):
		'''
		This function is called whenever there a current target is set and there is a lidar data
		available.  This is where you should put your code for moving the robot.  The target point
		is in the robot's coordinate frame.  The x-axis is positive-forwards, and the y-axis is
		positive to the left.

		The example sets constant velocities, which is clearly the wrong thing to do.  Replace this
		code with something that moves the robot more intelligently.

		Parameters:
			target:		The current target point, in the coordinate frame of the robot (base_link) as
						an (x, y) tuple.
			lidar:		A LaserScan containing the new lidar data.

		Returns:
			A Twist message, containing the commanded robot velocities.
		'''
		angle = atan2(target[1], target[0])
		distance = sqrt(target[0] ** 2 + target[1] **2)
		rospy.loginfo(f'Distance: {distance:.2f}, angle: {angle:.2f}')

		# This builds a Twist message with all elements set to zero.
		command = Driver.zero_twist()

		# Forwards velocity goes here, in meters per second.
		command.linear.x = 0.2

		# Rotational velocity goes here, in radians per second.  Positive is counter-clockwise.
		command.angular.z = 0.3

		return command
# def get_twist(self, target, lidar):

#        command = Driver.zero_twist()

#        # TODO:
#        #  Step 1) Calculate the angle the robot has to turn to in order to point at the target
#        #  Step 2) Set your speed based on how far away you are from the target, as before
#        #  Step 3) Add code that veers left (or right) to avoid an obstacle in front of it

#        # Set where we are going and the angle of turns
#        target_x, target_y = target
#        anglet = atan2(target_y, target_x)

#        # use the distance function to get target distance
#        distancet = sqrt(target_x ** 2 + target_y ** 2)

#        # Clamp linear speed between 0.5 and the distance
#        command.linear.x = min(0.5, distancet)
#        command.angular.z = 2.0 * anglet

#        if lidar:
#            # front_angles = range(len(lidar.ranges) // 3, 2 * len(lidar.ranges) // 3)

#            total_readings = len(lidar.ranges)

#            # Focus on only the front 1/3 region of the robots view
#            start_index = total_readings // 3 
#            end_index = 2 * total_readings // 3
#            front_angles = range(start_index, end_index)

#            min_distance = min([lidar.ranges[i] for i in front_angles if lidar.ranges[i] > 0.1], default=float('inf'))

#            #min_distance = float('inf')

#            # Go through the front angles to find the minimum distance
#            for i in front_angles:
#                distance = lidar.ranges[i]
              
#                if distance > 0.1:
#                    min_distance = min(min_distance, distance)

#            if min_distance < 1:  # dist from wa ll, we need custom logic when were close
#                # added global so I can stop alternation
#                if self.state == None:
#                    command.linear.x = 0.0  # stop straigth

#                    mid_index = len(lidar.ranges) // 2
#                    left_ranges = [r for r in lidar.ranges[:mid_index] if r > 0.1]
#                    right_ranges = [r for r in lidar.ranges[mid_index:] if r > 0.1]

#                    min_left_distance = min(left_ranges, default=float('inf'))
#                    min_right_distance = min(right_ranges, default=float('inf'))


#                    if min_left_distance > min_right_distance:
#                        print("goright")
#                        command.angular.z = 0.5 #go r
#                        self.state = True
#                    else:
#                        print("goleft")
#                        command.angular.z = -0.5  # go l
#                        self.state = False
#                else:
#                    # we need to prevent jittering
#                    if self.state:
#                        command.angular.z = 1
#                    else:
#                        command.angular.z = -1 # bigger rotation so we get away from wall

#        return command

if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()
