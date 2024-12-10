#!/usr/bin/env python3


import sys
import rospy

from new_driver import Driver

from math import pi, cos, sin, atan2, sqrt


class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self, threshold=0.5):
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

		K = 5 #gain for turing from objects
		Turning_Threshhold = 1 # Range to start turning from walls m
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
		distance_target = sqrt(target[0] ** 2 + target[1] **2)
		#rospy.loginfo(f'Distance: {distance_target:.2f}, angle: {angle:.2f}')

		# This builds a Twist message with all elements set to zero.
		command = Driver.zero_twist()

		# Forwards velocity goes here, in meters per second.
		max_linear_speed = 0.4
		command.linear.x = min(max_linear_speed, distance_target * 0.5)
		
		command.linear.x *= (pi - abs(angle))/pi # slows the robot down if not pointing at target

		# Rotational velocity goes here, in radians per second.  Positive is counter-clockwise.
		max_angular_speed = pi / 5
		command.angular.z = max(-max_angular_speed, min(max_angular_speed, angle * 2))

		if lidar:

			min_forward_distance = min(lidar.ranges[i] for i in range(0,180))
			T_Thresh = min(Turning_Threshhold, distance_target) # threshhold for turning between max 2m and distance to taget

			if min_forward_distance < 0.2: #checking for forward collision
				rospy.logerr_throttle(5,f'Colided With wall. Reversing...')
				command.linear.x =  -.4 # reverse to give some space
				

			if min_forward_distance < T_Thresh: 

				command.linear.x = min(max_linear_speed, distance_target * 0.5) # maintain speed during turns

				left_A = sum(lidar.ranges[i] for i in range(0,90,1))
				right_A = sum(lidar.ranges[i] for i in range(90,180,1))
				L_ratio = left_A / (left_A + right_A)
				R_ratio = right_A / (left_A + right_A)

				turning_factor = (R_ratio - L_ratio) # Calculate turning direction from areas, postivie is turn left
				
				command.angular.z += (turning_factor) * K  # add to original turning rate for avoiding corner
				#rospy.loginfo(f'Twist_Commands: {(turning_factor)}')

		return command

if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()
