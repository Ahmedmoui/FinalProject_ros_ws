#!/usr/bin/env python3


import sys
import rospy
import signal

from controller import RobotController

from exploring import find_all_possible_goals, find_best_point, plot_with_explore_points, find_waypoints

class StudentController(RobotController):
	'''
	This class allows you to set waypoints that the robot will follow.  These robots should be in the map
	coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
	StudentDriver.
	'''
	def __init__(self):
		super().__init__()

	def distance_update(self, distance):
		'''
		This function is called every time the robot moves towards a goal.  If you want to make sure that
		the robot is making progress towards it's goal, then you might want to check that the distance to
		the goal is generally going down.  If you want to change where the robot is heading to, you can
		make a call to set_waypoints here.  This call will override the current set of waypoints, and the
		robot will start to drive towards the first waypoint in the new list.

		Parameters:
			distance:	The distance to the current goal.
		'''
		#rospy.loginfo(f'Distance: {distance}')

	def map_update(self, point, map, map_data):
		'''
		This function is called every time a new map update is available from the SLAM system.  If you want
		to change where the robot is driving, you can do it in this function.  If you generate a path for
		the robot to follow, you can pass it to the driver code using set_waypoints().  Again, this will
		override any current set of waypoints that you might have previously sent.

		Parameters:
			point:		A PointStamped containing the position of the robot, in the map coordinate frame.
			map:		An OccupancyGrid containing the current version of the map.
			map_data:	A MapMetaData containing the current map meta data.
		'''
		rospy.loginfo('Got a map update.')

		try:
			robot_position = (point.point.x, point.point.y)
			rospy.loginfo(f'Robot is at {robot_position} {point.header.frame_id}')
		except:
			rospy.loginfo('No odometry information')

		# Detect frontiers in the map
		frontiers = self.find_frontiers(map_data)

		# If we find frontiers, generate a goal and set the waypoint
		if frontiers:
			goal = self.generate_frontier_goal(frontiers)
			rospy.loginfo(f'Setting new goal at {goal}')
			self.set_waypoints([goal])  # Move to the new frontier
		else:
			rospy.loginfo('No frontiers found to explore.')

	def find_frontiers(self, map_data):
		frontiers = []
		# Iterate through the map grid, and check for frontier cells
		for y in range(map_data.info.height):
			for x in range(map_data.info.width):
				cell = map_data.data[y * map_data.info.width + x]
				# A frontier cell is adjacent to both unknown and free space
				if cell == 0:  # Free space
					# Check neighboring cells for unknown space
					if self.is_adjacent_to_unknown(x, y, map_data):
						frontiers.append((x, y))
		return frontiers

	def is_adjacent_to_unknown(self, x, y, map_data):
		directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4 cardinal directions
		for dx, dy in directions:
			nx, ny = x + dx, y + dy
			if 0 <= nx < map_data.info.width and 0 <= ny < map_data.info.height:
				cell = map_data.data[ny * map_data.info.width + nx]
				if cell == -1:  # Unknown space
					return True
		return False

	def generate_frontier_goal(self, frontiers):
		if not frontiers:
			rospy.loginfo("No frontiers found!")
			return None
		# Choose the first frontier point as the goal
		goal = frontiers[0]  # You could implement a more sophisticated selection here
		return goal

if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)

	# Start the controller.
	controller = StudentController()

	# This will move the robot to a set of fixed waypoints.  You should not do this, since you don't know
	# if you can get to all of these points without building a map first.  This is just to demonstrate how
	# to call the function, and make the robot move as an example.
	controller.set_waypoints(((-4, -3), (-3, 0), (5, 0)))

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()

