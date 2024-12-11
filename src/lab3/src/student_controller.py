#!/usr/bin/env python3


import sys
import rospy
import signal
import numpy as np

from controller import RobotController

from exploring import find_all_possible_goals, find_best_point, plot_with_explore_points, find_waypoints, convert_pix_to_x_y, convert_x_y_to_pix
from path_planning import convert_image, dijkstra

class StudentController(RobotController):
	'''
	This class allows you to set waypoints that the robot will follow.  These robots should be in the map
	coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
	StudentDriver.
	'''
	def __init__(self):
		super().__init__()
		self.lock = False
		self.Map = None

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
		rospy.loginfo(f'Distance: {distance}')
		if len(controller._waypoints) <= 1:
			command.angular.z = 5

		
		

	def map_update(self, point, map, map_Metadata):
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

		# It's possible that the position passed to this function is None.  This try-except block will deal
		# with that.  Trying to unpack the position will fail if it's None, and this will raise an exception.
		# We could also explicitly check to see if the point is None.
		# try:
		# 	# The (x, y) position of the robot can be retrieved like this.
		# 	robot_position = (point.point.x, point.point.y)
		# 	des = find_best_point(map_2D,All_possible_goals,robot_position)
		# 	rospy.loginfo(f'Robot is at {robot_position} {point.header.frame_id}')
		# except:
		# 	rospy.loginfo('No odometry information')


		map_size = (map_Metadata.width, map_Metadata.height)
		resolution = map_Metadata.resolution
		map_2D = np.array(map.data).reshape((map_size))

		possible_pix = find_all_possible_goals(map_2D)
		#rospy.loginfo(f'Frontier points {possible_pix}')

		# All_possible_goals = []
		# for pix in possible_pix:
		# 	All_possible_goals.append((convert_pix_to_x_y(map_size,pix,map_Metadata.resolution)))
		
		if point is not None:
			robot_position = (point.point.x, point.point.y)
			x, y = robot_position

			rob_pos =  (int((x - map_Metadata.origin.position.x)/ resolution), int((y - map_Metadata.origin.position.y)/ resolution))

			rospy.loginfo(f'Robot is at {robot_position} {point.header.frame_id}')

		if self.lock == False:
			
			des = find_best_point(map_2D,possible_pix,rob_pos)
			Goal_point = ((des[0] * resolution) + map_Metadata.origin.position.x,(des[1] * resolution) + map_Metadata.origin.position.y)
			rospy.loginfo(f'Robot pos, Des {robot_position} ,{Goal_point} ,res {resolution}')
			rospy.loginfo(f'Robot pos, Des {convert_pix_to_x_y(map_size,rob_pos,resolution)} {convert_pix_to_x_y(map_size,des,resolution)}')

			path = dijkstra(map_2D,rob_pos,des)
			
			Path_as_waypoints = find_waypoints(map_2D,path)

			converted_path = []
			for point in Path_as_waypoints:
				converted_path.append((point[0]*resolution + map_Metadata.origin.position.x, point[1]*resolution + map_Metadata.origin.position.y))

				if converted_path is not None:
					controller.set_waypoints(((converted_path)))
					self.lock = True
					rospy.loginfo(f'Waypoints {len(controller._waypoints)}')

		if len(controller._waypoints) <= 1:
			self.lock = False



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)

	# Start the controller.
	controller = StudentController()
		
	# This will move the robot to a set of fixed waypoints.  You should not do this, since you don't know
	# if you can get to all of these points without building a map first.  This is just to demonstrate how
	# to call the function, and make the robot move as an example.
	
	#controller.set_waypoints(((-4, -3), (-3, 0), (5, 0)))

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()
