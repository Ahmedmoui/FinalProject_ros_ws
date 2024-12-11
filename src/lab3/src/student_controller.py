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
		self.mapUpdate_iteration = 0
		self.rob_pos = None

		
		#command = Driver.zero_twist()
		#command.angular.z = 2
		#Driver._cmd_pub.publish(command)

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
		Period = 3
		waypointsLeftInQ = 3
		#rospy.loginfo_throttle_identical(Period,f'Distance: {distance}')
		try:
			rospy.loginfo_throttle_identical(Period,f'Waypoints left: {len(controller._waypoints)}')
			if len(controller._waypoints) <= waypointsLeftInQ:
				self.lock = False
				rospy.loginfo('Path Lock off')
			elif len(controller._waypoints) == 0:
				rospy.loginfo_throttle_identical(Period,'Reached end of path "Waiting for new path"')
				self.generate_path(self.Map,self.rob_pos,self.map_Metadata,self.resolution)

		except:
			rospy.loginfo_throttle_identical(Period,'No Waypoints.')


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

		map_size = (map_Metadata.width, map_Metadata.height)
		resolution = map_Metadata.resolution
		map_2D = np.array(map.data).reshape((map_size))
		self.Map = map_2D
		self.map_metadata = map_Metadata
		self.resolution = resolution

		#rospy.loginfo(f'Frontier points {possible_pix}')

		# All_possible_goals = []
		# for pix in possible_pix:
		# 	All_possible_goals.append((convert_pix_to_x_y(map_size,pix,map_Metadata.resolution)))
		
		if point is not None:
			robot_position = (point.point.x, point.point.y)
			x, y = robot_position

			rob_pos =  (int((x - map_Metadata.origin.position.x)/ resolution), int((y - map_Metadata.origin.position.y)/ resolution))
			self.rob_pos = rob_pos
			rospy.loginfo(f'Robot is at {robot_position} {point.header.frame_id}')
			rospy.loginfo(f'map update #{self.mapUpdate_iteration}')

			# if self.mapUpdate_iteration == 0:
			# 	#controller.set_waypoints(((tuple((point.point.x+1, point.point.y+1)))))
			# 	rospy.loginfo(f'1st map update{(point.point.x, point.point.y)}')
			self.mapUpdate_iteration += 1

			if self.lock == False:

				self.generate_path(map_2D,rob_pos,map_Metadata,resolution)

			if controller._waypoints is not None:
				if len(controller._waypoints) == 0:
					self.lock = False
					rospy.loginfo('Path Lock off')

	

	def generate_path(self, map_2D, rob_pos, map_Metadata,resolution):
		possible_pix = find_all_possible_goals(map_2D)
		des = find_best_point(map_2D,possible_pix,self.rob_pos)
		Goal_point = ((des[0] * resolution) + map_Metadata.origin.position.x,(des[1] * resolution) + map_Metadata.origin.position.y)
					
		map_2D[rob_pos[0], rob_pos[1]] = 0
		path = dijkstra(map_2D,rob_pos,des)
					
		Path_as_waypoints = find_waypoints(map_2D,path)

		converted_path = []
		for point in Path_as_waypoints:
			converted_path.append((point[0]*resolution + map_Metadata.origin.position.x, point[1]*resolution + map_Metadata.origin.position.y))
		if converted_path is not None:
			controller.set_waypoints(((converted_path)))
			self.lock = True
			rospy.loginfo('Path Lock On')

		
	def convert_pos(point, map_size, resolution, map_metadata):
		return(point[0]*resolution + map_Metadata.origin.position.x, point[1]*resolution + map_Metadata.origin.position.y)


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
