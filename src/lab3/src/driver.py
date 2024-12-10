#!/usr/bin/env python3


import rospy
import sys
import math


from math import atan2, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


import actionlib
import tf


from lab2.msg import NavTargetAction, NavTargetResult, NavTargetFeedback




class Driver:
   def __init__(self, position_source, threshold=0.1):
       # Goal will be set later. The action server will set the goal; you don't set it directly
       self.goal = None
       self.threshold = threshold


       self.state = None


       self.transform_listener = tf.TransformListener()


       # Publisher before subscriber
       self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
       self.target_pub = rospy.Publisher('current_target', Marker, queue_size=1)


       # Subscriber after publisher
       self.sub = rospy.Subscriber('base_scan', LaserScan, self._callback, queue_size=1)


       # Action client
       self.action_server = actionlib.SimpleActionServer('nav_target', NavTargetAction, execute_cb=self._action_callback, auto_start=False)
       self.action_server.start()


   @classmethod
   def zero_twist(cls):
       """This is a helper class method to create and zero-out a twist"""
       command = Twist()
       command.linear.x = 0.0
       command.linear.y = 0.0
       command.linear.z = 0.0
       command.angular.x = 0.0
       command.angular.y = 0.0
       command.angular.z = 0.0


       return command


   # Respond to the action request.
   def _action_callback(self, goal):
       """ This gets called when an action is received by the action server
       @goal - this is the new goal """
       rospy.loginfo(f'Got an action request for ({goal.goal.point.x:.2f}, {goal.goal.point.y:.2f})')


       # Set the goal.
       self.goal = goal.goal


       # Build a marker for the goal point
       #   - this prints out the green dot in RViz (the current goal)
       marker = Marker()
       marker.header.frame_id = goal.goal.header.frame_id
       marker.header.stamp = rospy.Time.now()
       marker.id = 0
       marker.type = Marker.SPHERE
       marker.action = Marker.ADD
       marker.pose.position = goal.goal.point
       marker.pose.orientation.x = 0.0
       marker.pose.orientation.y = 0.0
       marker.pose.orientation.z = 0.0       
       marker.pose.orientation.w = 1.0
       marker.scale.x = 0.3
       marker.scale.y = 0.3
       marker.scale.z = 0.3
       marker.color.r = 0.0
       marker.color.g = 1.0
       marker.color.b = 0.0
       marker.color.a = 1.0


       # Wait until we're at the goal.  Once we get there, the callback that drives the robot will set self.goal
       # to None.
       while self.goal:
           self.target_pub.publish(marker)
           rospy.sleep(0.1)


       rospy.loginfo('Action completed')


       # Build a result to send back
       result = NavTargetResult()
       result.success.data = True


       self.action_server.set_succeeded(result)


       # Get rid of the marker
       marker.action = Marker.DELETE
       self.target_pub.publish(marker)


   def _callback(self, lidar):
       # If we have a goal, then act on it, otherwise stay still
       if self.goal:
           # Update the timestamp on the goal and figure out where it is now in the base_link frame.
           self.goal.header.stamp = rospy.Time.now()
           target = self.transform_listener.transformPoint('base_link', self.goal)


           rospy.loginfo(f'Target: ({target.point.x:.2f}, {target.point.y:.2f})')


           # Are we close enough?  If so, then remove the goal and stop
           distance = sqrt(target.point.x ** 2 + target.point.y ** 2)


           feedback = NavTargetFeedback()
           feedback.distance.data = distance
           self.action_server.publish_feedback(feedback)


           if distance < self.threshold:
               self.goal = None
               command = Driver.zero_twist()
           else:
               command = self.get_twist((target.point.x, target.point.y), lidar)
       else:
           command = Driver.zero_twist()


       self.cmd_pub.publish(command)






   def get_twist(self, target, lidar):


       command = Driver.zero_twist()


       # TODO:
       #  Step 1) Calculate the angle the robot has to turn to in order to point at the target
       #  Step 2) Set your speed based on how far away you are from the target, as before
       #  Step 3) Add code that veers left (or right) to avoid an obstacle in front of it


       # Set where we are going and the angle of turns
       target_x, target_y = target
       anglet = atan2(target_y, target_x)


       # use the distance function to get target distance
       distancet = sqrt(target_x ** 2 + target_y ** 2)


       # Clamp linear speed between 0.5 and the distance
       command.linear.x = min(0.5, distancet)
       command.angular.z = 2.0 * anglet




       if lidar:
           # front_angles = range(len(lidar.ranges) // 3, 2 * len(lidar.ranges) // 3)


           total_readings = len(lidar.ranges)


           # Focus on only the front 1/3 region of the robots view
           start_index = total_readings // 3 
           end_index = 2 * total_readings // 3
           front_angles = range(start_index, end_index)


           # min_distance = min([lidar.ranges[i] for i in front_angles if lidar.ranges[i] > 0.1], default=float('inf'))


           min_distance = float('inf')


           # Go through the front angles to find the minimum distance
           for i in front_angles:
               distance = lidar.ranges[i]
              
               if distance > 0.1:
                   min_distance = min(min_distance, distance)


           if min_distance < 1:  # dist from wa ll, we need custom logic when were close
               # added global so I can stop alternation
               if self.state == None:
                   command.linear.x = 0.0  # stop straigth




                   mid_index = len(lidar.ranges) // 2
                   left_ranges = [r for r in lidar.ranges[:mid_index] if r > 0.1]
                   right_ranges = [r for r in lidar.ranges[mid_index:] if r > 0.1]




                   min_left_distance = min(left_ranges, default=float('inf'))
                   min_right_distance = min(right_ranges, default=float('inf'))




                   if min_left_distance > min_right_distance:
                       print("goright")
                       command.angular.z = 0.5 #go r
                       self.state = True
                   else:
                       print("goleft")
                       command.angular.z = -0.5  # go l
                       self.state = False
               else:
                   # we need to prevent jittering
                   if self.state:
                       command.angular.z = 1
                   else:
                       command.angular.z = -1 # bigger rotation so we get away from wall


       return command

if __name__ == '__main__':
   rospy.init_node('driver', argv=sys.argv)


   driver = Driver('odom')


   rospy.spin()
