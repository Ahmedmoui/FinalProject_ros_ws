#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to do some math
import numpy as np

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


# A callback to deal with the LaserScan messages.
def callback(scan):
	# Every time we get a laser scan, calculate the shortest scan distance in front
	# of the robot, and set the speed accordingly.  We assume that the robot is 38cm
	# wide.  This means that y-values with absolute values greater than 19cm are not
	# in front of the robot.  It also assumes that the LiDAR is at the front of the
	# robot (which it actually isn't) and that it's centered and pointing forwards.
	# We can work around these assumptions, but it's cleaner if we don't

	# This calculates the absolute y coordinates of the scan contacts.
	abs_y = np.abs([r * np.sin(theta) for r, theta in zip(scan.ranges, np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges)))])

	# Find the minimum of the values for which the absolute value of y <= 0.19
	shortest = np.min(np.array(scan.ranges)[abs_y <= 0.19])

	# The previous two lines of code is pretty gnarly, since it uses numpy.  It is,
	# however, really fast.  If you're confused, then take a look at this (slower)
	# version of the code, which does the same thing.
	#shortest = 1000000
	#for r, theta in zip(scan.ranges, np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))):
	#	if np.abs(r * np.sin(theta)) <= 0.19:
	#		shortest2 = min(r, shortest2)

	# Create a twist and fill in all the fields.
	t = Twist()
	t.linear.x = 0.0
	t.linear.y = 0.0
	t.linear.z = 0.0
	t.angular.x = 0.0
	t.angular.y = 0.0
	t.angular.z = 0.0

	# Instead of just stopping, we're going to set the speed based on the distance.
	# As we get closer to the required distance, we slow down.  If we're too close,
	# we back up.  To smooth things out, we're going to use a tanh function, which
	# looks like a smoothed-out step function, which asymptotes to -1 and +1.
	# There's nothing magical about this function, but I like it for this sort of
	# thing.  We multiply the distance by 5 to make the function more step-like.
	# Play around with this value to see what effect it has.
	t.linear.x = np.tanh((shortest - 1.0) * 5)

	# If we're close enough, say within 1cm, then just stop.
	if np.abs(shortest - 1.0) < 0.01:
		t.linear.x = 0

	# Send the command to the robot.
	publisher.publish(t)

	# Print out a log message to the INFO channel to let us know it's working.
	rospy.loginfo(f'Shortest: {shortest} => {t.linear.x}')


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('stopper', argv=sys.argv)

	# Set up a publisher.  The default topic for Twist messages is cmd_vel.
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
	subscriber = rospy.Subscriber('base_scan', LaserScan, callback, queue_size=10)

	# Now that everything is wired up, we just spin.
	rospy.spin()