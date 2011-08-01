import roslib; roslib.load_manifest('navigation_actionlib')
import rospy

import actionlib
from navigation_actionlib.msg import *

from action import action, ros_request

###############################################################################
###############################################################################

@action
def ros_nav(target):

	x = target['x']
	y = target['y']
	z = target['z']
	qx = target['qx']
	qy = target['qy']
	qz = target['qz']
	qw = target['qw']
	print x, y , z, qx, qy, qz, qw
	# Creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor.
	client = actionlib.SimpleActionClient('move_base', navigation_actionlib.msg.NavigationAction)
		
	# Creates a goal to send to the action server.  
	goal = navigation_actionlib.msg.NavigationGoal()

	# Definition of the goal
	goal.target_pose.header.frame_id = "base_link"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = z

	goal.target_pose.pose.orientation.x = qx
	goal.target_pose.pose.orientation.y = qy
	goal.target_pose.pose.orientation.z = qz
	goal.target_pose.pose.orientation.w = qw
	
	return [ros_request(client, goal)]
	
###############################################################################

