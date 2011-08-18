import roslib; roslib.load_manifest('novela_actionlib')
import rospy

import actionlib
import move_base_msgs.msg

import action


###############################################################################
###############################################################################

@action.action
def goto(target, callback = None):
	""" Moves the robot base to a given target, using ROS 2D navigation stack.
	"""
	x = target['x']
	y = target['y']
	z = target['z']
	qx = target['qx']
	qy = target['qy']
	qz = target['qz']
	qw = target['qw']

      

	print x, y , z, qx, qy, qz, qw
	# Creates the SimpleActionClient, passing the type of the action
	# (Navigationction) to the constructor.
	client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

	ok = client.wait_for_server()
	if not ok:
		#logger.error("Could not connect to the ROS client! Aborting action")
		print("Could not connect to the ROS client! Aborting action")
		return



	# Creates a goal to send to the action server.  
	goal = move_base_msgs.msg.MoveBaseGoal()

	# Definition of the goal
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now();

	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = z

	goal.target_pose.pose.orientation.x = qx
	goal.target_pose.pose.orientation.y = qy
	goal.target_pose.pose.orientation.z = qz
	goal.target_pose.pose.orientation.w = qw
	
	

	return [action.ros_request(client, 
				goal, 
				wait_for_completion = False if callback else True,
				callback = callback
		)] # REturn a non-blocking action. Useful to be able to cancel it later!

	
###############################################################################

@action.action
def cancel():
	""" Interrupt a navigation task.
	"""
	client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	
	ok = client.wait_for_server()
	if not ok:
		#logger.error("Could not connect to the ROS client! Aborting action")
		print("Could not connect to the ROS client! Aborting action")
		return

	# Creates a goal to send to the action server.  
	goal = move_base_msgs.msg.MoveBaseGoal()

	return [action.ros_request(client, goal)]

