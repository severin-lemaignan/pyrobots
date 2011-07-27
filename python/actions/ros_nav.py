import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the _MoveBaseAction, including the
# goal message and the result message.
import actionlib_tutorials.msg

from action import action, ros_request

###############################################################################
###############################################################################

@action
def ros_nav(target):

    x, y, z, qx, qy, qz, qw = target.values()

	# Creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor.
	client = actionlib.SimpleActionClient('move_base_action', actionlib_tutorials.msg.MoveBaseAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	# Creates a goal to send to the action server.  
	goal = actionlib_tutorials.msg.MoveBaseActionGoal()

	# Definition of the goal
	goal.target_pose.header.frame_id = "base_link"
	goal.target_pose.header.stamp = ros.Time.now()

	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = z

	goal.target_pose.pose.orientation.x = qx
	goal.target_pose.pose.orientation.y = qy
	goal.target_pose.pose.orientation.z = qz
	goal.target_pose.pose.orientation.w = qw
	
	return [ros_request(client, goal)]
	
###############################################################################

