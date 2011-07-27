import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the _MoveBaseAction, including the
# goal message and the result message.
import actionlib_tutorials.msg



###############################################################################
###############################################################################

def move_base_action_client(x, y, z, qx, qy, qz, qw):

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
	
	return client, goal
	
###############################################################################

def ros_nav(target):

	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('move_base_action_client_py')
		
		x = target.get('x', "NEVER")
		y = target.get('y', "NEVER")
		z = target.get('z', "NEVER")
		
		qx = target.get('qx', "NEVER")
		qy = target.get('qy', "NEVER")
		qz = target.get('qz', "NEVER")
		qw = target.get('qw', "NEVER")
		
		result = move_base_action_client(x, y, z, qx, qy, qz, qw)
			print "Result:", ', '.join([str(n) for n in result.sequence])

	except rospy.ROSInterruptException:
			print "program interrupted before completion"
