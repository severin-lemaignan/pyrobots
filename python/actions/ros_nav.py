import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the _MoveBaseAction, including the
# goal message and the result message.
import actionlib_tutorials.msg


import json
import sys
sys.path.append('/home/ncourbet/hri-scripts/share/')

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
	
	
	    # Sends the goal to the action server.
	    client.send_goal(goal)

	    # Waits for the server to finish performing the action.
	    client.wait_for_result()

	    # Prints out the result of executing the action
	    return client.get_result()  # A _MoveBaseActionResult

###############################################################################

def ros_nav(choice):

	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('move_base_action_client_py')

		f = open('/home/ncourbet/hri-scripts/share/novela_places.json','rb')
		json_data=f.read()

		#print json_data
		symbolic_places = json.loads(json_data)

		#print 'Which symbolic places?'

		#choice = raw_input()


		for key, value in symbolic_places.items():
	
			if key == choice:
				#print ('%s is at (%d, %d, %d, %d, %d, %d, %d,)' % (key, value['a'], value['b'], value['c'], value['d'], value['e'], value['f'], value['g']))
				 position, x, y, z, qx, qy, qz, qw = [ str(key) , \
						value['a'], value['b'], \
						value['c'], value['d'], \
						value['e'], value['f'], value['g'] ]

			else:
				print 'This position doesn\'t exist'


		result = move_base_action_client(x, y, z, qx, qy, qz, qw)
			print "Result:", ', '.join([str(n) for n in result.sequence])

	except rospy.ROSInterruptException:
		print "program interrupted before completion"
