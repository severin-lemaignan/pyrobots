from action import action, genom_request, ros_request

@action
def look_at(place):
    """ A simple 'look at' method that uses pr2SoftMotion.

	:param place: a dictionary with the x,y,z position of objects in space.
		      If a 'frame' key is found, use it as reference frame. Else
		      the world frame '/map' is assumed.
    """
    try:
	frame = place['frame']
    catch KeyError:
	frame = "/map"

    return look_at_xyz(place['x'], place['y'], place['z'], frame)

@action
def look_at_xyz(x,y,z, frame = "/map"):
    """ Look at via pr2SoftMotion.
    """
    print("Looking at " + str([x,y,z]) + " in " + frame)
    actions = [
        genom_request(	"pr2SoftMotion",
            "MoveHead",
            [x,y,z,frame]
        )
    ]

    return actions
###############################################################################
###############################################################################

@action
def look_at_ros(target):
        """ Create the client and the goal.

        :param reqs:
	- a dictionnary name which contains object position parameters
        """

	import roslib; roslib.load_manifest('novela_actionlib')
	import rospy

	import actionlib
	import pr2_controllers_msgs.msg
	import geometry_msgs.msg


	x = target['x']
	y = target['y']
	z = target['z']


        # Creates the SimpleActionClient, passing the type of the action
        # (MoveBaseAction) to the constructor.
	client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', pr2_controllers_msgs.msg.PointHeadAction)

	# Waits for the action server to come up
	client.wait_for_server()
        ok = client.wait_for_server()
        if not ok:
                print("Could not connect to the ROS client! Aborting action")
                return

        # Creates a goal to send to the action server.  
	goal = pr2_controllers_msgs.msg.PointHeadGoal()

        # Definition of the goal
	point = geometry_msgs.msg.PointStamped()
	point.header.frame_id = 'map'
	point.point.x = x
	point.point.y = y
	point.point.z = z

	goal.target = point
	goal.pointing_frame = 'high_def_frame'
	goal.min_duration = rospy.Duration(0.5)
	goal.max_velocity = 1.0

	return [ros_request(client, goal)]

###############################################################################

