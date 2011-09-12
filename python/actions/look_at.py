from action import action, genom_request, ros_request, wait
from helpers.jointstate import getjoint

###############################################################################
###############################################################################

@action
def look_at(place, callback = None):
    """ A simple 'look at' method that uses pr2SoftMotion.

	Uses look_at_xyz underneath.

	:param place: a dictionary with the x,y,z position of objects in space.
		      If a 'frame' key is found, use it as reference frame. Else
		      the world frame '/map' is assumed.
    """
    try:
	frame = place['frame']
    except KeyError:
	frame = "map"

    return look_at_xyz(place['x'], place['y'], place['z'], frame, callback)

###############################################################################

@action
def look_at_xyz(x,y,z, frame = "map", callback = None):
    """ Look at via pr2SoftMotion.
	
	:param x: the x coordinate
	:param y: the y coordinate
	:param z: the z coordinate
	:param frame: the frame in which coordinates are interpreted. By default, '/map'
    """
    print("Looking at " + str([x,y,z]) + " in " + frame)
    actions = [
        genom_request(	"pr2SoftMotion",
            "MoveHead",
            [x,y,z,frame],
	    wait_for_completion = False if callback else True,
	    callback = callback
        )
    ]

    return actions

###############################################################################

@action
def track(place):
    """ Tracks an object with the head.

	This uses pr2SoftMotion.

	This is a background action. Can be cancelled with stop_tracking .

	:param place: a dictionary with the x,y,z position of objects in space.
		      If a 'frame' key is found, use it as reference frame. Else
		      the world frame '/map' is assumed.
    """
    try:
	frame = place['frame']
    except KeyError:
	frame = "/map"

    return track_xyz(place['x'], place['y'], place['z'], frame)

###############################################################################

@action
def track_xyz(x,y,z, frame = "/map"):
    """ Tracks a position in space at via pr2SoftMotion.
	
	This is a background action. Can be cancelled with stop_tracking .

	:param x: the x coordinate
	:param y: the y coordinate
	:param z: the z coordinate
	:param frame: the frame in which coordinates are interpreted. By default, '/map'
    """
    print("Tracking " + str([x,y,z]) + " in " + frame)
    actions = [
        genom_request(	"pr2SoftMotion",
            "HeadTrack",
            [x,y,z,frame],
	    wait_for_completion = False
        )
    ]

    return actions

###############################################################################

@action
def stop_tracking():
	""" If running, interrupt a current track action.
	"""
	actions = [
		genom_request(	"pr2SoftMotion",
		    "HeadTrack",
		    abort = True
		)
	]
    	return actions

###############################################################################

@action
def look_at_ros(place):
        """ Create the client and the goal.

        :param place: a dictionary which contains object position parameters. 
	Cf look_at for details.
        """

	import roslib; roslib.load_manifest('novela_actionlib')
	import rospy

	import actionlib
	import pr2_controllers_msgs.msg
	import geometry_msgs.msg


	x = place['x']
	y = place['y']
	z = place['z']

	try:
	   frame = place['frame']
	except KeyError:
	   frame = "/map"

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
	point.header.frame_id = frame
	point.point.x = x
	point.point.y = y
	point.point.z = z

	goal.target = point
	goal.pointing_frame = 'high_def_frame'
	goal.min_duration = rospy.Duration(0.5)
	goal.max_velocity = 1.0

	return [ros_request(client, goal)]

###############################################################################

@action
def glance_to(place, frame='/map'): 
        """ Glance to via pr2SoftMotion
        """

        head_tilt = getjoint('head_tilt_joint')
        head_pan = getjoint('head_pan_joint')

        actions = [
                genom_request("pr2SoftMotion", "MoveHead",
                         [ place['x'], place['y'], place['z'], frame ]
                        ),
                wait(2),
                genom_request("pr2SoftMotion", "GotoQ",
                        ["HEAD", 0, 0.0, head_pan,  head_tilt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        )
                ]
        return actions

