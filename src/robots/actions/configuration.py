import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

import random
from robots.exception import RobotError
from robots.action import *
from robots.helpers import trajectory, postures

import os

@tested("22/02/2012")
@action
def enabledevileye(robot):
    os.system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 3")

@tested("22/02/2012")
@action
def disabledevileye(robot):
    os.system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 1")
 
@tested("15/06/2012")
@action
def setpose(robot, posture, callback = None, part = None, collision_avoidance = False, relative = False, obj = 'PR2_ROBOT', support = 'NO_NAME'):
    """
    Set the PR2 base and internal joints in a given configuration.
    
    Beware: if collision_avoidance is not set to True, not collision detection is done.
    This can result in a dangerous robot behaviour. Note also that collision avoidance
    is available for the right arm only.
    
    While you can set the global position of the PR2 robot with this action, please
    use goto in the 'nav' category for obstacle aware navigation.
   
    If posture = {} and part = 'PR2', all parameters take 0 for value
 
    :param posture: a posture, ie, a dictionary built as follow:
    
      * Key '''BASE''': array of 6 floats [x,y,z,rx,ry,rz], in meters and in radians. The position is given in the world frame.
      * Key '''RARM''': array of 7 joint angles for the right arm, in radians,
      * Key '''LARM''': array of 7 joint angles for the left arm, in radians,
      * Key '''TORSO''': the height of the torso ([0-30]), in centimeters,
      * Key '''HEAD''': array of pan and tilt values, in radians,
    
    Note that none of the items of the dictionary are mandatory. If no robot part is 
    selected (see the 'part' parameter below), the smallest set of part is selected
    that allow execution of the pose.

    'posture' may also be the name of a pre-recorded posture, as available from
    the posture library (cf helpers/postures.py).

    :param part: (optional) force only a certain part of the PR2 to execute to posture.
    Allowed values: 'BASE', 'RARM', 'LARM', 'ARMS', 'PR2', 'PR2SYN', 'TORSO', 'PR2NOHEAD', 'HEAD'
    :param collision_avoidance: (optional), If true, the robot will be consider obstacles 
    in its environment. If true, you have to precise part = 'RARM'.
    :param relative: (default: False) if set to true, the posture is considered as
    being relative to the current configuration.
    :param obj: (optional) set which objet the robot has to avoid during the movement. 
    You can give only one object.
    :param support: (optional) set which support the robot has to avoid during the movement.
    You can give only one support.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.
    """
    if part and part not in ['BASE', 'RARM', 'LARM', 'ARMS', 'PR2', 'PR2SYN', 'TORSO', 'PR2NOHEAD', 'HEAD']:
        print("'setpose' for part " + part + " is not implemented.")

    logger.debug("Setting pose " + str(posture))

    current_pose = getpose(robot)

    forced_part = part

    if not isinstance(posture, dict):
        try:
            posture = postures.read()[posture]
        except KeyError:
            raise RobotError("Posture %s not found!" % posture)

    try:
        x, y, z, rx, ry ,rz = posture['BASE']
        if not forced_part:
            part = 'BASE'
    except KeyError:
        x, y, z, rx, ry ,rz = [0.0] * 6
    
    try:
        rq1, rq2, rq3, rq4, rq5, rq6, rq7 = posture['RARM']
        if not forced_part:
            part = 'RARM'
    except KeyError:
        rq1, rq2, rq3, rq4, rq5, rq6, rq7 = current_pose["RARM"]

    try:
        lq1, lq2, lq3, lq4, lq5, lq6, lq7 = posture['LARM']
        if not forced_part:
            if part == 'RARM':
                part = 'ARMS'
            else:
                part = 'LARM'
    except KeyError:
        lq1, lq2, lq3, lq4, lq5, lq6, lq7 = current_pose['LARM']

    try:
        [torso] = posture['TORSO']
        if not forced_part:
            if part in ['LARM', 'RARM', 'ARMS']:
                part = 'PR2NOHEAD'
            elif not part:
                part = 'TORSO'
            else:
                raise RobotError("Can not move only one arm and the torso in one call. Please call gotopostureraw twice.")
    except KeyError:
        [torso] = current_pose["TORSO"]

    try:
        pan, tilt = posture['HEAD']
        if not forced_part:
            if part == 'PR2NOHEAD':
                part = 'PR2'
            elif not part:
                part = 'HEAD'
            else:
                raise RobotError("Can not move only one arm or the torso and the head in one call. Please call gotopostureraw twice.")
    except KeyError:
         pan, tilt= current_pose["HEAD"]


    if not collision_avoidance:
        actions = [
        genom_request(
                    "pr2SoftMotion", 
                    "GotoQ",
                [forced_part if forced_part else part,
                1 if relative else 0,
                x,y,z,
                rx,ry,rz,
                torso, 
                pan, tilt, # head
                0.0, # laser tilt
                rq1, rq2, rq3, rq4, rq5, rq6, rq7, 0.0, 0.0, # Right arm
                lq1, lq2, lq3, lq4, lq5, lq6, lq7, 0.0, 0.0], # Left arm
                wait_for_completion = False if callback else True,
                callback = callback)
            ]

    else: #collision_avoidance == True
        if not part == 'RARM':
            raise RobotError("Can't use collision avoidance with other part than RARM")

        actions = [
        genom_request(
            "mhp",
            "ArmPlanTask",
                [0,
                'GEN_TRUE',
                 'MHP_ARM_TAKE_TO_FREE',
                0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                rq1, rq2, rq3, rq4, rq5, rq6, rq7,
                obj,
                support,
                'NO_NAME',
                'GEN_FALSE',
                0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
                0, # Rotation type
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), # x y z rx ry rz
        
        genom_request("mhp", "ArmSelectTraj", [0]),
            genom_request("pr2SoftMotion", 
            "TrackQ",['mhpArmTraj', 'PR2SM_TRACK_POSTER', part],
            wait_for_completion = False if callback else True,
            callback = callback)
        ]

    return actions

def getpose(robot):
    """Returns to current whole-body pose of the PR2 robot.
    """
    try:
        import rospy
        from sensor_msgs.msg import JointState
        logger.debug("Reading PR2 pose... (10 sec timeout)")
        msg = rospy.client.wait_for_message('/joint_states', JointState, timeout = 10)
    except rospy.exceptions.ROSException:
        logger.error("Could not read topic /joint_states. Right ROS_MASTER_URI? "
        "PR2 started?")
        return None

    raw = msg.position

    pose = {}
    pose["HEAD"] = [round(raw[14],3), round(raw[15],3)] # head_pan_joint, head_tilt_joint
    pose["TORSO"] = [round(raw[12], 3)] # torso_lift_joint

    # Arm joints order:
    # shoulder_pan_joint
    # shoulder_lift_joint
    # upper_arm_roll_joint 
    # elbow_flex_joint
    # forearm_roll_joint
    # wrist_flex_joint
    # wrist_roll_joint

    pose["RARM"] = [round(raw[18], 3), round(raw[19], 3), round(raw[17], 3), round(raw[21], 3), round(raw[20], 3), round(raw[22], 3), round(raw[23], 3), ]
    pose["LARM"] = [round(raw[32], 3), round(raw[33], 3), round(raw[31], 3), round(raw[35], 3), round(raw[34], 3), round(raw[36], 3), round(raw[37], 3), ]
    return pose

@tested("15/06/2012")
@action
def manipose(robot, nohead = True, callback = None):
    """
    Quick method to set the PR2 joints in manip configuration. 
    It's useful when the robot to handle objects in in free spaces.

    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """
    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'
    
    pose = postures.read()
    posture = pose['MANIP']
    
    return setpose(robot, posture, callback, part)

@tested("22/02/2012")
@broken
@action
def restpose(robot, nohead = True, callback = None):
    """
    Quick method to set the PR2 joints in rest configuration. 
    You have the choice with three rest configuration. This choice is random.
    
    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """


    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'

    n = random.randint(1,3)
    pose = postures.read()
    posture = pose['REST' + str(n)]

    return setpose(robot, posture, callback, part)

@tested("08/03/2012")
@action
def settorso(robot, height = 15, callback = None):
    """
    Set the PR2 torso height.

    :param height: The height of the torso, in centimeters. Defaults to 15
    cm.

    Height is clamped into [0, 30].

    :param callback: (optional) If given, the action is non-blocking,
    and the callback is invoked at the activity completion.

    """
    if height < 0:
        height = 0
    if height > 30:
        height = 30
    return setpose(robot, {'TORSO': [float(height) / 100]}, callback)


@tested("22/02/2012")
@action
def tuckedpose(robot, callback = None, nohead = True):
    """
    Tucks the robot arms.

    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """

    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'

    pose = postures.read()
    posture = pose['TUCKED']

    return setpose(robot, posture, callback, part)

@broken
@action
def idle(robot, choice = None,callback = None):
    """ action to do when nothing to do.

       :param choice: default to -1, this param make it possible to choose
       the action to do. if choice>10 or choice<0 the action to do is randomly
       choosed.
       :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.
    """
    
    c = -1
    if not choice or (choice > 10 or choice < 0):
        c = random.randint(0,9);
    else:
       c = choice

    part = 'PR2'
    pose = postures.read()
 
    actions = []
    if (c == 0):
        posture1 = pose['TUCKED']
        posture2 = pose['WATCH_LOOKING']
        actions = setpose(robot, posture1,callback,part) + setpose(robot, posture2,callback,part) + setpose(robot, posture1,callback,part) 
    elif(c == 1):
        short1 = trajectory.Trajectory('short1')
        actions = [
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + short1.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ short1.abspath(),"PR2SM_TRACK_FILE", "PR2"])
               ]
    elif(c == 2):
        scratch = trajectory.Trajectory('scratch')
        actions = [
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + scratch.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ scratch.abspath(),"PR2SM_TRACK_FILE", "PR2"])
               ]
    elif(c == 3):
        changeCrossing = trajectory.Trajectory('changeCrossing')
        actions = [
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + changeCrossing.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ changeCrossing.abspath(),"PR2SM_TRACK_FILE", "PR2"])
               ]

    elif(c == 4):
        etirement = trajectory.Trajectory('etirement')
        actions = [
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + etirement.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ etirement.abspath(),"PR2SM_TRACK_FILE", "PR2"])
               ]

    elif(c == 5):
        lookHuman = trajectory.Trajectory('lookHuman')
        actions = [
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + lookHuman.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ lookHuman.abspath(),"PR2SM_TRACK_FILE", "PR2"])
               ]

    elif(c == 6):
        lookUp = trajectory.Trajectory('lookUp')
        actions = [
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + lookUp.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ lookUp.abspath(),"PR2SM_TRACK_FILE", "PR2"])
               ]

    elif(c == 7):
        scratch = trajectory.Trajectory('scratch')
        actions = [
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + scratch.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ scratch.abspath(),"PR2SM_TRACK_FILE", "PR2"])
               ]

    elif(c == 8):
        posture1 = pose['TUCKED']
        posture2 = pose['WATCH_LOOKING']
        actions = setpose(robot, posture1,callback,part) + setpose(robot, posture2,callback,part) + setpose(robot, posture1,callback,part)
    elif(c == 9):
        posture1 = pose['TUCKED']
        posture2 = pose['WATCH_LOOKING']
        actions = setpose(robot, posture1,callback,part) + setpose(robot, posture2,callback,part) + setpose(robot, posture1,callback,part)

    return actions 


@action
def movearm(robot, target):
    """ Moves the robot right arm to a pose.
    
        :param target: the destination for the arm wrist
    """

    pose = robot.poses[target]
    
    client = None
    goal = None
    
    if robot.hasROS():
        import rospy
        import actionlib
        import arm_navigation_msgs.msg
        client = actionlib.SimpleActionClient('move_right_arm', arm_navigation_msgs.msg.MoveArmAction)

        ok = client.wait_for_server()
        if not ok:
            logger.error("Could not connect to the ROS client for arm navigation!"
			 " Aborting action")
            return

        # Creates a goal to send to the action server.  
        goal = arm_navigation_msgs.msg.MoveArmGoal()

        # Definition of the goal
	goal.motion_plan_request.group_name = "right_arm"
	goal.motion_plan_request.num_planning_attempts = 1
	goal.motion_plan_request.planner_id = ""
	goal.planner_service_name = "ompl_planning/plan_kinematic_path"
	goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)

	pc = arm_navigation_msgs.msg.PositionConstraint()
	pc.header.stamp = rospy.Time.now()
	pc.header.frame_id = pose['frame'] 
	pc.link_name = 'r_wrist_roll_link'
	pc.position.x = pose['x']
	pc.position.y = pose['y']
	pc.position.z = pose['z']

	pc.constraint_region_shape.type = arm_navigation_msgs.msg.Shape.BOX
	pc.constraint_region_shape.dimensions = [0.02, 0.02, 0.02]
	pc.constraint_region_orientation.w = 1.0

	goal.motion_plan_request.goal_constraints.position_constraints.append(pc)

	oc = arm_navigation_msgs.msg.OrientationConstraint()
	oc.header.stamp = rospy.Time.now()
	oc.header.frame_id = pose['frame']
	oc.link_name = 'r_wrist_roll_link'
	oc.orientation.x = 0.
	oc.orientation.y = 0.
	oc.orientation.z = 0.
	oc.orientation.w = 1.

	oc.absolute_roll_tolerance = 0.04
	oc.absolute_pitch_tolerance = 0.04
	oc.absolute_yaw_tolerance = 0.04
	oc.weight = 1.

	goal.motion_plan_request.goal_constraints.orientation_constraints.append(oc)
    else:
        # Useful for debugging purpose, without the actual robot
        client = "ROS arm_navigation"
        goal = pose
    

    return [ros_request(client, 
            goal, 
            wait_for_completion = False,
            callback = None
        )]


