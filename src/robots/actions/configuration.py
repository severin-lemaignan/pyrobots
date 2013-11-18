import logging; logger = logging.getLogger("robot." + __name__)

import random
from robots.lowlevel import *
from robots.exception import RobotError
from robots.action import *
from robots.helpers import trajectory

import os

@tested("04/10/2012")
@action
@workswith(ROS)
def enabledevileye(robot):
    os.system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 3")

@tested("04/10/2012")
@action
@workswith(ROS)
def disabledevileye(robot):
    os.system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 1")
 
@tested("04/10/2012")
@action
@workswith({POCOLIBS:["mhp", "lwr"]})
@workswith({POCOLIBS:["mhp", "pr2SoftMotion"]})
@workswith(NAOQI)
def setpose(robot, posture, callback = None, part = None, collision_avoidance = False, relative = False, obj = 'PR2_ROBOT', support = 'NO_NAME'):
    """
    Set the robot base and internal joints in a given configuration.
    
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
    the posture library (cf helpers/postures.py) or other robot-dependent
    prerecorded postures.

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
    if robot.supports(POCOLIBS) and robot.hasmodule("pr2SoftMotion"):
        return setposePocolibsPR2(robot, 
                           posture, 
                           callback, 
                           part, 
                           collision_avoidance, 
                           relative, 
                           obj, 
                           support)

    if robot.supports(NAOQI):
        return setposeNAOqi(robot, posture, relative)

    logger.warning("This robot does not support pose setting")
    return []


def setposePocolibsPR2(robot, posture, callback = None, part = None, collision_avoidance = False, relative = False, obj = 'PR2_ROBOT', support = 'NO_NAME'):
    """set pose on the PR2 with the LAAS motion environment.
    """

    if part and part not in ['BASE', 'RARM', 'LARM', 'ARMS', 'PR2', 'PR2SYN', 'TORSO', 'PR2NOHEAD', 'HEAD']:
        print("'setpose' for part " + part + " is not implemented.")

    logger.debug("Setting pose " + str(posture))

    current_pose = robot.state.getpose(robot)

    forced_part = part

    if not isinstance(posture, dict):
        try:
            posture = robot.postures[posture]
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
        if robot.hasmodule("lwr"):
            actions = [
            genom_request(
                "mhp",
                "ArmPlanTask",
                    [0,
                    'GEN_TRUE',
                     'MHP_ARM_FREE',
                    0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    rq1, rq2, rq3, rq4, rq5, rq6, rq7,
                    'NO_NAME',
                    'NO_NAME',
                    'NO_NAME',
                    'GEN_FALSE',
                    0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
                    0, # Rotation type
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), # x y z rx ry rz
            
            genom_request("mhp", "ArmSelectTraj", [0]),
            genom_request("lwr",
                "SetMonitorForceParam",
                [1, 0.5, 0.1, 0.3, 0.005, 0.1]),
            genom_request("lwr",
                "TrackQ",
                ["LWR_ARM_RIGHT", "mhpArmTraj", "LWR_TRACK_POSTER"]),
            wait(7)
                ]
        else: #default with PR2SoftMotion
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
        
	    genom_request("mhp", "ArmSelectTraj", [0])]

        if robot.hasmodule("pr2SoftMotion"):
            actions.append(
                    genom_request("pr2SoftMotion", 
                        "TrackQ", 
                        ['mhpArmTraj', 'PR2SM_TRACK_POSTER', part],
                        wait_for_completion = False if callback else True,
                        callback = callback)
                    )

        elif robot.hasmodule("lwr"):
            actions += [
                            genom_request("lwr",
                                "SetMonitorForceParam",
                                [1, 0.5, 0.1, 0.3, 0.005, 0.1]),
                            genom_request("lwr",
                                "TrackQ",
                                ["LWR_ARM_RIGHT", "mhpArmTraj", "LWR_TRACK_POSTER"]),
                            wait(7)
                                ]

        else:
            logger.warning("No module for rm trajectory execution. Trajectory only " \
                        "published on the mhpArmTraj poster.")


    return actions

def setposeNAOqi(robot, posture, relative):

    defaultPostures = ["Crouch", "LyingBack", "LyingBelly", "Sit", "SitRelax", "Stand", "StandInit", "StandZero"]


    logger.debug("Setting pose " + str(posture))

    if posture in defaultPostures:
        actions = [
            naoqi_request("posture", "goToPosture", [posture, 0.8])
        ]
        return actions


    if not isinstance(posture, dict):
        try:
            posture = robot.postures[posture]
        except KeyError:
            raise RobotError("Posture %s not found!" % posture)

    names = []
    angles = []


    if "HEAD" in posture:
        names += ['HeadYaw','HeadPitch']
        angles += posture["HEAD"]
    if "RARM" in posture:
        names += ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        angles += posture["RARM"]
    if "LARM" in posture:
        names += ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        angles += posture["LARM"]
    if "TORSO" in posture:
        logger.warn("Torso bending is not yet supported for Nao")

    if relative:
        ok, res = robot.execute([
            naoqi_request("motion", "getAngles", [names, True])
            ])
        angles = [ a + da for a, da in zip(angles, res)]

    actions = [
        naoqi_request("motion", "angleInterpolationWithSpeed", [names, angles, 0.1], parts = names)
    ]
    return actions


@tested("04/10/2012")
@action
@same_requirements_as(setpose)
def manipose(robot, nohead = True, callback = None):
    """
    Quick method to set the PR2 joints in "manip" configuration.
    This posture is designed for the robot to easily move around with
    an object in its right hand.

    :param no_head: (default: true) If true, only arms and torso will set a new configuration.
    Very useful if you track an object.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """
        
    if robot.supports(NAOQI):
        actions = setpose(robot, 'StandInit', callback)
        actions.append(naoqi_request("motion", 
                                     "wbEnableEffectorControl", 
                                     ['LArm', True]))
        actions.append(naoqi_request("motion", 
                                     "wbEnableEffectorControl", 
                                     ['RArm', True]))
        return actions


    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'
    posture = robot.postures['MANIP']
    
    return setpose(robot, posture, callback, part)

@tested("04/10/2012")
@action
def extractpose(robot, callback = None):
    """
    Quick method to set the PR2 joints in "extraction" configuration.
    In "extractaction" configuration, the robot's right hand is next to
    the shoulder, far from the table.

    Useful to prevent collisions when moving near to the tables.

    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """
    
    return setpose(robot, "EXTRACTION", callback)

@tested("04/10/2012")
@action
@same_requirements_as(setpose)
def rest(robot, nohead = True, callback = None):
    """
    Quick method to set the robot joints in rest configuration. 
    You have the choice with three rest configuration. This choice is random.
    
    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """

    if robot.supports(NAOQI):
        actions = [naoqi_request("motion", 
                                 "wbEnableEffectorControl", 
                                 ['LArm', False])]
        actions.append(naoqi_request("motion", 
                                     "wbEnableEffectorControl", 
                                     ['RArm', False]))
        actions.append(naoqi_request("motion", "rest"))
        return actions


    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'

    n = random.randint(1,3)
    posture = robot.postures['REST' + str(n)]

    return setpose(robot, posture, callback, part)

@tested("04/10/2012")
@action
@same_requirements_as(setpose)
def restpose(robot, nohead = True, callback = None):
    return rest(robot, nohead, callback)

@tested("04/10/2012")
@action
def settorso(robot, height = 0.15, callback = None):
    """
    Set the PR2 torso height.

    :param height: The height of the torso, in meters. Defaults to 0.15
    cm.

    Height is clamped into [0, 30].

    :param callback: (optional) If given, the action is non-blocking,
    and the callback is invoked at the activity completion.

    """
    if height < 0:
        height = 0
    if height > 0.3:
        height = 0.3
    return setpose(robot, {'TORSO': [float(height)]}, callback)


@tested("04/10/2012")
@action
def tuckedpose(robot, callback = None, nohead = True):
    """
    Tucks the robot arms.

    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.
    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object
    """

    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'

    posture = robot.postures['TUCKED']

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
 
    actions = []
    if (c == 0):
        posture1 = robot.postures['TUCKED']
        posture2 = robot.postures['WATCH_LOOKING']
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
        posture1 = robot.postures['TUCKED']
        posture2 = robot.postures['WATCH_LOOKING']
        actions = setpose(robot, posture1,callback,part) + setpose(robot, posture2,callback,part) + setpose(robot, posture1,callback,part)
    elif(c == 9):
        posture1 = robot.postures['TUCKED']
        posture2 = robot.postures['WATCH_LOOKING']
        actions = setpose(robot, posture1,callback,part) + setpose(robot, posture2,callback,part) + setpose(robot, posture1,callback,part)

    return actions 


@action
@workswith(ROS)
@workswith(NAOQI)
def movearm(robot, target, arm = "right"):
    """ Moves one of the robot arms to a pose.
    
        :param target: the destination for the arm wrist
    """

    pose = robot.poses[target]
    
    client = None
    goal = None
    
    if robot.supports(ROS) and not robot.supports(NAOQI):
        if arm != "right":
            logger.error("Currently, only the right arm can be controlled"
                         " from ROS.")
            return []

        import rospy
        import actionlib
        import arm_navigation_msgs.msg
        client = actionlib.SimpleActionClient('move_right_arm', arm_navigation_msgs.msg.MoveArmAction)

        ok = client.wait_for_server()
        if not ok:
            logger.error("Could not connect to the ROS client for arm navigation!"
            " Aborting action")
            return []

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

        return [ros_request(client, 
                goal, 
                wait_for_completion = False,
                callback = None
            )]

    elif robot.supports(NAOQI):

        # Requires whole body cartesian control! ie, you must call nao.manipose()
        # first!
        effector = "LArm" if arm == "left" else "RArm"
        hand = "l_wrist" if arm == "left" else "r_wrist"

        dtarget = robot.poses.ros.inframe(pose, "base_footprint")

        actions = [
            naoqi_request("motion", "wbSetEffectorControl", [effector, 
                                                        [dtarget['x'],
                                                         dtarget['y'],
                                                         dtarget['z']]]),
        ]
        return actions

    else:
        # Useful for debugging purpose, without the actual robot
        client = "ROS arm_navigation"
        goal = pose
    

    return [ros_request(client, 
            goal, 
            wait_for_completion = False,
            callback = None
        )]


@action
@same_requirements_as(movearm)
def pointsat(robot, target):
    return movearm(robot, target)
