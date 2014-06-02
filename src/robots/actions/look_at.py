import logging; logger = logging.getLogger("robot." + __name__)

import os

from robots.lowlevel import *
from robots.exception import RobotError, UnknownFrameError

from robots.action import *
from robots.helpers import position
from robots.helpers.cb import nop

from robots.actions.configuration import setpose


#TODO HACK: I create here a new action performer for background "tracking"
# actions to avoid pypoco multithreading issues
actionPerformerForTracking = None

###############################################################################
###############################################################################

def clip(v, vmin, vmax):
    return max(min(v, vmax), vmin)


@tested("21/11/2012")
@workswith({POCOLIBS:"pr2SoftMotion"})
@workswith({POCOLIBS:"platine"})
@workswith(NAOQI)
@action
def lookat(robot, place, callback = None):
    """ Orders the robot to look at a given place of object.

    If available, uses pr2SoftMotion::moveHead. Else tries with the platine-genom
    module.

    :param place: any valid pyrobots place (spark id, ROS frame, [x,y,z],...)
    """

    if not place:
        return []

    try:
        place = robot.poses[place]
    except UnknownFrameError:
        logger.warning("Could not resolve the pose %s. Skipping look_at." %place)
        return []

    if robot.hasmodule("pr2SoftMotion"):
        return look_at_xyz_with_moveHead(robot, place['x'], place['y'], place['z'], place['frame'], callback)

    elif robot.hasmodule("platine"):

        raw = robot.execute([genom_request("spark", "GetJointAbsPose", [place, "HeadX"])])
        
        ok, res = raw
        if not ok:
            # Object probably does not exist
            return []
        yaw, pitch, roll, x, y, z = [float(x) for x in res]

        actions = [
            genom_request("platine",
                "AimAtTargetPoint",
                [0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0,
                "POM_FRAME_ORIGIN",
                0,
                x,
                y,
                z
                ],
                wait_for_completion = False if callback else True,
                callback = callback)
            ]
        return actions

    elif robot.supports(NAOQI):
        yaw, pitch = robot.poses.ros.xyz2pantilt(robot.poses[place], 
                                                 headframe="HeadPitch_link")
        yaw = clip(yaw, -2.0857, 2.0857)
        pitch = clip(-pitch, -0.6720, 0.5149) #beware the '-pitch'!
        return setpose(robot, {"HEAD": (yaw, pitch)}, relative = True)
    else:
        logger.warning("No module available to execute a 'look_at'. Skipping this action.")
        return []

@action
@same_requirements_as(lookat)
def look_at(robot, place, callback = None):
    return lookat(robot, place, callback)
 
###############################################################################


@tested("22/02/2012")
@action
def look_at_xyz_with_moveHead(robot, x,y,z, frame = "map", callback = None):
    """ Look at via pr2SoftMotion.
    
    :param x: the x coordinate
    :param y: the y coordinate
    :param z: the z coordinate
    :param frame: the frame in which coordinates are interpreted. By default, '/map'
    """
    logger.debug("Looking at " + str([x,y,z]) + " in " + frame)
    actions = [
        genom_request("pr2SoftMotion",
            "MoveHead",
            [x,y,z,frame],
        wait_for_completion = False if callback else True,
        callback = callback
        )
    ]
    return actions

###############################################################################

@tested("15/06/2012")
@action
@same_requirements_as(look_at)
def track(robot, target):
    """ Tracks an object with the head.

    This uses pr2SoftMotion.

    This is a background action. Can be cancelled with cancel_track.

    :param target: a pyRobots pose to track. At each step, the pose is re-evaluated is needed (eg for a TF frame or a SPARK object)    """

    return [background_task(TrackAction, [target])]


###############################################################################

from threading import Thread
import time

# TODO: move head only if robot/target has moved
class TrackAction(Thread):
    def __init__(self, robot, target):
        global actionPerformerForTracking

        Thread.__init__(self)
        
        # At first track, initialize a new action performer
	#if not actionPerformerForTracking:
        #    from lowlevel import ActionPerformer
        #    actionPerformerForTracking = ActionPerformer('pr2c2', 9472, use_ros = False)

        self.running = False
        self.robot = robot
        self.target = target

    def run(self):
        self.running = True

        while self.running:
            if self.target:
                self.robot.look_at(self.target)
                #actionPerformerForTracking.execute(look_at, self.target)
            time.sleep(0.5)
    
    def stop(self):
        self.running = False


###############################################################################

@tested("15/06/2012")
@action
@same_requirements_as(look_at)
def cancel_track(robot):
    """ If running, interrupt a current track action.
    """
    return [background_task(TrackAction, abort=True)]

###############################################################################

@broken
@tested("22/02/2012")
@action
@workswith({ROS:["/head_traj_controller/point_head_action"]})
def look_at_ros(robot, place):
    """ Create the client and the goal.

    :param place: a dictionary which contains object position parameters. 
    Cf look_at for details.
    """

    #import roslib; roslib.load_manifest('p_actionlib')
    import rospy

    import actionlib
    import pr2_controllers_msgs.msg
    import geometry_msgs.msg


    place = robot.poses[place]
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
            logger.error("Could not connect to the ROS client! Aborting action")
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

@tested("22/02/2012")
@action
@same_requirements_as(look_at)
def glance_to(robot, place): 
    """ Briefly look at a given location, and come back to the
    currently pose.
    """

    pose_head_base = {"HEAD": robot.state.getpose()["HEAD"]}

    actions = look_at(robot, place)
    actions += [wait(2)]
    actions += setpose(robot, pose_head_base) 
    return actions

###############################################################################

@tested("22/02/2012")
@action
@same_requirements_as(look_at)
def sweep(robot, amplitude = 90, speed = 0.2):
    """ Makes a sweep movement with the robot head via pr2SoftMotion compared with its current position 
    
    :param amplitude: Number of degrees of the sweeping head movement (default:
    90 deg)
    :param speed: Speed of the movement (default: 0.2)
    """
    import math
    actions=[]
    
    amplitude_rd = math.radians(float(amplitude))

    head_pan, head_tilt = robot.state.getpose()["HEAD"]

    pose_head_base = {"HEAD": (head_pan,  head_tilt)}
    pose_head1 = {"HEAD": (head_pan + amplitude_rd/2,  head_tilt)}
    pose_head2 = {"HEAD": (head_pan - amplitude_rd/2,  head_tilt)}

    actions = []
    if robot.hasmodule("pr2SoftMotion"):
        actions += [genom_request("pr2SoftMotion", "SetTimeScale",[1.0, 1.0, speed, 1.0, 1.0, 1.0])]

    actions += setpose(robot, pose_head1) 
    actions += setpose(robot, pose_head2) 

    if robot.hasmodule("pr2SoftMotion"):
        actions +=[genom_request("pr2SoftMotion", "SetTimeScale",[1.0, 1.0, 1.0, 1.0, 1.0, 1.0])]

    actions += setpose(robot, pose_head_base) 

    return actions


###############################################################################
@tested("21/11/2012")
@action
@workswith(ROS)
def switch_active_stereo_pair(robot, pair = "wide_stereo"):
    os.system("rosservice call /viman_bridge/switch_cameras %s" % pair)

