import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.exception import RobotError

from robots.action import *
from robots.helpers.jointstate import getjoint
from robots.helpers import position
from robots.helpers.cb import nop

from robots.actions.configuration import setpose


#TODO HACK: I create here a new action performer for background "tracking"
# actions to avoid pypoco multithreading issues
actionPerformerForTracking = None

###############################################################################
###############################################################################

@tested("22/02/2012")
@action
def look_at(robot, place, callback = None):
    """ A simple 'look at' method that uses pr2SoftMotion.

    Uses look_at_xyz underneath.

    :param place: a dictionary with the x,y,z position of objects in space#.
              If a 'frame' key is found, use it as reference frame. Else
              the world frame '/map' is assumed.
    """
    
    place = robot.poses.get(place)

    return look_at_xyz(robot, place['x'], place['y'], place['z'], place['frame'], callback)
    #return look_at_xyz_with_moveHead(place['x'], place['y'], place['z'], frame, callback)

###############################################################################

@tested("22/02/2012")
@action
def look_at_xyz(robot, x,y,z, frame = "map", callback = None):
    """ Look at via pr2SoftMotion.
    
    :param x: the x coordinate
    :param y: the y coordinate
    :param z: the z coordinate
    :param frame: the frame in which coordinates are interpreted. By default, '/map'
    """
    logger.info("Looking at " + str([x,y,z]) + " in " + frame)
    pantilt = robot.poses.ros.xyz2pantilt(x,y,z, frame)

    return setpose(robot, {'HEAD':pantilt}, callback)

###############################################################################



@tested("22/02/2012")
@action
def look_at_xyz_with_moveHead(robot, xyz, frame = "map", callback = None):
    """ Look at via pr2SoftMotion.
    
    :param x: the x coordinate
    :param y: the y coordinate
    :param z: the z coordinate
    :param frame: the frame in which coordinates are interpreted. By default, '/map'
    """
    logger.info("Looking at " + str(xyz) + " in " + frame)
    x,y,z = xyz
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
def track(robot, place):
    """ Tracks an object with the head.

    This uses pr2SoftMotion.

    This is a background action. Can be cancelled with cancel_track.

    :param place: a dictionary with the x,y,z position of objects in space.
              If a 'frame' key is found, use it as reference frame. Else
              the world frame '/map' is assumed.
    """
    target = place
    target.setdefault("frame", "/map")

    return [background_task(TrackAction, [target])]

###############################################################################

@action
def track_human(robot, part = "HeadX"):
    """ Tracks the human head.

    This uses pr2SoftMotion.

    This is a background action. Can be cancelled with cancel_track.

    """
    return [background_task(TrackAction, [part, True])] # Update = True

###############################################################################

from threading import Thread
import time

# TODO: move head only if robot/target has moved
class TrackAction(Thread):
    def __init__(self, robot, target, needupdate = False):
        global actionPerformerForTracking

        Thread.__init__(self)
        
        # At first track, initialize a new action performer
	#if not actionPerformerForTracking:
        #    from lowlevel import ActionPerformer
        #    actionPerformerForTracking = ActionPerformer('pr2c2', 9472, use_ros = False)

        self.running = False

        #TODO HACK: currently not used because of pypoco issues
        self.robot = robot
        
        self.targettoupdate = needupdate
        self.robotpart = target
        self.target = target

    def updatetarget(self):
        self.target = position.gethumanpose(self.robot, part = self.robotpart)
        #self.target = position.gethumanpose(actionPerformerForTracking, part = self.robotpart)

    def run(self):
	logger.info("Starting task " + self.__class__.__name__)
        self.running = True

        while self.running:
            if self.targettoupdate:
                self.updatetarget()
            if self.target:
                self.robot.execute(look_at, self.target)
                #actionPerformerForTracking.execute(look_at, self.target)
            time.sleep(0.5)
    
    def stop(self):
	logger.info("Stopping task " + self.__class__.__name__)
        self.running = False


###############################################################################

@action
def cancel_track(robot):
    """ If running, interrupt a current track action.
    """
    return [background_task(TrackAction, abort=True)]

###############################################################################

@broken
@tested("22/02/2012")
@action
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
def glance_to(robot, place): 
    """ Glance to via pr2SoftMotion
    """

    head_tilt = getjoint('head_tilt_joint')
    head_pan = getjoint('head_pan_joint')
    pose_head_base = {"HEAD": (head_pan,  head_tilt)}

    actions = look_at(robot, place)
    actions += [wait(2)]
    actions += setpose(robot, pose_head_base) 
    return actions

###############################################################################

@tested("22/02/2012")
@action
def sweep_look(robot, amplitude = 90, speed = 0.2):
    """ Makes a sweep movement with the robot head via pr2SoftMotion compared with its current position 
    
    :param amplitude: Number of degrees of the sweeping head movement
    """
    import math
    actions =[]
    
    amplitude_rd = math.radians(float(amplitude))
    
    head_tilt = getjoint('head_tilt_joint')
    head_pan = getjoint('head_pan_joint')

    pose_head_base = {"HEAD": (head_pan,  head_tilt)}
    pose_head1 = {"HEAD": (head_pan + amplitude_rd/2,  head_tilt)}
    pose_head2 = {"HEAD": (head_pan - amplitude_rd/2,  head_tilt)}

    actions =[
        genom_request("pr2SoftMotion", "SetTimeScale",[1.0, 1.0, speed, 1.0, 1.0, 1.0])
    ]

    actions += setpose(robot, pose_head1) 
    actions += setpose(robot, pose_head2) 
    actions +=[
        genom_request("pr2SoftMotion", "SetTimeScale",[1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    ]
    actions += setpose(robot, pose_head_base) 

    return actions

