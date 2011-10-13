import logging; logger = logging.getLogger("novela." + __name__)
logger.setLevel(logging.DEBUG)

from action import action, genom_request, ros_request, background_task, wait
from helpers.jointstate import getjoint
from helpers import position
from helpers.cb import nop


import rospy
import numpy
import math
from geometry_msgs.msg import PointStamped
from tf import TransformerROS
from tf import TransformListener
from tf import transformations

listener = TransformListener()

#TODO HACK: I create here a new action performer for background "tracking"
# actions to avoid pypoco multithreading issues
actionPerformerForTracking = None

#############################################################################
############################################################################
def xyz_to_panTilt(frame, x, y, z):
    """ Change moveHead parameters from xyz tp pan and tilt
    :param x: the x coordinate
    :param y: the y coordinate
    :param z: the z coordinate
    :param frame: the frame in which coordinates are interpreted

    """

    goal = PointStamped()
    goal.header.frame_id =frame
    goal.header.stamp = rospy.Time(0);
    goal.point.x = x
    goal.point.y = y
    goal.point.z = z
    frame = 'map'

    goalInFrame = PointStamped()
    listener.waitForTransform(frame, 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    goalInFrame = listener.transformPoint('base_footprint',goal)
    
    x = goalInFrame.point.x
    y = goalInFrame.point.y
    z = goalInFrame.point.z

    #listener = TransformListener()
    # +0.067 to take into account the translation between base_footprint and head_pan_link
    pan= numpy.arctan2(y, x+0.067)
    listener.waitForTransform(frame, 'head_pan_link', rospy.Time(0), rospy.Duration(4.0))
    (transTilt,rotTilt) = listener.lookupTransform('head_pan_link', frame, rospy.Time(0))
    matBaseTilt= numpy.dot(transformations.translation_matrix(transTilt), transformations.quaternion_matrix(rotTilt))
    xyzTilt = tuple(numpy.dot(matBaseTilt, numpy.array([x, y, z, 1.0])))[:3]
    tilt= 5*numpy.arctan2(-xyzTilt[2], numpy.sqrt(math.pow(xyzTilt[0],2)+math.pow(xyzTilt[1],2)))
    #TODO: le 5* dans le tilt correspond a un hack pourri : le tilt ne correspond pas exactement au Z le coeficient est la pour palier a cela.

    #print numpy.sqrt(math.pow(xyzTilt[0],2)+math.pow(xyzTilt[1],2))
    #print -xyzTilt[2]
    #print "###################"
    #print "pan and tilt"
    #print pan
    #print tilt


    return (pan,tilt)


###############################################################################
###############################################################################

@action
def look_at(place, callback = None):
    """ A simple 'look at' method that uses pr2SoftMotion.

    Uses look_at_xyz underneath.

    :param place: a dictionary with the x,y,z position of objects in space#.
              If a 'frame' key is found, use it as reference frame. Else
              the world frame '/map' is assumed.
    """
    try:
        frame = place['frame']
    except KeyError:
        frame = "map"

    return look_at_xyz(place['x'], place['y'], place['z'], frame, callback)
    #return look_at_xyz_with_moveHead(place['x'], place['y'], place['z'], frame, callback)

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
    (pan,tilt) = xyz_to_panTilt(frame,x,y,z)
    actions = [
        genom_request(  "pr2SoftMotion",
            "GotoQ",
            ["HEAD",
            0,
            0.0, pan, tilt, 0.0,
            0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 
            0, 0, 0, 0, 0, 0, 0, 0.0, 0.0],
        wait_for_completion = False if callback else True,
        callback = callback
        )
    ]

    return actions

###############################################################################



@action
def look_at_xyz_with_moveHead(x,y,z, frame = "map", callback = None):
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
    (pan,tilt) = xyz_to_panTilt(frame,x,y,z)
    return actions

###############################################################################

@action
def track(place):
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
def track_human(part = "HeadX"):
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
def cancel_track():
    """ If running, interrupt a current track action.
    """
    return [background_task(TrackAction, abort=True)]

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

    actions = look_at_xyz(place['x'], place['y'], place['z'], frame)

    actions += [
                wait(2),
                genom_request("pr2SoftMotion", "GotoQ", 
                        ["HEAD", 0, 0.0, head_pan,  head_tilt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        )
                ]
    return actions

###############################################################################

@action
def sweep_look(amplitude = 90, speed = 0.2):
    """ Makes a sweep movement with the robot head via pr2SoftMotion compared with its current position 
    
    :param amplitude: Number of degrees of the sweeping head movement
    """
    import math
    actions =[]
    
    amplitude_rd = math.radians(float(amplitude))
    
    head_tilt = getjoint('head_tilt_joint')
    head_pan = getjoint('head_pan_joint')

    actions +=[
        genom_request("pr2SoftMotion", "SetTimeScale",[1.0, 1.0, speed, 1.0, 1.0]),
        genom_request("pr2SoftMotion", "GotoQ",
            ["HEAD", 0, 0.0, head_pan + amplitude_rd/2,  head_tilt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ),
        genom_request("pr2SoftMotion", "GotoQ",
            ["HEAD", 0, 0.0, head_pan - amplitude_rd/2,  head_tilt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ),
        genom_request("pr2SoftMotion", "SetTimeScale",[1.0, 1.0, 1.0, 1.0, 1.0]),
        genom_request("pr2SoftMotion", "GotoQ", 
            ["HEAD", 0, 0.0, head_pan,  head_tilt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        ]

    return actions

