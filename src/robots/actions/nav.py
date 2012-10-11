import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.helpers.cb import *

from robots.action import *

import math

###############################################################################
###############################################################################

@tested("04/10/2012")
@action
def goto(robot, target, callback = None, feedback = None):
    """ Moves the robot base to a given target, using ROS 2D navigation stack.

        Only (x,y,theta), ie (x, y, qw, qz), are considered for the target. 
        All other values are ignored.

        :param target: the destination, as a valid pyRobots position.
        :param callback: (optional) a callback to be called when the destination
        is reached. If nothing is provided, the action blocks until the 
        destination is reached.
        :param feedback: (optional) a callback that is called at regular intervals
        with an update of the current robot position
    """

    pose = robot.poses[target]
    
    client = None
    goal = None
    
    if robot.hasROS():
        import rospy
        import actionlib
        import move_base_msgs.msg
        # Creates the SimpleActionClient, passing the type of the action
        # (Navigationction) to the constructor.
        client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

        ok = client.wait_for_server()
        if not ok:
            #logger.error("Could not connect to the ROS client! Aborting action")
            print("Could not connect to the ROS client! Aborting action")
            return

        # Creates a goal to send to the action server.  
        goal = move_base_msgs.msg.MoveBaseGoal()

        # Definition of the goal
        goal.target_pose.header.frame_id = pose['frame']
        goal.target_pose.header.stamp = rospy.Time.now();

        goal.target_pose.pose.position.x = pose['x']
        goal.target_pose.pose.position.y = pose['y']
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = pose['qz']
        goal.target_pose.pose.orientation.w = pose['qw']
        
    else:
        # Useful for debugging purpose, without the actual robot
        client = "ROS move_base"
        goal = pose
    

    return [ros_request(client, 
            goal, 
            wait_for_completion = False if callback else True,
            callback = callback,
            feedback=feedback
        )] # Return a non-blocking action. Useful to be able to cancel it later!

###############################################################################
@action
def moveclose(robot, target, distance = 1, callback = None, feedback = None):
    """ Moves towards a target, stopping close to it.

        Underneath, it uses the ROS 2D navigation stack.

        Only (x,y,theta), ie (x, y, qw, qz), are considered for the target.
        All other values are ignored.

        :param target: the destination, as a valid pyRobots position.
        :param distance: the distance from the target, in meter, where to stop.
        :param callback: (optional) a callback to be called when the destination
        is reached. If nothing is provided, the action blocks until the 
        destination is reached.
        :param feedback: (optional) a callback that is called at regular intervals
        with an update of the current robot position

    """
    myself = robot.poses.myself()

    target = robot.poses[target]
    import copy
    orig_target = copy.deepcopy(target)

    target_distance = distance

    dir_x = target["x"] - myself["x"]
    dir_y = target["y"] - myself["y"]
    distance = math.sqrt(math.pow(dir_x, 2) + \
                            math.pow(dir_y, 2))


    if distance < target_distance:
        return []
    else:
        target["x"] = myself["x"] + dir_x * (1 - target_distance/distance)
        target["y"] = myself["y"] + dir_y * (1 - target_distance/distance)

        # Rotate to face the target
        heading = math.atan2(orig_target["y"] - target["y"], orig_target["x"] - target["x"])
        target["qz"] = math.sin(heading/2)
        target["qw"] = math.cos(heading/2)

        return goto(robot, target, callback, feedback)

@action
def carry(robot, target, callback = None):
    """ Moves to a place, taking into account door crossing.

    If the robot needs to cross a door, it will stop in front of the door,
    tuck its arms, cross the door, untuck the arms and complete its route.

    TODO: This action is currently completely hardcoded, and will work only
    for the Novela scenario.

    :param target: the destination, as a dictionary {x,y,z,qx,qy,qz,qw}.
    :param callback: (optional) a callback to be called when the destination
    is reached. If nothing is provided, the action blocks until the 
    destination is reached.
    """

    from helpers import position
    from helpers import places
    from actions import configuration

    mypos = position.mypose()

    target_is_in = position.isonstage(target)
    i_am_in = position.isonstage(mypos)
    if not (target_is_in ^ i_am_in):
        # No door to cross
        return goto(target, callback)
    else:
        # We need to cross a door!
        actions = []

        logger.info("I need to cross a door!!")
        print("I need to cross a door!!")
        
        if i_am_in:
            print("I'm in, I want to go out")
            # I'm in, I want to go out
            actions += configuration.tuckedpose(nop)
            actions += goto(places.read()["JARDIN_EXIT_IN"], callback)
            actions += goto(places.read()["HQ"], callback)
            actions += configuration.manipose(nop)
            actions += goto(target, callback)

        else:
            print("I'm out, I want to go in")
            # I'm out, I want to go in
            actions += configuration.tuckedpose(nop)
            actions += goto(places.read()["JARDIN_ENTER_OUT"], callback)
            actions += goto(places.read()["JARDIN_ENTER_IN"], callback)
            actions += configuration.manipose(nop)
            actions += goto(target, callback)

        return actions

################################################################################

@tested("04/10/2012")
@action
def waypoints(robot, points, callback = None, feedback = None):
    """ Moves the robot base along a set of waypoints, using ROS 2D navigation 
    stack (and the LAAS 'waypoints' ROS node).

        :param waypoints: a set of locations (any valid pyRobot pose).
        :param callback: (optional) a callback to be called when the final destination
        is reached. If nothing is provided, the action blocks until the 
        destination is reached.
        :param feedback: (optional) a callback to be called each time the feedback topic of the waypoint is updated (ie, every few percent of trajectory accomplishment).
    """

    # Normalize the poses
    wps = [robot.poses[pose] for pose in points]
    
    client = None
    goal = None
    
    if robot.hasROS():
        import rospy
        import actionlib
        import waypoints.msg
        # Creates the SimpleActionClient, passing the type of the action
        # (Navigationction) to the constructor.
        client = actionlib.SimpleActionClient('waypoints', waypoints.msg.waypointsAction)

        ok = client.wait_for_server()
        if not ok:
            #logger.error("Could not connect to the ROS client! Aborting action")
            print("Could not connect to the ROS client! Aborting action")
            return

        # Creates a goal to send to the action server.  
        goal = waypoints.msg.waypointsGoal()

        for pose in wps:
            goal.waypointTab.append(robot.poses.ros.asROSpose(pose))

    else:
        # Useful for debugging purpose, without the actual robot
        client = "ROS waypoints"
        goal = wps
    

    return [ros_request(client, 
            goal, 
            wait_for_completion = False if callback else True,
            callback = callback,
            feedback = feedback
        )] # Return a non-blocking action. Useful to be able to cancel it later!

###############################################################################


@action
def follow(robot, target):
    """ Follows a target until cancelled.

    Every 3 seconds, it re-evaluate the pose of the target, and move to it.

    This is a background action. Can be cancelled with cancel_follow.

    :param target: a pyRobots pose to follow. At each step, the pose is re-evaluated is needed (eg for a TF frame or a SPARK object)    
    """

    return [background_task(FollowAction, [target])]


###############################################################################

from threading import Thread
import time

class FollowAction(Thread):
    def __init__(self, robot, target):

        Thread.__init__(self)

        self.running = False
        self.robot = robot
        self.target = target

    def run(self):
        self.running = True

        while self.running:
            if self.target:
                self.robot.moveclose(self.target, 1.5, nop) # move, not closer that 1.5 meters
            time.sleep(3)
    
    def stop(self):
        self.running = False


###############################################################################

@action
def cancel_follow(robot):
    """ If running, interrupt a current follow action.
    """
    return [background_task(FollowAction, abort=True)]


