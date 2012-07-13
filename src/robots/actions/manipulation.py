import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

import random

from robots.action import *

from robots.helpers import postures
from robots.helpers.cb import *

from robots.actions import configuration, nav, look_at, speech

used_plan_id = []

@tested("22/02/2012")
@action
def release_gripper(robot, gripper = "RIGHT"):
    """
    Opens the gripper to release something.

    Like gripper_open, except it waits until it senses some effort on the gripper force sensors.

    :see: open_gripper

    :param gripper: "RIGHT" (default) or "LEFT"
    """
    
    if gripper == "RIGHT":
        return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["RRELEASE"])]
    else:
        return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["LRELEASE"])]


@tested("22/02/2012")
@action
def grab_gripper(robot, gripper = "RIGHT"):
    """
    Closes the gripper to grab something.

    Like gripper_close, except it waits until it senses some effort on the gripper force sensors.

    :see: close_gripper
    :param gripper: "RIGHT" (default) or "LEFT"
    """
    if gripper == "RIGHT":
        return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["RGRAB"])]
    else:
        return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["LGRAB"])]

@tested("22/02/2012")
@action
def open_gripper(robot, gripper = "RIGHT", callback = None):
    """
    Opens a gripper.

    If pr2SoftMotion-genom is available, it tries with it. Else it tries with fingers-genom.
    Parameter 'gripper' is ignored when using fingers-genom.

    :see: release_gripper
    :param gripper: "RIGHT" (default) or "LEFT"
    :param callback: if set, the action is non-blocking and the callback is invoked upon completion
    """

    if robot.hasmodule("pr2SoftMotion"):
        if gripper == "RIGHT":
            return [genom_request("pr2SoftMotion", 
                    "GripperGrabRelease", 
                    ["ROPEN"],
                    wait_for_completion = False if callback else True,
                    callback = callback)]

        else:
            return [genom_request("pr2SoftMotion", 
                    "GripperGrabRelease", 
                    ["LOPEN"],
                    wait_for_completion = False if callback else True,
                    callback = callback)]

    if robot.hasmodule("fingers"):
            return [genom_request("fingers", 
                    "OpenGrip", 
                    [],
                    wait_for_completion = False if callback else True,
                    callback = callback)]

    logger.warning("No module to open the gripper available. Skipping this action.")
    return []


@tested("22/02/2012")
@action
def close_gripper(robot, gripper = "RIGHT", callback = None):
    """ Closes the right gripper.
 
    If pr2SoftMotion-genom is available, it tries with it. Else it tries with fingers-genom.
    Parameter 'gripper' is ignored when using fingers-genom.

    :see: grab_gripper
    :param gripper: "RIGHT" (default) or "LEFT"
    :param callback: if set, the action is non-blocking and the callback is invoked upon completion
    """
    if robot.hasmodule("pr2SoftMotion"):
        if gripper == "RIGHT":
            return [genom_request("pr2SoftMotion", 
                    "GripperGrabRelease", 
                    ["RCLOSE"],
                    wait_for_completion = False if callback else True,
                    callback = callback)]
        else:
            return [genom_request("pr2SoftMotion", 
                    "GripperGrabRelease", 
                    ["LCLOSE"],
                    wait_for_completion = False if callback else True,
                    callback = callback)]

    if robot.hasmodule("fingers"):
            return [genom_request("fingers", 
                    "CloseGrip", 
                    [],
                    wait_for_completion = False if callback else True,
                    callback = callback)]

    logger.warning("No module to close the gripper available. Skipping this action.")
    return []


@tested("23/02/2012")
@action
def configure_grippers(robot, grab_acc = 8.0, grab_slip = 0.2, release_acc = 4.0, release_slip = 0.05, force = 25):
    """ Sets the grippers thresholds.
    
    :param grab_acc: threshold for grab detection
    :param grab_slip: threshold for grab detection
    :param release_acc: threshold for release detection
    :param release_slip: threshold for release detection
    :param force: hold force
    """
    return [genom_request("pr2SoftMotion", 
            "SetGripperTresholds", 
            [grab_acc, grab_slip, release_acc, release_slip, force])]


def getplanid():
    """ Returns a random plan id (for Amit planification routines) which is
    guaranteed to be 'fresh'.
    """
    plan_id = random.randint(1, 1000)
    while plan_id in used_plan_id:
        plan_id = random.randint(1, 1000)
    used_plan_id.append(plan_id)
    return plan_id

@action
def pick(robot, obj, use_cartesian = "GEN_FALSE"):
    """ Picks an object that is reachable by the robot.

    Uses MHP to plan a trajectory.
    The trajectory is executed with by pr2SoftMotion-genom if available, 
    else with lwr-genom if available, else oonly exported to the mhpArmTraj poster.

    :param object: the object to pick.
    """

    def haspickedsmthg(robot):

        try:
            gripper_joint = robot.state.getjoint('r_gripper_joint')

            if gripper_joint < 0.01:
                logger.warning("I think I've nothing in my right gripper...")
                return (False, "gripper joint < 0.01")
            else:
                return (True, None)
        except NameError: #no robot.state? consider the pick is successfull
            return (True, None)
        except AttributeError as detail: #no robot.state? consider the pick is successfull
            return (True, None)

    # Open gripper
    actions = open_gripper(robot)

    # Plan trajectory to object and execute it
    actions += [
    genom_request("mhp", "ArmPlanTask",
            [0,
            'GEN_TRUE',
            'MHP_ARM_PICK_GOTO',
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            obj,
            'NO_NAME',
            'NO_NAME',
            use_cartesian,
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        genom_request("mhp", "ArmSelectTraj", [0]),
    ]

    if robot.hasmodule("pr2SoftMotion"):
        actions.append(
                genom_request("pr2SoftMotion", 
                    "TrackQ", 
                    ['mhpArmTraj', 'PR2SM_TRACK_POSTER', 'RARM']))

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

    # Close gripper
    actions += close_gripper(robot)

    actions += [python_request(haspickedsmthg)]


    return actions


@tested("")
@action
def attachobject(robot, obj, attach = True):
    """ attach or detach the object to the robot hand
    
    This function should be called after a release or a grab
    :param obj: the object to attach/dettach.
    :param attach: true (default) to attach an object, false to detach it
    """
    i = 0
    if (attach) :
      i = 1
    actions = [
        genom_request("spark","SetGraspedObject", [obj, i, 0]),
        genom_request("spark","SetInferrenceForObject", [obj, i, robot.id, 0,
            "SPARK_PRECISE_ROBOT_HAND", 1.0])
    ]

    return actions

@tested("05/07/2012")
@action
def basictake(robot):
    """ The ultra stupid basic TAKE: simply hand the object in front of the
    robot.
    """

    posture = postures.read()
    
    actions = configuration.setpose(robot, posture["GIVE"])
    actions += open_gripper(robot)
    actions += [wait(2)]
    actions += grab_gripper(robot)
    actions += configuration.manipose(robot)
        
    return actions

@action
def take(robot, human, obj):

    actions = look_at.look_at(robot, human)
    actions += configuration.manipose(robot, nop)
    actions += look_at.look_at(robot, [1,0,0.7,"base_link"])

    mobility = 0.0

    res = robot.planning.handover(human, mobility = mobility)

    if not res:
        logger.warning("OTP planning failed. Retrying.")
        robot.sorry()
        res = robot.planning.handover(human, mobility=mobility)
        if not res:
            logger.error("OTP planning failed again. Giving up.")
            return []

    wps, pose = res

    torso = pose["TORSO"]
    del pose["TORSO"]
    actions += configuration.settorso(robot, torso, nop)

    actions += nav.waypoints(robot, wps)
    actions += look_at.look_at(robot, human,nop)

    actions += configuration.setpose(robot, pose)

    actions += speech.say(robot, "Ok, I take it")
    actions += open_gripper(robot)
    actions += grab_gripper(robot)

    return actions

@tested("22/02/2012")
@action
def basicgive(robot):
    """ The ultra stupid basic GIVE: simply hand the object in front of the
    robot.
    
    After handing the object, the robot waits for someone to take it, and
    stay in this posture. 
    """

    posture = postures.read()
    
    actions = configuration.setpose(robot, posture["GIVE"])
    actions += release_gripper(robot)
    actions += [wait(2)]
    actions += close_gripper(robot, callback = nop)
        
    return actions

@tested("22/02/2012")
@action
def basicgrab(robot):
    """ The ultra stupid basic GRAB: simply take the object in front of the
    robot.
    
    After handing its gripper, the robot waits for someone to put an object in
    it, and stay in this posture.
    """

    posture = postures.read()
    
    actions = open_gripper(nop)
    actions += configuration.setpose(robot, posture["GIVE"])
    actions += grab_gripper(robot)
        
    return actions

#def chained_handover_feedback_cb(previous_cb = None):
#    def cb(progress):
#        if progress >


@tested("15/06/2012")
@action
def handover(robot, human, mobility = 0.0, feedback = None):
    """ Computes and executes a move for a 'hand-over': given a
    human, the robot finds a trajectory and a pose to go and hand
    the object in its right hand to the human.

    The efforts can be shared between the human and the robot with
    the 'mobility' parameter.

    Note that the object is assumed to be already in the hand.

    :param human: the human (SPARK ID) to hand an object over.
    :param mobility: the level of mobility of the human. 0.0 means
    the human can not move (the robot does all the displacement),
    1.0 means the robot and the human will each roughly move half 
    the way.
    :param feedback: (optional) a callback that is invoked as the
    robot moves along. It provides the percentage of the trajectory
    already covered, the distance to go and the distance already 
    covered.
    """

    actions = look_at.look_at(robot, human)
    actions += configuration.manipose(robot, nop)
    actions += look_at.look_at(robot, [1,0,0.7,"base_link"])
    res = robot.planning.handover(human, mobility = mobility)

    if not res:
        logger.warning("OTP planning failed. Retrying.")
        robot.sorry()
        res = robot.planning.handover(human, mobility=mobility)
        if not res:
            logger.error("OTP planning failed again. Giving up.")
            return []

    wps, pose = res
    logger.debug("the torso should be at height : " + str(pose["TORSO"]))
    torso = pose["TORSO"]
    del pose["TORSO"]
    logger.debug("the torso should be at height : " + str(torso[0]))
    actions += configuration.settorso(robot, torso[0])

    actions += nav.waypoints(robot, wps, feedback = feedback)
    actions += look_at.look_at(robot, human,nop)

    # Collision avoidance
    #pose_rarm = {'RARM':pose['RARM']}
    #actions += configuration.settorso(pose['TORSO'][0], nop)
    #actions += configuration.setpose(robot, pose_rarm, collision_avoidance = True, callback=nop)

    # No collision avoidance
    actions += configuration.setpose(robot, pose)

    actions += speech.say(robot, "Here your object")
    actions += release_gripper(robot)
    actions += [wait(1)]
    actions += close_gripper(robot, callback=nop)

    return actions


@action
def amit_give(robot, performer, obj, receiver):
    """ The 'Amit' GIVE.
    """
    plan_id = getplanid()
    actions = [
        genom_request("mhp",
            "Plan_HRI_Task",
            [plan_id, "GIVE_OBJECT", obj, performer,  receiver, 0, 0, 0]
        ),
        genom_request(	"mhp",
            "Get_SM_Traj_HRI_Task",
            [plan_id]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["OPEN"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [0]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [1]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["CLOSE"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [3]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [4]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["RELEASE"]
        )
    ]

    return actions

