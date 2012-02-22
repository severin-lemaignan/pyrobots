import random

from robots.action import action, genom_request, wait, broken

from robots.helpers import postures
from robots.helpers.cb import *

from robots.actions import configuration

used_plan_id = []

@action
def release_gripper(robot):
    """
    Opens the gripper to release something.

    Like gripper_open, except it waits until it senses some effort on the gripper force sensors.

    :see: open_gripper
    """
    return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["RELEASE"])]

@action
def grab_gripper(robot):
    """
    Closes the gripper to grab something.

    Like gripper_close, except it waits until it senses some effort on the gripper force sensors.

    :see: close_gripper
    """
    return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["GRAB"])]

@action
def open_gripper(robot, callback = None):
    """
    Opens the right gripper.

    :see: release_gripper
    """
    return [genom_request("pr2SoftMotion", 
            "GripperGrabRelease", 
            ["OPEN"],
            wait_for_completion = False if callback else True,
            callback = callback)]

@action
def close_gripper(robot, callback = None):
    """ Closes the right gripper.
    
    :see: grab_gripper
    """
    return [genom_request("pr2SoftMotion", 
            "GripperGrabRelease", 
            ["CLOSE"],
            wait_for_completion = False if callback else True,
            callback = callback)]


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

    :param object: the object to pick.
    """

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
        genom_request("pr2SoftMotion", "TrackQ", ['mhpArmTraj', 'PR2SM_TRACK_POSTER', 'RARM'])
    ]

    # Close gripper
    actions += close_gripper(robot)
    return actions

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
    actions += close_gripper(nop)
        
    return actions

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

