import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

import os

from robots.exception import RobotError
from robots.helpers.cb import nop

from robots.action import *
from robots.action import wait as basewait

from robots.actions.manipulation import configure_grippers

@action
def setup_scenario(robot, scenario):
    """ Allows to load a custom scene configuration ('sce' files) in SPARK
    at runtime.
    
    :param scenario: The complete path to a valid scenario file ('.sce')
    """
    return [genom_request("spark", "LoadSce", scenario)]

@action
def lock_object(robot, object):
    """ Lock to position of an object in SPARK. It won't be updated
    anymore.
    
    :param object: The ID of the object to lock.
    """
    return [genom_request("spark", "SetObjectFixed", [object, "GEN_TRUE"])]

@action
def unlock_object(robot, object):
    """ Unlock to position of an object in SPARK. It will be updated
    when seen.
    
    :param object: The ID of the object to unlock.
    """
    return [genom_request("spark", "SetObjectFixed", [object, "GEN_FALSE"])]


@tested("22/02/2012")
@action
def wait(robot, seconds):
    """ This special action simply waits for a given amount of second before 
    sending the next action.
    
    :param seconds: the time to wait, in seconds
    """
    return [basewait(seconds)]


@tested("22/02/2012")
@action
def init(robot, \
        pr2SoftMotion = True, \
        spark = False,
        p3d = ""):
    """ Initialize modules in correct order.
    """

    if pr2SoftMotion:
        actions = configure_grippers(robot)
        actions += [genom_request("pr2SoftMotion", "Init")]
    if spark:
        if not os.path.exists(p3d):
            raise RobotError("P3D file not found!")
        actions += [genom_request("spark", "LoadP3d", [p3d, 1, 0]),
                    genom_request("spark", "UpdateInterface", callback = nop),
                    genom_request("spark", "SetInterfaceParams", [1200, 900, 0, 1, 1, 0, 0, 0]), #Scale up the SPARK window·
                    genom_request("spark", "SetInterfaceAgentParams", ["ACHILE_HUMAN1", 1, 1, 0]), # Display visibility and pointing cones.
                    genom_request("spark", "ChangeCameraPos", [4, -4, 1, 5, -0.7, 0.7]), #Move the camera
                    genom_request("spark", "ReadRobot", ["lwrCurrentPoseArmRight", "lwrSAHand" , "pomPos", "pomPlatineFramePos", "."], callback = nop), # LWR poster, sahand poster, POM poster, Platine poster, sparkyarp robot config
                    genom_request("spark", "ReadHumans", ['USE_NIUT', 0 , 'niutHuman'], callback = nop),
                    genom_request("spark", "ReadObjects", ['USE_VIMAN', 0 , 'vimanObjectPose'], callback = nop),
                    genom_request("spark", "SetKBAddress", ["localhost", 6969]),
                    genom_request("spark", "ComputeFacts", callback = nop)
                    ]

    return actions

