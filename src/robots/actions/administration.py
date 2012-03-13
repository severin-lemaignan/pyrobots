# coding=utf-8
import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

import os

from robots.exception import RobotError
from robots.helpers.cb import nop

from robots.action import *
from robots.action import wait as basewait

from robots.actions.manipulation import configure_grippers
from robots.actions.configuration import settorso

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

@tested("13/03/2012")
@action
def display(robot, window, host = "localhost"):
    """ Allows to select which software window is displayed by the robot.

    Requires:
     - laaswm as windows manager
     - laaswm.py installed (available from laaswm-libs repository)

    :param window: name (or part of the name) of the window to put
    on front. Case insensitive.
    :param host: the LAASWM host (default: localhost)
    """
    import laaswm
    wm = laaswm.WM(host)

    windows = wm.list()
    for id, params in windows.items():
        if window.lower() in params["name"].lower():
            wm.fullscreen(id)
            return


@tested("13/03/2012")
@action
def init(robot, \
        viman = True, \
        niut = True, \
        pr2SoftMotion = True, \
        spark = True,
        p3d = ""):
    """ Initialize modules in correct order.

        Modules whose initialization is reentrant (ie, init code can be safely called
        twice) are enabled by default.
    """

    actions = []

    if viman:
        actions += [genom_request("viman", "Init", ["VIMAN_INPUT_VIAM", # Input stereo stream
                                                   "VIMAN_ENABLE", # use stereo
                                                   "VIMAN_DISABLE", # use color
                                                   os.environ["ROBOTPKG_BASE"] + "/share/viman", # object config path
                                                   "vimanBridgeImage", # input poster
                                                   "CAMERA_TOP_LEFT", # left camera name
                                                   "CAMERA_TOP_RIGHT", # right camera name
                                                   1, # stereo ratio baseline
                                                   "VIMAN_ENABLE", # display status
                                                   0]), # unused
                    genom_request("viman", "ActivateTracking", callback = nop)]

    if niut:
        actions += [genom_request("niut", "Init")]

    if pr2SoftMotion:
        actions += configure_grippers(robot)
        actions += [genom_request("pr2SoftMotion", "Init")]
        actions += settorso(robot, 10)

    # SPARK initialization
    if spark:
        if not os.path.exists(p3d):
            logger.error("P3D file not found! Skipping the loading of the environment")
        else:
            actions += [genom_request("spark", "LoadP3d", [p3d, 1, 0])]

        actions += [genom_request("spark", "UpdateInterface", callback = nop),
                    genom_request("spark", "SetInterfaceParams", [1200, 900, #Scale up the SPARK window·
                                                                  "GEN_FALSE", # Save interface
                                                                  "GEN_TRUE", # Enable floors
                                                                  "GEN_FALSE", # Enable walls
                                                                  "GEN_FALSE", # Enable shadows
                                                                  "GEN_FALSE", # Enables tiles
                                                                  "GEN_FALSE"]), # Antialiasing
                    genom_request("spark", "SetInterfaceAgentParams", ["HERAKLES_HUMAN1", 1, 1, 0]), # Display visibility and pointing cones.
                    genom_request("spark", "ChangeCameraPos", [6, -2, 1, 5, -1, 0.5]), #Move the camera
                    genom_request("spark", "SetKinectFixedFrame", ["GEN_TRUE", 3.0, 0.0, 2.51, 0, 0.45, -0.78]), #Place the fixed Kinect above Rachid's door
                    genom_request("spark", "ReadRobot", ["ignored", "ignored" , "ignored", "ignored", "ignored", "pr2Pose", "pr2JointState", "pr2JointMap"], callback = nop), # LWR poster, sahand poster, POM poster, Platine poster, sparkyarp robot config
                    genom_request("spark", "ReadHumans", ['USE_NIUT', 0 , 'niutHuman'], callback = nop),
                    genom_request("spark", "ReadObjects", ['USE_VIMAN', 0 , 'vimanObjectPose'], callback = nop)]

        if robot.knowledge: # The robot has been created with an knowledge base, let's use it
            logger.info("I'm connected to a knowledge base. Let's start filling it from perception!")
            actions += [genom_request("spark", "SetKBAddress", [robot.knowledge.host, robot.knowledge.port]),
                        genom_request("spark", "ComputeFacts", callback = nop)
                        ]

    return actions

