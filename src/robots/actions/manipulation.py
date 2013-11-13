import logging; logger = logging.getLogger("robot." + __name__)

import random
import math

from robots.lowlevel import *
from robots.action import *
from robots.exception import UnknownFrameError

from robots.helpers import postures
from robots.helpers.cb import *

from robots.actions import configuration, nav, look_at, speech


@tested("04/10/2012")
@action
@workswith({POCOLIBS:['pr2SoftMotion']})
@workswith(NAOQI)
def release_gripper(robot, gripper = "right"):
    """
    Opens the gripper to release something.

    Like gripper_open, except it waits until it senses some effort on the gripper force sensors.

    :see: open_gripper

    :param gripper: "right" (default) or "left"
    """

    gripper = gripper.lower()

    if robot.hasmodule("pr2SoftMotion"):
        if gripper == "right":
            return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["RRELEASE"])]
        else:
            return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["LRELEASE"])]
    if robot.hasmodule("lwr"):
            return [genom_request("fingers", "OpenGrip", [])]

    if robot.supports(NAOQI):
        joint = "LHand" if gripper == "left" else "RHand"
        actions = [
            wait(1),
            naoqi_request("motion", "openHand", [joint]),
            wait(1),
            naoqi_request("motion", "angleInterpolationWithSpeed", [joint, 0.3, 0.5])
         ]
        return actions

    logger.warning("No module to open the gripper available. Skipping this action.")
    return []




@tested("04/10/2012")
@action
@workswith({POCOLIBS:['pr2SoftMotion']})
@workswith(NAOQI)
def grab_gripper(robot, gripper = "right"):
    """
    Closes the gripper to grab something.

    Like gripper_close, except it waits until it senses some effort on the
    gripper force sensors, and tries to appliy a force suitable to hold something.

    :see: close_gripper
    :param gripper: "right" (default) or "left"
    """
    gripper = gripper.lower()

    if robot.hasmodule("pr2SoftMotion"):
        if gripper == "right":
            return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["RGRAB"])]
        else:
            return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["LGRAB"])]

    if robot.supports(NAOQI):
        joint = "LHand" if gripper == "left" else "RHand"
        actions = [
            naoqi_request("motion", "openHand", [joint]),
            wait(1),
            naoqi_request("motion", "stiffnessInterpolation", [joint, 1.0, 1.0]),
            naoqi_request("motion", "angleInterpolationWithSpeed", [joint, 0.0, 0.5])
        ]
        return actions

    logger.warning("No module to close the gripper available. Skipping this action.")
    return []


@tested("04/10/2012")
@action
@workswith({POCOLIBS:['pr2SoftMotion']})
@workswith(NAOQI)
def open_gripper(robot, gripper = "right", callback = None):
    """
    Opens a gripper.

    If pr2SoftMotion-genom is available, it tries with it. Else it tries with fingers-genom.
    Parameter 'gripper' is ignored when using fingers-genom.

    :see: release_gripper
    :param gripper: "right" (default) or "left"
    :param callback: if set, the action is non-blocking and the callback is invoked upon completion
    """

    gripper = gripper.lower()

    if robot.hasmodule("pr2SoftMotion"):
        if gripper == "right":
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

    if robot.supports(NAOQI):
        joint = "LHand" if gripper == "left" else "RHand"
        actions = [
            naoqi_request("motion", "openHand", [joint])
        ]
        return actions

    logger.warning("No module to open the gripper available. Skipping this action.")
    return []


@tested("04/10/2012")
@action
@workswith({POCOLIBS:['pr2SoftMotion']})
@workswith(NAOQI)
def close_gripper(robot, gripper = "right", callback = None):
    """ Closes the right gripper.
 
    If pr2SoftMotion-genom is available, it tries with it. Else it tries with fingers-genom.
    Parameter 'gripper' is ignored when using fingers-genom.

    :see: grab_gripper
    :param gripper: "right" (default) or "left"
    :param callback: if set, the action is non-blocking and the callback is invoked upon completion
    """
    gripper = gripper.lower()

    if robot.hasmodule("pr2SoftMotion"):
        if gripper == "right":
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

    if robot.supports(NAOQI):
        joint = "LHand" if gripper == "left" else "RHand"
        actions = [
            naoqi_request("motion", "closeHand", [joint])
        ]
        return actions

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
    if robot.supports(POCOLIBS):
        return [genom_request("pr2SoftMotion", 
                "SetGripperTresholds", 
                [grab_acc, grab_slip, release_acc, release_slip, force])]
    return []

def haspickedsmthg(robot, gripper = "right"):
    """
    Returns a tuple (True, None) if something is detected in the
    gripper.
    else return a tuple (False, [error msg]).

    Meant to by used as a pyRobots' "python_request".

    If not access to the robot joints, assume always true.

    :param gripper: "right" or "left". By default, right gripper.
    """

    try:
        gripper_joint = robot.state.getjoint(gripper[0] + '_gripper_joint')

        if gripper_joint < 0.01:
            logger.info("I think I've nothing in my " + gripper + " gripper: gripper fully closed")
            return (False, "gripper joint < 0.01")
        else:
            if robot.state.fingerpressed(gripper):
                return (True, None)
            else:
                logger.info("I think I've nothing in my " + gripper + " gripper: no pressure on the finger tip")
                return (False, "no pressure on the finger tips")
    except NameError: #no robot.state? consider the pick is successfull
        return (True, None)
    except AttributeError as detail: #no robot.state? consider the pick is successfull
        return (True, None)

@tested("04/10/2012")
@action
def pick(robot, obj, use_cartesian = "GEN_FALSE"):
    """ Picks an object that is reachable by the robot.

    Uses MHP to plan a trajectory.
    The trajectory is executed with by pr2SoftMotion-genom if available, 
    else with lwr-genom if available, else oonly exported to the mhpArmTraj poster.

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

def twoPointDist(p1,p2):
    """ compute the distance between two point

        point in form : (x,y)

        :param p1: first point
        :param p2: second point
    """
    return math.sqrt(math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2))

def computeDist(pr,p1,p2):
    """ compute the distance between a point and a segment
      
        pr is the point and p1 p2 constitute the segment vertices (all in 2d)
        if the orthonormal projection is not in the segment, the nearest vertice is 
        selected to compute the distance.

       :param pr: the point
       :param p1,p2: the segment
    """
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = pr[0]
    y3 = pr[1]
       

    px = x2-x1
    py = y2-y1

    something = px*px + py*py

    u =  ((x3 - x1) * px + (y3 - y1) * py) / something

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    dist = math.sqrt(dx*dx + dy*dy)

    return dist


def getNearAndFarPoints(pr,p1,p2,p3,p4):
    """ Compute the nearest edge
  
        Compute the nearest edge between pr and all the edges possible between
        the 2d points p1, p2, p3, p4. return the distance to the nearest edge,
        a pair of points from the nearest edge, and a pair of point from the
        fartest edge

        :param pr:          point from where to compute the distances
        :param p1,p2,p3,p4: points that constitute the edges
    """
    dists = {"d1":(computeDist(pr,p1,p2), (p1,p2), (p3,p4)),
             "d2":(computeDist(pr,p1,p3), (p1,p3), (p2,p4)),
             "d3":(computeDist(pr,p1,p4), (p1,p4), (p2,p3)),
             "d4":(computeDist(pr,p2,p3), (p2,p3), (p1,p4)),
             "d5":(computeDist(pr,p2,p4), (p2,p4), (p1,p3)),
             "d6":(computeDist(pr,p3,p4), (p3,p4), (p1,p2))}
    item= "d1"
    d = dists[item][0]
    for i in dists:
     # print("id = " + str(i) + " dist = " + str(dists[i][0]))
      if dists[i][0] < d :
        d = dists[i][0]
        item = i

    return (dists[item][0], dists[item][1],dists[item][2])



def getPlaceFromObject(robot, support, poseDist):
   """ Compute a 3d place to place an object
       
       use the bounding box of the support to find a place separated from the edge with dist

       :param robot: the robot
       :param support: the object on what we want to put something
       :param dist:    the distance between the edge and the position in meters
   """
   ok, bb = robot.execute([genom_request("spark", "GetBBPoints", [support])])
   ok, bb = robot.poco_modules["spark"].GetBBPoints(support)
   logger.info("The robot will try to put the object in hand in support : " + support)
   if ok=="OK":
      #the height is unique!
      support_height = float(bb[2])

      #point of 2d Bounding box :
      p1 = (float(bb[0]),float(bb[1]))
      p2 = (float(bb[3]),float(bb[4]))
      p3 = (float(bb[6]),float(bb[7]))
      p4 = (float(bb[9]),float(bb[10]))

      #robot 2d pose
      t = robot.poses.myself()
      pr = (t["x"],t["y"])
      
      #geting the near edge and the far edge
      (dist,near,far) = getNearAndFarPoints(pr,p1,p2,p3,p4)
      
      #computing the nearest point:
      cx = (0,0)

      if dist == twoPointDist(pr,near[0]):
        cx = near[0]
      elif dist == twoPointDist(pr,near[1]):
        cx = near[1]
      elif near[0][0] - near[1][0] == 0:
        cx = (near[0][0], pr[2])
      else:
        m = (near[0][1] - near[1][1])/(near[0][0] - near[1][0])
        c = near[0][1] -m* near[0][0]
        cons = pr[1] + pr[0]/m
        cx = (((cons-c)*m)/(1+m*m), ((c - cons)/(1+m*m))+cons)
      scale = poseDist/(twoPointDist(pr,cx) + poseDist)
      ppt = (cx[0] - scale*(pr[0]-cx[0]), cx[1] - scale*(pr[1]-cx[1]) )
      logger.info("the placing point : ( " + str(ppt[0]) + " , " + str(ppt[1]) + " , " + str(support_height) + " )")
      return (ppt[0], ppt[1], support_height)





@action
def put(robot, obj, support):
    """ Put an object on a given support

    Uses MHP to plan a trajectory.
    The trajectory is executed with by pr2SoftMotion-genom if available, 
    else with lwr-genom if available, else oonly exported to the mhpArmTraj poster.

    :param object: the object to put.
    :param support: the support to put the object
    """
   
    (x,y,z) = getPlaceFromObject(robot, support, 0.2)
    # Plan trajectory to object and execute it
    actions = [
    genom_request("mhp", "ArmPlanTask",
            [0,
            'GEN_TRUE',
            'MHP_ARM_PLACE_FROM_FREE',
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            obj,
            'NO_NAME',
            support,
            'GEN_FALSE',
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0, 
            9, #whichrot (rotation of the objet) 
            x, y, z, 
            0.0, 0.0, 0.0]),
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
    actions += open_gripper(robot)

    #actions += [python_request(haspickedsmthg)]

    actions += [
    genom_request("mhp", "ArmPlanTask",
            [0,
            'GEN_TRUE',
            'MHP_ARM_ESCAPE_OBJECT',
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            obj,
            'NO_NAME',
            'NO_NAME',
            'GEN_FALSE',
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0,
            0, #whichrot (rotation of the objet) 
            0, 0, 0,
            0.0, 0.0, 0.0]),
        genom_request("mhp", "ArmSelectTraj", [0])]

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



    return actions


@tested("04/10/2012")
@action
def attachobject(robot, obj, attach = True, hand = "right", holder = None):
    """ attach or detach the object to the robot hand
    
    This function should be called after a release or a grab
    :param obj: the object to attach/dettach.
    :param attach: true (default) to attach an object, false to detach it
    """

    if not holder:
        holder = robot.id

    actions = []
    if attach:
        #causes a inconsistency in ORO!
        actions += add_knowledge([holder + " hasIn" + hand.capitalize() + "Hand " + obj])
        #pass
    else:
        actions += retract_knowledge([holder + " hasIn" + hand.capitalize() + "Hand " + obj])

    i = 0
    if (attach) :
      i = 1
    actions += [
        genom_request("spark","SetGraspedObject", [obj, i, 0]),
        genom_request("spark","SetInferrenceForObject", [obj, i, holder, 0,
            "SPARK_PRECISE_ROBOT_HAND", 1.0])
    ]

    return actions

@tested("04/10/2012")
@action
@workswith({POCOLIBS:['pr2SoftMotion']})
@workswith(NAOQI)
def basictake(robot, target = None):
    """ The ultra stupid basic TAKE: simply hand the object in front of the
    robot.
    """
    actions = []
    if robot.supports(NAOQI):
        if not target:
            target = [1,0,0.1,'base_link']
        actions += configuration.movearm(robot, target)
    else:
        actions += configuration.setpose(robot, robot.postures["GIVE"])
    actions += open_gripper(robot)
    actions += [wait(2)]
    actions += grab_gripper(robot)
    actions += configuration.manipose(robot)
        
    return actions

@tested("04/10/2012")
@action
def take(robot, human, obj):
    """ Go to the human and take an object.

    Uses the OTP algorithm to find a path + posture to
    take an object from the human.
    """

    actions = look_at.look_at(robot, human)
    actions += configuration.manipose(robot, nop)
    actions += look_at.look_at(robot, [1,0,0.7,"base_link"])

    mobility = 0.1

    res = robot.planning.handover(human, mobility = mobility)

    if not res:
        logger.warning("OTP planning failed. Retrying.")
        robot.sorry()
        res = robot.planning.handover(human, mobility=mobility)
        if not res:
            logger.error("OTP planning failed again. Giving up.")
            return []

    wps, pose = res

    torso = pose["TORSO"][0]
    actions += configuration.settorso(robot, torso, nop)

    actions += nav.waypoints(robot, wps)
    actions += look_at.look_at(robot, human,nop)

    actions += configuration.setpose(robot, pose)

    actions += speech.say(robot, "Ok, I take it")
    actions += [genom_request("mhp", "ErasePath", [])] # remove the display of the trajectory
    actions += open_gripper(robot)
    actions += [wait(0.5)]
    actions += grab_gripper(robot)

    return actions

@tested("04/10/2012")
@action
def basicgive(robot):
    """ The ultra stupid basic GIVE: simply hand the object in front of the
    robot.
    
    After handing the object, the robot waits for someone to take it, and
    stay in this posture. 
    """
    actions = configuration.setpose(robot, robot.postures["GIVE"])
    actions += release_gripper(robot)
    actions += [wait(2)]
    actions += close_gripper(robot, callback = nop)
        
    return actions

@tested("04/10/2012")
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
            robot.say("I'm sorry, I got confused with my trajectory...")
            return []

    wps, pose = res
    torso = pose["TORSO"]
    logger.debug("the torso should be at height : " + str(torso[0]))
    actions += configuration.settorso(robot, torso[0], nop)

    actions += nav.waypoints(robot, wps, feedback = feedback)
    actions += look_at.look_at(robot, human,nop)
    
    actions += configuration.settorso(robot, torso[0]) #ensure the torso had time to actually reach the right height

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
    actions += [genom_request("mhp", "ErasePath", [])] # remove the display of the trajectory

    return actions


@tested("13/11/2012")
@action
def amit_give(robot, plan_id):
    actions = [
        genom_request(	"mhp",
            "Get_SM_Traj_HRI_Task",
            [plan_id]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["ROPEN"]
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
            ["RCLOSE"]
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
            ["RRELEASE"]
        )
    ]
    return actions

@tested("13/11/2012")
@action
def put_accessible(robot, plan_id):
    """ Grasps + shows the object to a 'receiver'
    """
    actions = [
        genom_request(	"mhp",
            "Get_SM_Traj_HRI_Task",
            [plan_id]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["ROPEN"]
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
            ["RCLOSE"]
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
            ["ROPEN"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [5]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        )
    ]

    return actions

@tested("13/11/2012")
@action
def show(robot, plan_id):
    """ Grasps + shows the object to a 'receiver'
    """
    actions = [
        genom_request(	"mhp",
            "Get_SM_Traj_HRI_Task",
            [plan_id]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["ROPEN"]
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
            ["RCLOSE"]
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
        )
    ]
    return actions

@tested("13/11/2012")
@broken
@action
def hide(robot, plan_id):
    actions = [
        genom_request(	"mhp",
            "Get_SM_Traj_HRI_Task",
            [plan_id]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["ROPEN"]
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
            ["RCLOSE"]
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
            ["ROPEN"]
        )
    ]
    return actions




