import random
from exception import RobotError
from action import action, genom_request
from helpers import trajectory, postures

import os

@action
def enabledevileye():
    os.system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 3")

@action
def disabledevileye():
    os.system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 1")
 
@action
def setpose(posture, callback = None, part = None, collision_avoidance = False, obj = 'PR2_ROBOT', support = 'NO_NAME'):
    """
    Set the PR2 joints in a given configuration.

    Set a posture taking or not (depend on collision_avoidance state) into account collisions.
   
    If posture = {} and part = 'PR2', all parameters take 0 for value
 
    :param posture: a posture, ie, a dictionary built as follow:
    
      * Key '''RARM''': array of 7 joint angles for the right arm, in radians,
      * Key '''LARM''': array of 7 joint angles for the left arm, in radians,
      * Key '''TORSO''': the height of the torso ([0-30]), in centimeters,
      * Key '''HEAD''': array of pan and tilt values, in radians,

    :param part: (optional) force only a certain part of the PR2 to execute to posture.
    Allowed values: 'RARM', 'LARM', 'ARMS', 'PR2', 'PR2SYN', 'TORSO', 'PR2NOHEAD', 'HEAD'
    :param collision_avoidance: (optional), If true, the robot will be consider obstacles 
    in its environment. If true, you have to precise part = 'RARM'.
    :param obj: (optional) precise which objet the robot has to avoid during the movement. 
    You can give only one object.
    :param support: (optional precise which support the robot has to avoid during the movement.
    You can give only one support.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.
    """

    if collision_avoidance and not part == 'RARM':
        raise RobotError("Can't use collision avoidance with other part than RARM")

    if part and part not in ['RARM', 'LARM', 'ARMS', 'PR2', 'PR2SYN', 'TORSO', 'PR2NOHEAD', 'HEAD']:
        print("'Go to posture raw' for part " + part + " is not implemented.")

    print (posture)

    forced_part = part

    try:
        rq1, rq2, rq3, rq4, rq5, rq6, rq7 = posture['RARM']
        if not forced_part:
            part = 'RARM'
    except KeyError:
        rq1, rq2, rq3, rq4, rq5, rq6, rq7 = [0.0] * 7

    try:
        lq1, lq2, lq3, lq4, lq5, lq6, lq7 = posture['LARM']
        if not forced_part:
            if part == 'RARM':
                part = 'ARMS'
            else:
                part = 'LARM'
    except KeyError:
        lq1, lq2, lq3, lq4, lq5, lq6, lq7 = [0.0] * 7

    try:
        [torso] = posture['TORSO']
        if not forced_part:
            if part == 'ARMS':
                part = 'PR2NOHEAD'
            elif not part:
                part = 'TORSO'
            else:
                raise RobotError("Can not move only one arm and the torso in one call. Please call gotopostureraw twice.")
    except KeyError:
        torso = 0.0

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
         pan, tilt= 0.0, 0.0


    if not collision_avoidance:
        actions = [
        genom_request(
                    "pr2SoftMotion", 
                    "GotoQ",
                [forced_part if forced_part else part,
                0,
                torso, pan, tilt, 0.0,
                    rq1, rq2, rq3, rq4, rq5, rq6, rq7, 0.0, 0.0, # Right arm
                lq1, lq2, lq3, lq4, lq5, lq6, lq7, 0.0, 0.0], # Left arm
                wait_for_completion = False if callback else True,
                callback = callback)
            ]

    else: #collision_avoidance == True
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
        
        genom_request("mhp", "ArmSelectTraj", [0]),
            genom_request("pr2SoftMotion", 
            "TrackQ",['mhpArmTraj', 'PR2SM_TRACK_POSTER', part],
            wait_for_completion = False if callback else True,
            callback = callback)
        ]

    return actions

@action
def manipose(nohead = True, callback = None):
    """
    Quick method to set the PR2 joints in manip configuration. 
    It's useful when the robot to handle objects in in free spaces.

    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """
    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'
    
    pose = postures.read()
    posture = pose['MANIP']
    
    return setpose(posture, callback, part)

@action
def restpose(nohead = True, callback = None):
    """
    Quick method to set the PR2 joints in rest configuration. 
    You have the choice with three rest configuration. This choice is random.
    
    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object.
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """


    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'

    n = random.randint(1,3)
    pose = postures.read()
    posture = pose['REST' + str(n)]

    return setpose(posture, callback, part)

@action
def tuckedpose(callback = None, nohead = True):
    """
    Quick method to set the PR2 joints in rest configuration. 
    You have the choice with three rest configuration. This choice is random.

    :param no_head: (optional) If true, only arms and torso will set a new configuration.
    Very useful if you track an object
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.

    """

    if nohead:
        part = 'PR2NOHEAD'
    else:
        part = 'PR2'

    pose = postures.read()
    posture = pose['TUCKED']

    return setpose(posture, callback, part)

@action
def idle(choice = None,callback = None):
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
    pose = postures.read()
 
    actions = []
    if (c == 0):
        posture1 = pose['TUCKED']
        posture2 = pose['WATCH_LOOKING']
        actions = setpose(posture1,callback,part) + setpose(posture2,callback,part) + setpose(posture1,callback,part) 
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
        posture1 = pose['TUCKED']
        posture2 = pose['WATCH_LOOKING']
        actions = setpose(posture1,callback,part) + setpose(posture2,callback,part) + setpose(posture1,callback,part)
    elif(c == 9):
        posture1 = pose['TUCKED']
        posture2 = pose['WATCH_LOOKING']
        actions = setpose(posture1,callback,part) + setpose(posture2,callback,part) + setpose(posture1,callback,part)

    return actions 
     



"""
@action
def idle():
    :param dfsdfsd: 

    import look_at
    return look_at.sweep_look(..., nop) + [
         genom_request(
                    "pr2SoftMotion", 
                    "GotoQ",
                ["ARMS",
                0,
                torso, pan, tilt, 0.0,
                    rq1, rq2, rq3, rq4, rq5, rq6, rq7, 0.0, 0.0, # Right arm
                lq1, lq2, lq3, lq4, lq5, lq6, lq7, 0.0, 0.0], # Left arm
                wait_for_completion = False if callback else True,
                callback = callback)
         ]
""" 
