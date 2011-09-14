import random
from exception import RobotError
from action import action, genom_request
from helpers import trajectory, postures

@action
def setpose(posture, part = None, collision_avoidance = False, obj = 'NO_NAME', support = 'NO_NAME',callback = None):
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
    :param callback: (optional) If given, the action is non-blocking, and the callback is
    invoked at the activity completion.
    """
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
        torso = posture['TORSO']
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
    return actions

