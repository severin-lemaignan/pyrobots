import random
from exception import RobotError
from action import action, genom_request
from helpers import trajectory


@action
def rightarmgoto(posture, obj, support = 'HRP2TABLE', use_cartesian = 'GEN_FALSE'):
	return gotoposture(posture, obj, 'RARM', support, use_cartesian)

@action
def gotoposture(posture, obj, part = 'RARM', support = 'HRP2TABLE', use_cartesian = 'GEN_FALSE'):
    """ Set a posture taking into account
	collisions.
    """

    if part not in ['RARM', 'LARM']:
	print("'Go to' for part " + part + " is not implemented.")

    print (posture)
    q1, q2, q3, q4, q5, q6, q7 = posture
    actions = [
	genom_request("mhp", "ArmPlanTask",
    	 [0,
    	 'GEN_TRUE',
    	 'MHP_ARM_TAKE_TO_FREE',
    	 0,
    	 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    	 q1, q2, q3, q4, q5, q6, q7,
    	 obj,
    	 support,
    	 'NO_NAME',
    	 use_cartesian,
    	 0,
    	 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
    	 0, # Rotation type
    	 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), # x y z rx ry rz
        
	genom_request("mhp", "ArmSelectTraj", [0]),
        genom_request("pr2SoftMotion", "TrackQ",['mhpArmTraj', 'PR2SM_TRACK_POSTER', part])
    ]

    return actions

@action
def setpose(posture, part = None, callback = None):
    """
    Set the PR2 joints in a given configuration.

    Like gotoposture except it does not plan (no collision avoidance)
    
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
    if part not in ['RARM', 'LARM', 'ARMS', 'PR2', 'PR2SYN', 'TORSO', 'PR2NOHEAD', 'HEAD']:
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
        torso = 0.0


    actions = [
	genom_request(
            "pr2SoftMotion", 
            "GotoQ",
    		[    forced_part if forced_part else part,
        		 0,
        		 torso, pan, tilt, 0.0,
    	    	 rq1, rq2, rq3, rq4, rq5, rq6, rq7, 0.0, 0.0, # Right arm
    		     lq1, lq2, lq3, lq4, lq5, lq6, lq7, 0.0, 0.0], # Left arm
       		wait_for_completion = False if callback else True,
	    	callback = callback)
    ]
    return actions

@action
def rest():

        n = random.randint(1,3)

        if n == 1 :
                traj = trajectory.Trajectory('rest_position')

        elif n == 2 :
                traj = trajectory.Trajectory('rest_position2')

        elif n == 3 :
                traj = trajectory.Trajectory('rest_position3')

        actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + traj.initcoords())
                  ]

        return actions

@action
def rest_without_head():

        n = random.randint(1,3)

        if n == 1 :
                traj = trajectory.Trajectory('rest_position')

        elif n == 2 :
                traj = trajectory.Trajectory('rest_position2')

        elif n == 3 :
                traj = trajectory.Trajectory('rest_position3')

        actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['ARMS', 0] + traj.initcoords(),
                        wait_for_completion = False )
                  ]

        return actions

@action
def manip(callback = None):
	return [genom_request("pr2SoftMotion", 
				"GotoQ", 
				['PR2', 0,
				 0.2, 0.0, 0.6, 0.0,
				 -0.48157, 0.91579, 0.05525, -2.27493, -2.60623, -1.38896, 3.00473,
				 0.0, 0.0,
				 0.55276, 1.27965, 2.02007, -1.4, 0.0, 0.0, 0.0,
				 0.0, 0.0],
				wait_for_completion = False if callback else True,
				callback = callback
				)]

@action
def tucked(callback = None):
        return [genom_request("mhp", "ArmPlanTask", 
                       [0,
			'GEN_TRUE',
			'MHP_ARM_FREE',
			0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.05842, 1.28695, -0.80267, -2.17115, 3.14000, -1.11226, 3.0,
			'NO_NAME',
			'NO_NAME',
			'NO_NAME',
			'GEN_FALSE',
			0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
		 	0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
                        ),
		genom_request("mhp", "ArmSelectTraj", [0]),
		genom_request(	"pr2SoftMotion", 
				"TrackQ", 
				['mhpArmTraj', 'PR2SM_TRACK_POSTER', 'RARM'],
				wait_for_completion = False if callback else True,
				callback = callback)

		]
		
