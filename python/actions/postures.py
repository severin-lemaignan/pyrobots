import random
from action import action, genom_request
from helpers import trajectory

@action
def release_gripper():
	"""
	Like gripper_open, except it waits util it senses some effort on the gripper force sensors.
	"""
        return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["RELEASE"])]

@action
def open_gripper():
        return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["OPEN"])]

@action
def close_gripper():
        return [genom_request("pr2SoftMotion", "GripperGrabRelease", ["CLOSE"])]


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
def gotopostureraw(posture, part = 'RARM', r_wrist_angle = 0.0):
    """
    Like gotoposture except it does not plan (no collision avoidance)
    """
    if part not in ['RARM']:
	print("'Go to posture raw' for part " + part + " is not implemented.")

    q1, q2, q3, q4, q5, q6, q7 = posture
    actions = [
	genom_request("pr2SoftMotion", "GotoQ",
    		[part,
    		 0,
    		 0.2, 0.0, 0.6, 0.0,
    		 q1, q2, q3, q4, q5, q6, q7, r_wrist_angle, 0.0, # Right arm
    		 0.73, 0.86, 0.06, -2.09, 2.42, -1.29, -2.95, 0.0, 0.0]) # Left arm (ignored)
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

