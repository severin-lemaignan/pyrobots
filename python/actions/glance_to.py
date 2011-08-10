import getQ
import json
from action import action, genom_request, wait
from helpers.jointstate import getjoint


@action
def glance_to(place, frame='/map'):
	""" Glance to via pr2SoftMotion
	"""
	
	head_tilt = getjoint('head_tilt_joint')
	head_pan = getjoint('head_pan_joint')

	actions = [
		genom_request("pr2SoftMotion", "MoveHead",
			 [ place['x'], place['y'], place['z'], frame ]
			),
		wait(2),
		genom_request("pr2SoftMotion", "GotoQ",
			["HEAD", 0, 0.0, head_pan,  head_tilt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			)
		]
	return actions
