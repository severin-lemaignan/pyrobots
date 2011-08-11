#! /usr/bin/env python

import json

import logging
logging.basicConfig(level = logging.DEBUG)

from actions import  glance_to, look_at, give, nav, games
from lowlevel import ActionPerformer


def getplaces():
	f = open('../share/novela_places.json','r')
	json_data=f.read()
	return json.loads(json_data)

def getpostures():
	f = open('../share/pr2_postures.json','r')
	json_data=f.read()
	return json.loads(json_data)


def getpr2():
	return ActionPerformer('pr2c1', 1235)


if __name__=="__main__":
	symbolic_places = getplaces()

	robot = getpr2()

	###############################################################################
	#	GAZE

	#robot.execute(look_at, symbolic_places["SHELF"])
	robot.execute(glance_to.glance_to, symbolic_places["SHELF"])

	###############################################################################
	#	NAVIGATION

	#robot.execute(ros_nav.ros_nav, symbolic_places["TABLE"])

	####Recorded navigation
	##To create a new one
	#robot.execute(new_recorded_nav, "file_name")

	##To execute an exisiting one
	#robot.execute(recorded_nav, "file_name")

	###############################################################################
	#	BODY MOVEMENTS

	#robot.execute(give, "PR2", "BOTTLE", "XAVIER")
	robot.execute(games.gym)
	#robot.execute(games.handsup)
	#robot.execute(games.arms_against_torso)
	#robot.execute(games.handsup_folded)
	#robot.execute(games.alternative_handsup_folded)
	#robot.execute(games.move_head)
	#robot.execute(games.larm_swinging)
        #robot.execute(games.rarm_swinging)  
	#robot.execute(games.slow_arms_swinging)
	#robot.execute(games.speed_arms_swinging)
	robot.close()
