#! /usr/bin/env python

import json

import logging
logging.basicConfig(level = logging.DEBUG)


f = open('../share/novela_places.json','rb')
json_data=f.read()
symbolic_places = json.loads(json_data)

from actions import give, ros_nav, look_at_genom 
from lowlevel import ActionPerformer


robot = ActionPerformer('pr2c1', 1235)

###############################################################################
#	GAZE

#robot.execute(look_at, symbolic_places["SHELF"])
#robot.execute(object_tracking, symbolic_places["SHELF"])

###############################################################################
#	NAVIGATION

robot.execute(ros_nav, symbolic_places["TABLE"])

####Recorded navigation
##To create a new one
#robot.execute(new_recorded_nav, "file_name")

##To execute an exisiting one
#robot.execute(recorded_nav, "file_name")

###############################################################################
#	BODY MOVEMENTS

#robot.execute(give, "PR2", "BOTTLE", "XAVIER")
#robot.execute(play_planned_traj, symbolic_traj["GYM"])
