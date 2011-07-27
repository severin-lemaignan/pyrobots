#! /usr/bin/env python

import json

import logging
logging.basicConfig(level = logging.DEBUG)

sys.path.append('../hri-scripts/share/')
f = open('../novela_places.json','rb')
json_data=f.read()
#print json_data
symbolic_places = json.loads(json_data)

from actions import give ros_nav
from lowlevel import ActionPerformer


robot = ActionPerformer(tclserv = "...", rosmaster = "...")

###############################################################################
#	GAZE

#robot.execute(lookat("BOTTLE"))

###############################################################################
#	NAVIGATION

robot.execute(ros_nav(symbolic_places["TABLE"])


###############################################################################
#	BODY MOVEMENTS

robot.execute(give("PR2", "BOTTLE", "XAVIER"))


