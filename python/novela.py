#! /usr/bin/env python

import json

import logging
logging.basicConfig(level = logging.DEBUG)

from actions import give 
from lowlevel import ActionPerformer

json_data=open("../share/novela_places.json").read()
symbolic_places = json.loads(json_data)

robot = ActionPerformer(tclserv = "...", rosmaster = "...")

#robot.execute(lookat("BOTTLE"))
#robot.execute(recorded_goto(symbolic_places["TABLE"]))
robot.execute(give,"PR2", "BOTTLE", "XAVIER")


