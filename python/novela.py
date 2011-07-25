#! /usr/bin/env python

import logging
logging.basicConfig(level = logging.DEBUG)

from actions import give 
from lowlevel import ActionPerformer


#symbolic_places = json.load("../share/novela_places.json")

robot = ActionPerformer(tclserv = "...", rosmaster = "...")

#robot.execute(lookat("BOTTLE"))
#robot.execute(recorded_goto(symbolic_places["TABLE"]))
robot.execute(give,"PR2", "BOTTLE", "XAVIER")


