#! /usr/bin/env python

import json
import sys
import time
from random import *

import logging
logging.basicConfig(level = logging.DEBUG)


#from lowlevel import ActionPerformer
from dummylowlevel import ActionPerformer

from helpers import places
from helpers.position import getabspose, gethumanpose, mypose
from helpers.spark_replace import *
from action import *

from actions.nav import *
from actions.look_at import *
from actions.manipulation import *
from actions.configuration import *

def getpr2():
	return ActionPerformer(['pr2c2', 'pr2c1'], 1235)

def printer(e):
	print("End of lookat!! " + str(e))

if __name__=="__main__":

    robot = getpr2()

    p = places.read()

    robot.execute(carry, p["HQ"])
    robot.execute(basicgrab)
    robot.execute(carry, p["TABLE"])
    robot.execute(basicgive)
    robot.execute(track, p["AUDIENCE"])
    robot.execute(carry, p["AUDIENCE_WATCHING"])
    robot.execute(stop_tracking)
    robot.execute(sweep_look)
    robot.execute(carry, p["CENTER"])
    robot.close()
