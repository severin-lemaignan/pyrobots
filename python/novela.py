#! /usr/bin/env python

import json
import sys
import time
from random import *

import logging
logging.basicConfig(level = logging.DEBUG)


from lowlevel import ActionPerformer

from action import *
from helpers.abspose import *
from helpers.spark_replace import *
from actions.nav_line import *
from actions.nav import *


def getplaces():
	f = open('../share/novela_places.json','r')
	json_data=f.read()
	return json.loads(json_data)

def getpostures():
	f = open('../share/pr2_postures.json','r')
	json_data=f.read()
	return json.loads(json_data)


def getpr2():
	return ActionPerformer('pr2c2', 1235)

#logger.info("*** Novela will rock! ***")

def printer(e):
	print("End of lookat!! " + str(e))

if __name__=="__main__":


	if (len(sys.argv) > 1):

		robot = getpr2()
		print (sys.argv[1])

		if sys.argv[1] == 'Basket':
			print("####### GAMES - BASKET #######")
			
			robot.execute(place_agent, 'HERAKLES_HUMAN1', 5.0, -5.0)
			x_prec, y_prec = 5.0, -5.0

			j = 0
			i = sys.argv[2]
			while j < i:

				# Where is the human?
				robot.execute(getabspose, 'HERAKLES_HUMAN1', 'Pelvis')
				x, y, z = setabspose()

				# Evaluation of the distance that he is moved
				delta_x = x - x_prec
				delta_y = y - y_prec

				print("#######  DELTA_X AND DELTA_Y #######")
				print delta_x, delta_y

				# Mirror moving of the robot
				if (((delta_x > 0.2) or (delta_x < -0.2)) and (delta_y > 0.2) and (delta_y < -0.2)):
					nav_line(-delta_x, 0, 0.2)
				if (((delta_y > 0.2) or (delta_y < -0.2)) and (delta_x < 0.2) and (delta_x > -0.2)):
					nav_line(0, -delta_y, 0.2)
				if (((delta_x > 0.2) or (delta_x < -0.2)) and ((delta_y < 0.2) or (delta_y > -0.2 ))):
					nav_line(-delta_x, -delta_y, 0.2)
				else:
					pass

				# Moving of the human
				x_new = randint(-80, 80)/200 + x
				y_new = randint(-80, 80)/200 + y
				robot.execute(place_agent, 'HERAKLES_HUMAN1', x_new, y_new)
				
				time.sleep(2.0)
				x_prec, y_prec = x, y
				j += 1
		

		if sys.argv[1] == 'Multi_nav':
			
			print("####### MULTI NAV #######")
			reached_place = False
			destinations = ["SHELF", "TABLE"]	
			
			# Definition of the call back
			def cb(status, result):
				global state, reached_place, destinations
				print("Reached " + destinations[state])
				#import pdb; pdb.set_trace()
				reached_place = True
				state += 1

			while state < len(destinations):
				print("Going to " + destinations[state])
				robot.execute(nav.goto, symbolic_places[destinations[state]], cb)
				while not reached_place:
					time.sleep(0.1)

			reached_place = False

	robot.close()
