#! /usr/bin/env python

import json
import sys
import time
from random import *

import logging
logging.basicConfig(level = logging.DEBUG)


from lowlevel import ActionPerformer

from helpers import places
from helpers.position import *
from helpers.spark_replace import *

from action import *
from actions.nav import *
from actions.look_at import *
from actions.manipulation import *
from actions.configuration import *

def getpr2():
	return ActionPerformer(['pr2c2', 'pr2c1'], 1235)

#logger.info("*** Novela will rock! ***")

def printer(e):
	print("End of lookat!! " + str(e))

if __name__=="__main__":


	if (len(sys.argv) > 1):

		robot = getpr2()
		symbolic_places = places.read()
		print (sys.argv[1])

		if sys.argv[1] == 'PickNavPlace':
			print("####### PICK NAV AND PLACE #######")

			robot.execute(pick, 'GREY_TAPE', 'IKEA_SHELF')
			robot.execute(nav_line, -0.2)
			robot.execute(rdytonav)
			robot.execute(rotation, 3.14)
			robot.execute(goto, symbolic_places["TEST"])
			robot.execute(manip_conf)
			robot.execute(nav_line, 0.2)
			res = robot.execute(getabspose,'PLACEMAT_BLUE')
			ans = res[1]
			x, y, z = float(ans[3]), float(ans[4]), float(ans[5])
			robot.execute(place, 'GREY_TAPE', 'HRP2TABLE', x, y, z)
	

		if sys.argv[1] == 'Basket':
			print("####### GAMES - BASKET #######")
			
			robot.execute(place_agent, 'HERAKLES_HUMAN1', 5.0, -5.0)
			x_prec, y_prec = 5.0, -5.0

			j = 0
			i = sys.argv[2]
			while j < i:

				# Where is the human?
				ok, res = robot.execute(getabspose, 'HERAKLES_HUMAN1', 'Pelvis')
				x, y, z = sparkcoords2xyz(res)

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
		

		if sys.argv[1] == 'carry':
			
			print("####### CARRY #######")
			robot.execute(carry, symbolic_places["HQ"])
			robot.execute(basicgrab)
			robot.execute(carry, symbolic_places["TABLE"])
			robot.execute(basicgive)
			robot.execute(track, symbolic_places["AUDIENCE"])
			robot.execute(carry, symbolic_places["AUDIENCE_WATCHING"])
			robot.execute(stop_tracking)
			robot.execute(sweep_look)
			robot.execute(carry, symbolic_places["CENTER"])

		if sys.argv[1] == 'Multi_nav':
			
			print("####### MULTI NAV #######")
			reached_place = False
			destinations = ["JARDIN_ENTER_OUT", "JARDIN_ENTER_IN", "AUDIENCE_WATCHING", "COUR_EXIT_IN", "COUR_EXIT_OUT"]	
			state = 0			
			# Definition of the call back
			def cb(status, result):
				global state, reached_place, destinations
				print("Reached " + destinations[state])
				#import pdb; pdb.set_trace()
				reached_place = True
				state += 1

			while state < len(destinations):
				print("Going to " + destinations[state])
				robot.execute(goto, symbolic_places[destinations[state]], cb)
				if ("ENTER_OUT" in destinations[state]) or ("EXIT_IN" in destinations[state]):
					robot.execute(rdytonav)
				elif ("ENTER_IN" in destinations[state]) or ("EXIT_OUT" in destinations[state]):
					robot.execute(manip_conf)
				else:
						pass
				while not reached_place:
					time.sleep(0.1)

				reached_place = False

	robot.close()
