#! /usr/bin/env python

import json
import sys
import time
from random import *

import logging
logging.basicConfig(level = logging.DEBUG)


from lowlevel import ActionPerformer
#from dummylowlevel import ActionPerformer

from helpers import places
from helpers.position import getabspose, gethumanpose, mypose
from helpers.spark_replace import *
from helpers.cb import nop
from action import *

from actions.nav import *
from actions.primitive_nav import *
from actions.look_at import *
from actions.manipulation import *
from actions.configuration import *

def getpr2():
	return ActionPerformer(['pr2c2', 'pr2c1'], 9472)

def printer(e):
	print("End of lookat!! " + str(e))

#TODO: Fix 'carry'

def gotoniche():
    #TODO: track niche 
    #robot.execute(track, p["NICHE"])
    robot.execute(look_at, p["NICHE"])
    robot.execute(goto, p["NICHE"])
    #robot.execute(cancel_track)
    robot.execute(look_at, p["AUDIENCE"])

def gotohq():
    #TODO: track HQ
    #robot.execute(track, p["NICHE"])
    robot.execute(look_at, p["HQ"])
    robot.execute(goto, p["HQ"])
    #robot.execute(cancel_track)
    robot.execute(look_at, p["HQ"])

def idling():
    #TODO: trigger it automatically
    pass

def s2p1():
    robot.execute(goto, p["AUDIENCE_WATCHING"])
    robot.execute(look_at, p["AUDIENCE"])
    robot.execute(sweep_look)
    gotoniche()

def s2p2():
    #TODO: track Xavier
    robot.execute(look_at, p["CENTER"])
    robot.execute(look_at, p["TABLE"])
    robot.execute(look_at, p["AUDIENCE"])

def s2p3():
    #TODO: track table 
    #TODO: en fait, Xavier lui montre les tags
    robot.execute(look_at, p["TABLE"])
    robot.execute(goto, p["TABLE"])
    robot.execute(look_at, p["TABLE"])
    robot.execute(sweep_look)
    #TODO: track hanger 
    robot.execute(look_at, p["HANGER_WATCHING"])
    robot.execute(goto, p["HANGER_WATCHING"])
    robot.execute(look_at, p["HANGER_WATCHING"])
    robot.execute(sweep_look)
    gotoniche()

    #TODO idle in niche

def s2p4():
    robot.execute(look_at, p["TABLE"])
    robot.execute(track, p["HQ"])
    robot.execute(goto, p["HQ"])
    robot.execute(cancel_track)
    robot.execute(basicgrab)
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["JARDIN_ENTER_IN"])
    robot.execute(manipose, nop)
    robot.execute(goto, p["TABLE"])
    robot.execute(basicgive)
    robot.execute(tuckedpose, nop)
    gotoniche()

def s3p1():
    #TODO tracking tag pour gros fauteuil au lieu chaise
    robot.execute(look_at, p["AUDIENCE"])

def s3p2():
    gotoniche()
    robot.execute(look_at, p["TABLE"])

def s3p3():
    """
    Ventilo
    """
    robot.execute(look_at, p["HQ"])
    robot.execute(goto, p["HQ"])
    robot.execute(basicgrab)
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["JARDIN_ENTER_IN"])
    robot.execute(manipose, nop)
    # TODO: position angle table pour ventilo
    robot.execute(goto, p["TABLE"])
    robot.execute(basicgive)

def s3p31():
    """Geste desespoir milieu scene
    """
    robot.execute(goto, p["CENTER"])

def s3p32():
    """Retour niche
    """
    gotoniche()

def s3p4():
    """
    Retour techniciens
    """
    robot.execute(look_at, p["HQ"])
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["HQ"])

def s3p5():
    """
    Exploration de nuit 
    """
    robot.execute(goto, p["JARDIN_ENTER_IN"])
    robot.execute(look_at, p["TABLE"])
    robot.execute(goto, p["TABLE"])
    robot.execute(look_at, p["TABLE"])
    robot.execute(sweep_look)

    robot.execute(look_at, p["HANGER_WATCHING"])
    robot.execute(goto, p["HANGER_WATCHING"])
    robot.execute(look_at, p["HANGER_WATCHING"])
    robot.execute(sweep_look)

    robot.execute(look_at, p["HQ"])
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["HQ"])
    # TODO: scene SPARK bordel

def s3p6():
    """
    Retour niche 
    """
    gotoniche()

def s3p7():
    """
    Re-exploration autour de Xavier qui remet les tags en place 
    """
    robot.execute(look_at, p["TABLE"])
    robot.execute(goto, p["TABLE"])
    robot.execute(look_at, p["TABLE"])
    robot.execute(sweep_look)

    robot.execute(look_at, p["HANGER_WATCHING"])
    robot.execute(goto, p["HANGER_WATCHING"])
    robot.execute(look_at, p["HANGER_WATCHING"])
    robot.execute(sweep_look)

    gotoniche()

def s4p1():
    """
    Corbeille
    """
    robot.execute(look_at, p["HQ"])
    robot.execute(goto, p["HQ"])
    robot.execute(basicgrab)
    #TODO: adjust basket game posture
    #robot.execute(tuckedpose, nop)
    robot.execute(goto, p["JARDIN_ENTER_IN"])
    #robot.execute(manipose, nop)
    #TODO: adjust basket game position
    robot.execute(goto, p["CENTER"])

def basket_game(): 
	j = 0
	duration = 5

	# Where is the human?
	human = position.gethumanpose(robot)

	x_prec = human["x"]
	y_prec = human["y"]

	while j < duration:

		# Where is the human?
		human = position.gethumanpose(robot)

		x = human["x"]
		y = human["y"]
                
                if x == 0.0 and y == 0.0:
                    #Human at origin
                    logger.warning("HUMAN not detected!!!")
		    time.sleep(1.0)
		    j += 1
                    continue

		# Evaluation of the distance that he is moved
		delta_x = x - x_prec
		delta_y = y - y_prec

		print("#######  DELTA_X AND DELTA_Y #######")
		print(delta_x, delta_y)

		# Mirror moving of the robot
		if (((delta_x > 0.2) or (delta_x < -0.2)) and (delta_y > 0.2) and (delta_y < -0.2)):
			# TODO: nav_line -> ugly code
			nav_line(-delta_x, 0.0,  0.3)
		if (((delta_y > 0.2) or (delta_y < -0.2)) and (delta_x < 0.2) and (delta_x > -0.2)):
			nav_line(0.0, -delta_y, 0.3)
		if (((delta_x > 0.2) or (delta_x < -0.2)) and ((delta_y < 0.2) or (delta_y > -0.2 ))):
			nav_line(-delta_x, -delta_y, 0.3)
		else:
			pass

		time.sleep(1.0)
		x_prec, y_prec = x, y
		j += 1
 
if __name__=="__main__":

    robot = getpr2()

    p = places.read()
    poses = postures.read()

    try:
#      robot.execute(tuckedpose)
#      robot.execute(goto, p["HQ"])
#      robot.execute(look_at, p["LOOK_NOWHERE"])
#      raw_input("Press enter when robot in place...")
#  
#      s2p1()
#      raw_input("Press enter when Xavier is recognized...")
#      s2p2()
#      raw_input("Press enter when Xavier in place...")
#      s2p3()
#      raw_input("Press enter when Xavier is thirsty...")
#      #TODO idling
#      s2p4()
#      print("[[[[[[[[[[[[[ END OF SCENE 2 ]]]]]]]]]]]]]")
#      raw_input("Press enter when robot need to track chair tag...")
#      #TODO idling
#      s3p1()
#      #TODO idling
#
#      raw_input("Press enter when Xavier is tooo hot...")
#
#      s3p2()
#      raw_input("Press enter when Xavier is tooo hot...")
#      s3p3()
#      raw_input("Press enter when PR2 must be desesperate...")
#      s3p31()
#      raw_input("Press enter when PR2 must go back to niche...")
#      s3p32()
#      raw_input("Press enter when PR2 must go back the technos...")
#      s3p4()
#
#      raw_input("Press enter when PR2 has the frontale...")
#      s3p5()
#      raw_input("Press enter to bring PR2 to the niche...")
#      s3p6()
#      
#      print("[[[[[[[[[[[[[ END OF SCENE 3 ]]]]]]]]]]]]]")
#      raw_input("Press enter to get the basket...")
#      s4p1()
#      raw_input("Press enter to bring PR2 to play the basket game...")
      basket_game()
      gotohq()
      
      print("[[[[[[[[[[[[[ END OF SCENE 4 ]]]]]]]]]]]]]")
    except Exception as e:
      print(e)
    finally:
      robot.close()
