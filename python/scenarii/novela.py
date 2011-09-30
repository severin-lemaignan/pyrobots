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
from actions.games import *
from actions.administration import *

SCE_PATH = "/u/slemaign/openrobots/share/move3d/assets/novela/SCENARIO/"

def getpr2():
	return ActionPerformer(['pr2c2', 'pr2c1'], 9472)

def printer(e):
	print("End of lookat!! " + str(e))

#TODO: Fix 'carry'
#TODO: pour gagner du temps, prise d'objet dans l'embrasure
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
    robot.execute(restpose)
    robot.execute(manipose)
    robot.execute(glance_to, p["TABLE"])
    robot.execute(glance_to, p["HQ"])
    robot.execute(look_at, p["AUDIENCE"])

def s2p1():
    #TODO: ouverture rideau, passage technos
    robot.execute(goto, p["AUDIENCE_WATCHING"])
    robot.execute(look_at, p["AUDIENCE"])
    robot.execute(sweep_look)
    robot.execute(goto, p["JARDIN_ENTER_IN"])

def s2p2():
    #TODO: track Xavier
    robot.execute(look_at, p["CENTER"])
    robot.execute(look_at, p["TABLE"])
    robot.execute(look_at, p["AUDIENCE"])

def s2p3():
    #TODO: track table 
    #TODO: en fait, Xavier lui montre les tags
    robot.execute(look_at, p["TABLE_CENTER"])
    robot.execute(goto, p["TABLE_CORNER"])
    robot.execute(look_at, p["HANGER"])
    robot.execute(sweep_look, 10, 0.1)
    robot.execute(look_at, p["TABLE_CENTER"])
    robot.execute(sweep_look, 80, 0.1)
    robot.execute(look_at, p["NICHE"])
    robot.execute(goto, p["NICHE_WATCHING"])
    robot.execute(look_at, p["NICHE"])
    robot.execute(sweep_look, 80, 0.1)
    robot.execute(goto, p["NICHE"])

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
    poses = postures.read()
    robot.execute(look_at, p["HQ"])
    robot.execute(goto, p["HQ"])
    robot.execute(basicgrab)
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["JARDIN_ENTER_IN"])
    robot.execute(setpose, poses["GIVE"], nop)
    robot.execute(goto, p["TABLE_CORNER"])

def s3p31():
    """Geste desespoir milieu scene
    """
    robot.execute(release_gripper)
    #TODO: geste desespoir
    robot.execute(goto, p["CENTER_FACE"])

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
    #TODO:regarder le sol devant proscenium
    robot.execute(look_at, p["TABLE"])
    robot.execute(goto, p["TABLE"])
    robot.execute(look_at, p["TABLE"])
    robot.execute(sweep_look)

    robot.execute(look_at, p["HQ"])
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["HQ"])
    robot.execute(setup_scenario, [SCE_PATH + "bazar.sce"])

def s3p6():
    """
    Retour niche 
    """
    gotoniche()

def s4p1():
    """
    Corbeille
    """
    robot.execute(setup_scenario, [SCE_PATH + "clean.sce"])
    robot.execute(look_at, p["HQ"])
    robot.execute(goto, p["JARDIN_ENTER_OUT"])
    robot.execute(look_at, p["TABLE"], nop)
    robot.execute(basicgrab)
    #TODO: adjust basket game posture
    #robot.execute(tuckedpose, nop)
    robot.execute(goto, p["JARDIN_ENTER_IN"])
    #robot.execute(manipose, nop)

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

def s5p1():
    robot.execute(goto, p["AUDIENCE_WATCHING"])

def s5p2():
    poses = postures.read()
    robot.execute(gym)
    # PR2 se met en position menacante
    robot.execute(goto, p["MONSTER_POSE"], nop)
    robot.execute(setpose, poses["ARMS_UP"])

def s5p3():
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["CENTER_FACE"])
 
if __name__=="__main__":

    robot = getpr2()

    p = places.read()
    poses = postures.read()

    try:
#      robot.execute(tuckedpose)
#      robot.execute(goto, p["HQ"])
#      robot.execute(look_at, p["LOOK_NOWHERE"])
#      print("[[[[[[[[[[[[[ END OF INIT ]]]]]]]]]]]]]")
      raw_input("Press enter when robot in place...")
  
      s2p1()
      raw_input("Press enter when Xavier is recognized...")
      s2p2()
      raw_input("Press enter when Xavier in place...")
      s2p3()
#      idling()
#      raw_input("Press enter when Xavier is thirsty...")
#      s2p4()
#      print("[[[[[[[[[[[[[ END OF SCENE 2 ]]]]]]]]]]]]]")
#      raw_input("Press enter when robot need to track chair tag...")
#      #TODO idling
#      s3p1()
#      #TODO idling
#      idling()
#
#      raw_input("Press enter when PR2 need to look at Xavier...")
#
#      s3p2()
#      raw_input("Press enter when Xavier is tooo hot...")
#      s3p3()
#      raw_input("Press enter when PR2 must release ventilo and be desesperate...")
#      s3p31()
#      raw_input("Press enter when PR2 must go back to niche...")
#      s3p32()
#      raw_input("Press enter when PR2 must go back the technos...")
#      s3p4()

#      raw_input("Press enter when PR2 has the frontale...")
#      s3p5()
#      raw_input("Press enter to bring PR2 to the niche...")
#      s3p6()
      
#      print("[[[[[[[[[[[[[ END OF SCENE 3 ]]]]]]]]]]]]]")
#      raw_input("Press enter to get the basket...")
#      s4p1()
#      raw_input("Press enter when finished with basket game...")
#      #raw_input("Press enter to bring PR2 to play the basket game...")
#      #basket_game()
#      gotohq()
#      print("[[[[[[[[[[[[[ END OF SCENE 4 ]]]]]]]]]]]]]")
      
#      raw_input("Press enter to send PR2 to gym...")
#      s5p1()
#      raw_input("Press enter to start gym (start 38 sec after music start)...")
#      s5p2()
#      raw_input("Press enter to go back to center...")
#      s5p3()

#      print("[[[[[[[[[[[[[ END OF SCENE 5 ]]]]]]]]]]]]]")
    except Exception as e:
      print(e)
    finally:
      robot.close()
