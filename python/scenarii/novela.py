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

def s1p1():
    #TODO: ouverture rideau, passage technos
    robot.execute(goto, p["AUDIENCE_WATCHING"])
    robot.execute(look_at, p["AUDIENCE"])
    robot.execute(goto, p["JARDIN_ENTER_OUT"])

def s2p1():
    robot.execute(track_human)

def s2p2():
    
    robot.execute(track, p["TABLE_CENTER"])
    robot.execute(goto, p["TABLE_CORNER"])
    robot.execute(wait, 2)
    robot.execute(cancel_track)
    robot.execute(look_at, p["HANGER"])
    robot.execute(sweep_look, 10, 0.1)
    robot.execute(look_at, p["TABLE_CENTER"])
    robot.execute(sweep_look, 80, 0.1)
    robot.execute(track_human)
    robot.execute(goto, p["NICHE_WATCHING"])
    robot.execute(wait, 2)
    robot.execute(cancel_track)
    robot.execute(look_at, p["NICHE"])
    robot.execute(sweep_look, 80, 0.1)
    robot.execute(goto, p["NICHE"])

    #TODO idle in niche

def s2p3():
    robot.execute(look_at, p["TABLE_CENTER"])
    robot.execute(track, p["HQ"])
    robot.execute(manipose, nop)
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(wait, 2)
    robot.execute(cancel_track)
    robot.execute(basicgrab)
    robot.execute(manipose, nop)
    robot.execute(goto, p["TABLE"])
    robot.execute(basicgive)
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["AUDIENCE_WATCHING"])
    robot.execute(wait, 2)
    robot.execute(look_at, p["AUDIENCE"])
    robot.execute(wait, 2)
    robot.execute(look_at, p["TABLE_CENTER"]) #TODO: track human?

def s3p1():
    robot.execute(goto, p["AUDIENCE_WATCHING_JARDIN"])
    robot.execute(look_at, p["AUDIENCE"])

def s3p2():
    #TODO: tracking ARMCHAIR tag
    robot.execute(track_human)
    robot.execute(goto, p["TABLE"])
    robot.execute(wait, 2)
    robot.execute(cancel_track)
    robot.execute(look_at, p["TABLE_CENTER"])

def s3p3():
    """
    Ventilo
    """
    poses = postures.read()
    robot.execute(track, p["HQ"])
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(cancel_track)
    robot.execute(basicgrab)
    robot.execute(setpose, poses["GIVE"], nop)
    robot.execute(goto, p["TABLE_CORNER"])

def s3p31():
    """Geste desespoir milieu scene
    """
    poses = postures.read()
    robot.execute(release_gripper)
    robot.execute(goto, p["CENTER_FACE"])
    robot.execute(setpose, poses["DECEPTION"])
    #TODO head movement

def s3p32():
    """Retour niche
    """
    gotoniche()

def s3p4():
    """
    Retour techniciens
    """
    robot.execute(track, p["HQ"])
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(cancel_track)

def s3p5():
    """
    Exploration de nuit 
    """
    #TODO:regarder le sol devant proscenium
    robot.execute(track, p["AUDIENCE_WATCHING"])
    robot.execute(goto, p["CENTER"])
    robot.execute(cancel_track)
    robot.execute(sweep_look)

    robot.execute(track, p["TABLE_CENTER"])
    robot.execute(goto, p["TABLE"])
    robot.execute(cancel_track)
    robot.execute(sweep_look)

    robot.execute(setup_scenario, [SCE_PATH + "bazar.sce"])
    gotoniche()

def s4p1():
    """
    Corbeille
    """
    robot.execute(setup_scenario, [SCE_PATH + "clean.sce"])
    robot.execute(track, p["HQ"])
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(cancel_track)
    robot.execute(basicgrab)
    #TODO: adjust basket game posture
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["CENTER"])

def s4p3():
    poses = postures.read()
    robot.execute(setpose, poses["TRASHGAME_BACK"], nop)


def s5p1():
    robot.execute(goto, p["AUDIENCE_WATCHING"])

def s5p2():
    poses = postures.read()
    robot.execute(setpose, poses["TRANSITION"])
    robot.execute(gym)
    # PR2 se met en position menacante
    robot.execute(goto, p["MONSTER_POSE"], nop)
    robot.execute(setpose, poses["ARMS_UP"])

def s5p3():
    robot.execute(setpose, poses["TRANSITION"])
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["CENTER_FACE"])
 
if __name__=="__main__":

    robot = getpr2()

    p = places.read()
    poses = postures.read()

    try:
#      robot.execute(setup_scenario, [SCE_PATH + "clean.sce"]) #TODO: broken!!
#      robot.execute(tuckedpose)
#      robot.execute(goto, p["HQ"])
#      robot.execute(look_at, p["LOOK_NOWHERE"])
#      print("[[[[[[[[[[[[[ END OF INIT ]]]]]]]]]]]]]")
#      raw_input("Press enter when robot in ready to open curtain...")
  
#      s1p1()
#      print("[[[[[[[[[[[[[ END OF SCENE 1 ]]]]]]]]]]]]]")
#      raw_input("Press enter when Xavier is recognized...")
#      s2p1()
#      raw_input("Press enter when Xavier in place...")
      s2p2()
#      idling()
      raw_input("Press enter when Xavier is thirsty...")
      s2p3()
      print("[[[[[[[[[[[[[ END OF SCENE 2 ]]]]]]]]]]]]]")
      raw_input("Press enter when PR2 need to move away from main window...")
      s3p1()

      raw_input("Press enter when robot need to track chair tag...")

      s3p2()
      raw_input("Press enter when Xavier is tooo hot...")
      s3p3()
      raw_input("Press enter when PR2 must release ventilo and be desesperate...")
      s3p31()
      raw_input("Press enter when PR2 must go back to niche...")
      s3p32()
      raw_input("Press enter when PR2 must go back the technos...")
      s3p4()

      raw_input("Press enter when PR2 has the frontale...")
      s3p5()
      
      print("[[[[[[[[[[[[[ END OF SCENE 3 ]]]]]]]]]]]]]")
      raw_input("Press enter to get the basket...")
      s4p1()
      raw_input("Press enter to bring PR2 to play the basket game...")
      basket(robot, poses)
      s4p3()     
      raw_input("Press enter when finished with basket game...")
      gotohq()
      print("[[[[[[[[[[[[[ END OF SCENE 4 ]]]]]]]]]]]]]")
     
      raw_input("Press enter to send PR2 to gym...")
      s5p1()
      raw_input("Press enter to start gym (start 38 sec after music start)...")
      s5p2()
      raw_input("Press enter to go back to center...")
      s5p3()

      print("[[[[[[[[[[[[[ END OF SCENE 5 ]]]]]]]]]]]]]")
    except Exception as e:
      print(e)
    finally:
      robot.close()
