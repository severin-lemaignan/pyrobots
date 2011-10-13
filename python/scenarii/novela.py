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
from helpers.getch import getch

from actions.nav import *
from actions.primitive_nav import *
from actions.look_at import *
from actions.manipulation import *
from actions.configuration import *
from actions.games import *
from actions.administration import *

SCE_PATH = "/u/slemaign/openrobots/share/move3d/assets/novela/SCENARIO/"

def getpr2():
	return ActionPerformer('pr2c2', 9472)

def printer(e):
	print("End of lookat!! " + str(e))

# If true, will do all the trackings. Else, avoid the trackings that
# take place during other actions
fulltrack = False

def gotoniche():
    robot.execute(track, p["NICHE"])
    robot.execute(goto, p["NICHE"])
    robot.execute(cancel_track)
    robot.execute(look_at, p["AUDIENCE"])

def s1p1():
    robot.execute(tuckedpose)
    #robot.execute(close_gripper)
    robot.execute(setpose, poses["RIDEAU_1"])
    robot.execute(look_at, p["LOOK_NOWHERE"])
    raw_input("Press enter to start rideau openning")
    robot.execute(setpose, poses["RIDEAU_2"])
    robot.execute(look_at, p["AUDIENCE"])
    robot.execute(setpose, poses["RIDEAU_3"])
    robot.execute(setpose, poses["RIDEAU_4"])
    robot.execute(sweep_look, 40)
    raw_input("Press enter to go back to HQ")
    robot.execute(tuckedpose)
    robot.execute(look_at, p["LOOK_NOWHERE"])
    robot.execute(goto, p["BACKSTAGE_JARDIN"])
    robot.execute(goto, p["JARDIN_ENTER_OUT"])
    robot.execute(look_at, p["AUDIENCE"])

def s2p1():
    robot.execute(track_human)

def s2p2():
    
    robot.execute(cancel_track)
    if fulltrack:
        robot.execute(track, p["TABLE_CENTER"])
    robot.execute(goto, p["TABLE"])
    robot.execute(cancel_track)
    robot.execute(track_human, "lWristX")
    raw_input("Press enter to lock the phone")
    robot.execute(lock_object, "PHONE_RED")
    raw_input("Press enter to lock the lamp")
    robot.execute(lock_object, "LAMP")
def s2p22():
    robot.execute(cancel_track)
    if fulltrack:
        robot.execute(track, p["HANGER"])
    robot.execute(goto, p["HANGER_WATCHING"])
    robot.execute(wait, 2)
    robot.execute(sweep_look, 10, 0.1)
def s2p23():
    robot.execute(lock_object, "HANGER")
    if fulltrack:
        robot.execute(track, p["NICHE"])
    robot.execute(goto, p["NICHE_WATCHING"])
    robot.execute(wait, 2)
    robot.execute(cancel_track)
    robot.execute(wait, 2)
    robot.execute(sweep_look, 10, 0.1)
def s2p24():
    robot.execute(lock_object, "SHELTER")
    robot.execute(setup_scenario, [SCE_PATH + "object_positions.sce"])
    gotoniche()
    robot.execute(wait, 5)
    robot.execute(idle, 3) 

def s2p3():
    """ Amene bouteille
    """
    robot.execute(look_at, p["TABLE_CENTER"])
    robot.execute(wait, 2)
    if fulltrack:
        robot.execute(track, p["HQ"])
    robot.execute(manipose, nop)
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(wait, 2)
    robot.execute(cancel_track)
    robot.execute(basicgrab)
    robot.execute(tuckedpose)
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
    robot.execute(wait, 5)
    #robot.execute(idle, 2)

def s4p1():
    #TODO: tracking ARMCHAIR tag
    if fulltrack:
        robot.execute(track_human)
    robot.execute(goto, p["TABLE"])
    robot.execute(wait, 2)
    robot.execute(cancel_track)
    robot.execute(track_human, "lWristX")
    raw_input("Press enter to lock the sofa")
    robot.execute(lock_object, "SOFA")
    robot.execute(cancel_track)

def s4p2():
    """
    Ventilo
    """
    poses = postures.read()
    if fulltrack:
        robot.execute(track, p["HQ"])
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(cancel_track)
    robot.execute(basicgrab)
    robot.execute(tuckedpose)
    robot.execute(goto, p["FLYING_PAPER"], nop)
    robot.execute(wait, 5)
    robot.execute(setpose, poses["GIVE"], nop)
    #TODO: making the paper fly

def s4p21():
    """Geste desespoir milieu scene
    """
    poses = postures.read()
    robot.execute(release_gripper)
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["CENTER_FACE"])
    robot.execute(setpose, poses["TRANSITION"])
    robot.execute(setpose, poses["DECEPTION"], nop)
    robot.execute(sweep_look, speed = 0.5)
    robot.execute(close_gripper)
    #TODO head movement

def s4p22():
    """Retour niche
    """
    gotoniche()
    robot.execute(tuckedpose)

def s4p3():
    """
    Retour techniciens
    """
    if fulltrack:
        robot.execute(track, p["HQ"])
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    #TODO : the robot should go to the tecnicians
    robot.execute(cancel_track)

def s4p4():
    """
    Exploration de nuit 
    """
    #TODO:regarder le sol devant proscenium
    if fulltrack:
        robot.execute(track, p["TABLE_CENTER"])
    robot.execute(enabledevileye)
    robot.execute(goto, p["CENTER_TABLE"])
    robot.execute(cancel_track)
    robot.execute(sweep_look, 50)
    #robot.execute(disabledevileye)

    if fulltrack:
        robot.execute(track, p["HANGER_LOW"])
    robot.execute(goto, p["TABLE"])
    #robot.execute(enabledevileye)
    robot.execute(cancel_track)
    robot.execute(sweep_look)

    robot.execute(setup_scenario, [SCE_PATH + "bazar.sce"])
    robot.execute(look_at, p["LOOK_FEET"])
    robot.execute(goto, p["NICHE"])
    robot.execute(disabledevileye)

def s5p1():
    robot.execute(track_human)

def s5p2():
    """
    Corbeille
    """
    poses = postures.read()
    robot.execute(cancel_track)
    robot.execute(setup_scenario, [SCE_PATH + "clean.sce"])
    if fulltrack:
        robot.execute(track, p["HQ"])
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(cancel_track)
    robot.execute(setpose, poses["TOP_PICK"], nop)
    robot.execute(open_gripper)
    robot.execute(wait, 3)
    robot.execute(grab_gripper)
    robot.execute(tuckedpose)
    robot.execute(goto, p["JARDIN_ENTER_IN"]) 

def s5p3():
    poses = postures.read()
    if fulltrack:
        robot.execute(track_human)
    basket(robot)
    robot.execute(setpose, poses["TRASHGAME_BACK"], nop)
    robot.execute(cancel_track)


def s5p4():
    """ Rend la corbeille au HQ 
    """
    if fulltrack:
        robot.execute(track, p["HQ"])
    robot.execute(setpose, poses["TRASHGAME_FRONT"])
    robot.execute(goto, p["JARDIN_EXIT_IN"])
    robot.execute(cancel_track)
    robot.execute(release_gripper)
    robot.execute(wait, 3)
    robot.execute(close_gripper)

def s6p1():
    robot.execute(goto, p["AUDIENCE_WATCHING"])

def s6p2():
    poses = postures.read()
    robot.execute(setpose, poses["TRANSITION"])
    robot.execute(gym)
    # PR2 se met en position menacante
    robot.execute(goto, p["MONSTER_POSE"], nop)
    robot.execute(setpose, poses["ARMS_UP"])

def s6p3():
    robot.execute(setpose, poses["TRANSITION"])
    robot.execute(tuckedpose, nop)
    robot.execute(goto, p["AUDIENCE_WATCHING"])
    robot.execute(look_at, p["AUDIENCE"])
 
if __name__=="__main__":

    robot = getpr2()

    p = places.read()
    poses = postures.read()

    try:
      #raw_input("Press enter to begin track human")
      #robot.execute(track_human)
      #raw_input("Press enter to end track human")
      #robot.execute(cancel_track)
      if getch("Press enter to init the robot") != chr(27):
           robot.execute(setup_scenario, [SCE_PATH + "clean.sce"]) #TODO: broken!!
           robot.execute(tuckedpose)
           robot.execute(goto, p["AUDIENCE_WATCHING"])
           robot.execute(look_at, p["LOOK_NOWHERE"])
           robot.execute(unlock_object, "BLUE_BOTTLE")
           robot.execute(unlock_object, "PHONE_RED")
           robot.execute(unlock_object, "LAMP")
           robot.execute(unlock_object, "SHELTER")
           robot.execute(unlock_object, "HANGER")
           robot.execute(unlock_object, "SOFA")
      print("[[[[[[[[[[[[[ END OF INIT ]]]]]]]]]]]]]")

      if getch("Press enter when robot in ready to open curtain...") != chr(27):
          s1p1()
      print("[[[[[[[[[[[[[ END OF SCENE 1 ]]]]]]]]]]]]]")

      if getch("Press enter when Xavier is recognized...") != chr(27):
	      s2p1()
      if getch("Press enter when Xavier in place...") != chr(27):
	      s2p2()
      if getch("Press enter to look at the hanger") != chr(27):
              s2p22()
      if getch("Press enter to lock hanger and go niche") != chr(27):
              s2p23()
      if getch("Press enter to lock shelter and go niche") != chr(27):
              s2p24()
      if getch("Press enter when Xavier is thirsty...") != chr(27):
	      s2p3()
      print("[[[[[[[[[[[[[ END OF SCENE 2 ]]]]]]]]]]]]]")

      if getch("Press enter when PR2 need to move away from main window...") != chr(27):
	      s3p1()
      print("[[[[[[[[[[[[[ END OF SCENE 3 ]]]]]]]]]]]]]")

      if getch("Press enter when robot need to track chair tag...") != chr(27):

	      s4p1()
      if getch("Press enter when Xavier is tooo hot...") != chr(27):
	      s4p2()
      if getch("Press enter when PR2 must release ventilo and be desesperate...") != chr(27):
	      s4p21()
      if getch("Press enter when PR2 must go back to niche...") != chr(27):
	      s4p22()
      #if getch("Press enter when PR2 must go back the technos...") != chr(27):
      #	      s4p3()

      if getch("Press enter when PR2 is ready for the night...") != chr(27):
	      s4p4()
      print("[[[[[[[[[[[[[ END OF SCENE 4 ]]]]]]]]]]]]]")
     
      if getch("Press enter when Xavier is kinected...") != chr(27):
	      s5p1() 
      if getch("Press enter to get the basket...") != chr(27):
	      s5p2()
      if getch("Press enter to bring PR2 to play the basket game...") != chr(27):
	      s5p3()     
      if getch("Press enter when finished with basket game...") != chr(27):
	      s5p4()
      print("[[[[[[[[[[[[[ END OF SCENE 5 ]]]]]]]]]]]]]")
     
      if getch("Press enter to send PR2 to gym (start at 20 sec)...") != chr(27):
	      s6p1()
      if getch("Press enter to start gym (start 36 sec after music start)...") != chr(27):
	      s6p2()
      if getch("Press enter to go back to center...") != chr(27):
	      s6p3()

      print("[[[[[[[[[[[[[ END OF SCENE 6 ]]]]]]]]]]]]]")
    except Exception as e:
      print(e)
    finally:
      robot.close()
