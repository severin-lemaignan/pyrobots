#! /usr/bin/env python

import logging
logger = logging.getLogger("robot")
logger.setLevel(logging.DEBUG)

#console = logging.StreamHandler()
#console.setLevel(logging.INFO)
#formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
#console.setFormatter(formatter)

#logger.addHandler(console)


import sys
import time
import Queue as queue

import pyoro
import robots
from robots.desires import desires
from robots.helpers.cb import nop

human = "HERAKLES_HUMAN1"
human_mood = "NORMAL"

incoming_desires = queue.Queue()
incoming_human_experiences = queue.Queue()

def on_human_experience(e):
    logger.info("Something happened to the human!! Details:" + str(e))
    for d in e:
        incoming_human_experiences.put(d)

def ondesires(e):
    logger.info("Incomig desires:" + str(e))
    for d in e:
        incoming_desires.put(d)

def onemotion(e):
    logger.info("New emotional state:" + str(e))

with robots.PR2(knowledge = pyoro.Oro(), init = False) as pr2:

    if "--init" in sys.argv:
        logger.info("Initializing the robot...")
        pr2.init(p3d = "/u/magharbi/openrobots/share/move3d/assets/ADREAM/ADREAM-assets.p3d")

    # Subscribe to new human orders
    pr2.knowledge.subscribe([human + " desires ?d"], ondesires)

    # subscribe to changes of emotional state
    pr2.knowledge.subscribe(["myself experiences ?s"], onemotion)

    # subscribe to changes of emotional state
    pr2.knowledge.subscribe([human + " experiences ?s"], on_human_experience)

    try:
        logger.info("Waiting for events...")
        while True:
            try:
                human_evt = incoming_human_experiences.get(False)

                if human_evt:
                    evt_type = pr2.knowledge.getDirectClassesOf(human_evt).keys()[0]
                    if evt_type == "Fall":
                        logger.info("The human falled down! Carambar!")
                        places = pr2.knowledge[human + " isAt *"]
                        if not places:
                            logger.error("I've no clue where the human is!")
                        else:
                            logger.info("I think the human is in " + str(places))
                            logger.info("I go to " + places[0])
                            pr2.manipose(nop)
                            pr2.look_at([1.0,0.0,1.0,"base_link"])
                            pr2.goto(places[0])
                            pr2.look_at([1.0,0.0,0.5,"base_link"])
                            pr2.setpose("FALL")
                            pr2.release_gripper()
                            pr2.close_gripper()
                            pr2.manipose(nop)
                            pr2.translate(-0.2)
            except queue.Empty:
                pass

            try:
                sit = incoming_desires.get(False)
                if sit:
                    try:
                        desire = desires.desire_factory(sit, pr2)
                        desire.perform()
                    except desires.NotExistingDesireTypeError as e:
                        logger.error(str(e))
                        logger.info("Skipping this desire.")
            except queue.Empty:
                pass

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

