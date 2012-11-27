#! /usr/bin/env python

import logging
logger = logging.getLogger("robot")
logger.setLevel(logging.DEBUG)

console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)

logger.addHandler(console)


import sys
import time
import Queue as queue

import pyoro
import robots
from robots.behaviours import desires, interrogation
from robots.helpers.cb import nop

human = "HERAKLES_HUMAN1"
human_mood = "NORMAL"

incoming_desires = queue.Queue()
incoming_human_experiences = queue.Queue()

def on_human_experience(e):
    logger.info("Something happened to the human!! Details:" + str(e))
    for d in e:
        incoming_human_experiences.put(d)

def onemotion(e):
    logger.warning("New emotional state:" + str(e))

with robots.PR2(knowledge = pyoro.Oro(), init = False) as pr2:

    desires_performer = desires.DesiresPerformer(pr2)

    # Callback for desires
    def ondesires(e):
        logger.info("Incomig desires:" + str(e))
        for sit in e:
            try:
                desire = desires.desire_factory(sit, pr2)

                # Has the new desire an higher priority? if yes, interrupt current one.
                desires_performer.trysuperseed(desire)
                incoming_desires.put(desire)
            except desires.NotExistingDesireTypeError as e:
                logger.warning(str(e))
                logger.warning("Skipping this desire.")

    def onverbalization(e):
        for t in e:
            text = pr2.knowledge["%s verbalisesTo *" % t][0]
            logger.warning("New verbalization from Dialogs: <%s>" % text)
            pr2.say(t)


    if "--init" in sys.argv:
        logger.info("Initializing the robot...")
        pr2.init(p3d = "/u/magharbi/openrobots/share/move3d/assets/ADREAM/ADREAM-assets.p3d")

    # Subscribe to new human orders
    pr2.knowledge.subscribe([human + " desires ?d"], ondesires)

    # subscribe to changes of emotional state
    pr2.knowledge.subscribe(["myself experiences ?s"], onemotion)

    # subscribe to changes of emotional state
    pr2.knowledge.subscribe([human + " experiences ?s"], on_human_experience)

    pr2.knowledge.subscribe(["?sit verbalisesTo ?s"], onverbalization, var = "?sit")

    try:
        logger.info("Waiting for events...")
        while True:
            try:
                human_evt = incoming_human_experiences.get(False)

                if human_evt:
                    evt_type = pr2.knowledge.getDirectClassesOf(human_evt).keys()
                    logger.debug("Type of human event: " + str(evt_type))
                    if "InterrogativeState" in evt_type:
                        logger.info("The interactor is asking a question. Let's handle it.")
                        question = interrogation.question_factory(human_evt, pr2)
                        question.perform()

                    if "Fall" in evt_type:
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
                            pr2.wait(1.5)
                            pr2.close_gripper()
                            pr2.manipose(nop)
                            pr2.translate(-0.2)
            except queue.Empty:
                pass

            try:
                desire = incoming_desires.get(False)
                desires_performer.perform(desire)
            except queue.Empty:
                pass

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

