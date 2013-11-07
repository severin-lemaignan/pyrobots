#! /usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import sys
import time
import Queue as queue

import kb
import robots
from robots.behaviours import desires, interrogation

human = "HUMAN1"

incoming_desires = queue.Queue()
salient_events = queue.Queue()
emotional_states = queue.Queue()

def onsaliency(evt):
    logger.info("Salient event:" + str(evt))
    for d in evt:
        salient_events.put(d)

def ondesires(evt):
    logger.info("Incoming desires:" + str(evt))
    for d in evt:
        incoming_desires.put(d)

def onemotion(evt):
    logger.info("New emotional state:" + str(evt))
    for d in evt:
        emotional_states.put(d)

def start_interaction(dummy = False):
    with robots.Robot(knowledge = kb.KB(), dummy = dummy) as robot:

        # Subscribe to new human orders
        robot.knowledge.subscribe([human + " desires ?d"], ondesires)

        # Subscribe to new salient events
        robot.knowledge.subscribe(["?evt rdf:type SalientEvent"], onsaliency)

        # Subscribe to changes of emotional state
        robot.knowledge.subscribe(["myself experiences ?s"], onemotion)

        try:
            logger.info("Waiting for desires/salient event...")
            while True:
                try:
                    sit = incoming_desires.get_nowait()

                    try:
                        desire = desires.desire_factory(sit, robot)
                        desire.perform()
                    except desires.NotExistingDesireTypeError as e:
                        logger.error(e)
                        logger.info("Skipping this desire.")

                except queue.Empty:
                    pass

                try:
                    salientevt = salient_events.get_nowait()
                except queue.Empty:
                    pass

                try:
                    emotion = emotional_states.get_nowait()
                except queue.Empty:
                    pass


                time.sleep(0.05)

        except KeyboardInterrupt:
            logger.info("Quitting now")
            return


if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description='pyrobots base application.')
    parser.add_argument('-d', '--debug', action='store_true',
                                help='enables verbose output')
    parser.add_argument('-q', '--quiet', action='store_true',
                                help='be quiet (only errors are reported)')
    parser.add_argument('--dummy', action='store_true',
                                help='use a dummy middleware (outputs all actions to stdout)')
    args = parser.parse_args()



    logger = logging.getLogger("robot")
    kblogger = logging.getLogger("kb")

    if args.debug:
        logger.setLevel(logging.DEBUG)
        kblogger.setLevel(logging.DEBUG)
    elif args.quiet:
        logger.setLevel(logging.ERROR)
        kblogger.setLevel(logging.ERROR)
    else:
        logger.setLevel(logging.INFO)
        kblogger.setLevel(logging.INFO)

    console = logging.StreamHandler()
    console.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
    console.setFormatter(formatter)

    logger.addHandler(console)
    kblogger.addHandler(console)

    start_interaction(args.dummy)
