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

def ondesires(e):
    logger.info("Incoming desires:" + str(e))
    for d in e:
        incoming_desires.put(d)

def onemotion(e):
    logger.info("New emotional state:" + str(e))

def start_interaction(dummy = False):
    with robots.Robot(knowledge = kb.KB(), dummy = dummy) as robot:

        # Subscribe to new human orders
        robot.knowledge.subscribe([human + " desires ?d"], ondesires)

        # Subscribe to changes of emotional state
        robot.knowledge.subscribe(["myself experiences ?s"], onemotion)
        try:
            logger.info("Waiting for desires...")
            while True:
                sit = incoming_desires.get()

                if sit:
                    try:
                        desire = desires.desire_factory(sit, robot)
                        desire.perform()
                    except desires.NotExistingDesireTypeError as e:
                        logger.error(e)
                        logger.info("Skipping this desire.")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass


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
