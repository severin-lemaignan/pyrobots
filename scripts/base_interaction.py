#! /usr/bin/env python

import logging
logger = logging.getLogger("robots")
logger.setLevel(logging.DEBUG)

console = logging.StreamHandler()
console.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)

logger.addHandler(console)

import sys
import time
import Queue as queue

import pyoro
import robots
from robots import desires

human = "HERAKLES_HUMAN1"

incoming_desires = queue.Queue()

def ondesires(e):
    logger.info("Incomig desires:" + str(e))
    for d in e:
        incoming_desires.put(d)

def onemotion(e):
    logger.info("New emotional state:" + str(e))

with robots.PR2(knowledge = pyoro.Oro(), init = False) as pr2:

    if not "--noinit" in sys.argv:
        pr2.init(p3d = "/u/slemaign/openrobots/share/move3d/assets/ADREAM/ADREAM.p3d",
                 sce = "/u/slemaign/openrobots/share/move3d/assets/ADREAM/SCENARIOS/ADREAM-salon-only.sce")
        pr2.setenvironment()

    # Subscribe to new human orders
    pr2.knowledge.subscribe([human + " desires ?d"], ondesires)

    # Subscribe to changes of emotional state
    pr2.knowledge.subscribe(["myself experiences ?s"], onemotion)
    try:
        logger.info("Waiting for desires...")
        while True:
            sit = incoming_desires.get()

            if sit:
                try:
                    desire = desires.desire_factory(sit, pr2)
                    desire.perform()
                except desires.NotExistingDesireTypeError as e:
                    logger.error(e)
                    logger.info("Skipping this desire.")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

