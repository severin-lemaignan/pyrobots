#! /usr/bin/env python

import logging
logger = logging.getLogger("robots")
logger.setLevel(logging.DEBUG)

console = logging.StreamHandler()
console.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)

logger.addHandler(console)


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

with robots.PR2(knowledge = pyoro.Oro(), init = False) as pr2:

    pr2.init(p3d = "/u/slemaign/openrobots/share/move3d/assets/GS/gsPr2.p3d")
    pr2.knowledge.subscribe([human + " desires ?d"], ondesires)
    
    try:
    	logger.info("Waiting for desires...")
    	while True:
    		sit = incoming_desires.get()
    		
    		if sit:
    			desire = desires.desire_factory(sit, pr2)
    			desire.perform()
    		time.sleep(0.1)
    except KeyboardInterrupt:
    	pass
    
