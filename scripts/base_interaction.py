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

pr2 = robots.PR2()
oro = pyoro.Oro()

incoming_desires = queue.Queue()

def ondesires(e):
	logger.info("Incomig desires:" + str(e))
	for d in e:
		incoming_desires.put(d)


oro.subscribe([human + " desires ?d"], ondesires)

try:
	logger.info("Waiting for desires...")
	while True:
		sit = incoming_desires.get(False)
		
		if sit:
			desire = desires.desire_factory(sit, oro, pr2)
			desire.perform()
		time.sleep(0.1)
except KeyboardInterrupt:
	pass

oro.close()
pr2.close()
