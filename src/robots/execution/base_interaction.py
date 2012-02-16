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
import pyoro
import robots
import Queue as queue

import desires

human = "HERAKLES_HUMAN1"

pr2 = robots.PR2()
oro = pyoro.Oro()

incoming_desires = queue.Queue()

def ondesires(e):
	print("Incomig desires:" + str(e))
	for d in e:
		incoming_desires.put(d)


oro.subscribe([human + " desires ?d"], ondesires)

try:
	print("Waiting for desires...")
	while True:
		desire = desires.desire_factory(incoming_desires.get(), oro, pr2)
		desire.perform()
except KeyboardInterrupt:
	oro.close()
	pr2.close()
