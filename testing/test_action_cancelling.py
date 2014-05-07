#! /usr/bin/env python
import logging; logging.basicConfig()

import time

from robots import GenericRobot
from robots.signals import ActionCancelled

with GenericRobot() as robot:

    def on_evt1():
        print("Started evt1")
        try:
            robot.goto([1,0,0]).wait()
        except ActionCancelled:
            print("Ok, I interrupt my goto")



    robot.on("scale", increase = 100).do(on_evt1)

    try:
        while True:
            time.sleep(0.5)
            print("Press ctrl+C to stop.")

    except KeyboardInterrupt:
        robot.events.close()
        robot.cancel_all()

print("Closed gracefully")
