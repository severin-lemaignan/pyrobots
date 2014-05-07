#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import logging
import unittest
import robots
from robots.decorators import action
from robots import __version__
from robots.signals import ActionCancelled

# a dummy goto action that waits for 1 sec
@action
def goto(robot, target):
    print("Starting to move to %s" % target)
    for i in range(10):
        time.sleep(0.1)
    print("Target reached")

class MyRobot(robots.GenericRobot):

    def __init__(self):
        super(MyRobot, self).__init__(actions=[goto], dummy = True)
        self.state.update({"sensor": 0.})
    

class PyrobotsTests(unittest.TestCase):

    def setUp(self):
        self.robot = MyRobot()

    def tearDown(self):
        pass

    def test_basics(self):
        
        ## Accessing/altering the robot's state
        self.assertEqual(self.robot.state.sensor, 0.)
        self.robot.state.sensor = 1.
        self.assertEqual(self.robot.state.sensor, 1.)

    def test_action(self):
        robot = self.robot
        robot.goto([1,0,0]).wait()

    def test_event_cancelling(self):
        robot = self.robot

        def on_evt1():
            print("Started evt1")
            try:
                robot.goto([1,0,0]).wait()
            except ActionCancelled:
                print("Ok, I interrupt my goto")



        robot.on("sensor", increase = 100).do(on_evt1)

        try:
            time.sleep(0.5)
            raise KeyboardInterrupt()

        except KeyboardInterrupt:
            robot.events.close()
            robot.cancel_all()

def version():
    print("pyRobots tests %s" % __version__)


if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description='Test suite for pyrobots2.')
    parser.add_argument('-v', '--version', action='version',
                       version=version(), help='returns pyrobots version')
    parser.add_argument('-f', '--failfast', action='store_true',
                                help='stops at first failed test')

    args = parser.parse_args()


    logger = logging.getLogger("robots")
    console = logging.StreamHandler()
    logger.setLevel(logging.DEBUG)
    logger.addHandler(console)
    
    unittest.main(failfast=args.failfast)
