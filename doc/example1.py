#! /usr/bin/env python


## Setting up logging

import logging
logger = logging.getLogger("robot")
logger.setLevel(logging.DEBUG)

console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)

logger.addHandler(console)


from robots import Robot




with Robot(dummy = True) as robot:

    robot.look_at("LArm")
    robot.manipose()
