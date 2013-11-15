import logging; logger = logging.getLogger("robot." + __name__)

import random

from robots.exception import RobotError

from robots.lowlevel import *
from robots.actions.look_at import sweep
from robots.action import *

###############################################################################


@action
@workswith(ALL)
def satisfied(robot):
    actions = kb_satisfied()
    return actions


@action
@same_requirements_as(sweep)
def sorry(robot, speed = 0.5):
    actions = kb_sorry()
    actions += sweep(robot, 45, speed)
    return actions

def _generate_id():
    sequence = "abcdefghijklmopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
    sample = random.sample(sequence, 5)
    return "".join(sample)


def _send_state(state):


    state_id = _generate_id()
    statements = [state_id + " rdf:type " + state, 
                  "myself experiences " + state_id]

    logger.info("Setting my mood to " + state)
    return add_knowledge(statements, lifespan=10)


def kb_confused():
    return _send_state("ConfusedState")

def kb_satisfied():
    return _send_state("SatisfiedState")

def kb_sorry():
    return _send_state("SorryState")

def kb_happy():
    return _send_state("HappyState")

def kb_angry():
    return _send_state("AngryState")

def kb_sad():
    return _send_state("SadState")


