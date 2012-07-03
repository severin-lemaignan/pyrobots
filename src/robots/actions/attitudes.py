import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.exception import RobotError

from robots.actions.look_at import sweep_look
from robots.action import *

###############################################################################


@action
def sorry(robot, speed = 0.5):
    return sweep_look(robot, 45, speed)
 
