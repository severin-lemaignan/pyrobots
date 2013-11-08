import logging; logger = logging.getLogger("robot." + __name__)

from robots.exception import RobotError

from robots.actions.look_at import sweep
from robots.action import *

###############################################################################


@action
def sorry(robot, speed = 0.5):
    return sweep(robot, 45, speed)
 
