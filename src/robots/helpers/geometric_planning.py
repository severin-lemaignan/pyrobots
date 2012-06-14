import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.action import *
from robots.exception import RobotError


class PlanningManager:
    """ This class helps with geometric planning related tasks:

    Currently only:
     * compute a path for the object hand-over between human and robot
     """

    def __init__(self, robot):
        self.robot = robot
    
    def gethandoverwaypoints(self, human = "HERAKLES_HUMAN1", standing = True, object_necessity = 0.0):

        raw = robot.execute([genom_request("mhp", "ComputeObjectTransferRos", [human, standing, object_necessity])])
        return _process_result(raw)

    def _process_result(self, raw):
        
        ok, res = raw
        if ok != 'OK':
            return None
        yaw, pitch, roll, x, y, z = [float(x) for x in res]

        return (x, y, z, roll, pitch, yaw)

