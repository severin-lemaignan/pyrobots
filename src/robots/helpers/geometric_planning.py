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
    
    @tested("14/06/2012")
    @helper("planning")
    def handover(self, human = "HERAKLES_HUMAN1", standing = True, mobility = 0.0):
        """ Computes a set of waypoints that allow the robot to reach a position 
        suitable to transfer an object to a human.

        :param human: the name of the human that must be reached, as
        known by SPARK.
        :param standing: if True, the human is considered as standing, if False
        as sitting.
        :param mobility: ratio of displacement between the human and the robot.
        At 0.0, the human does not move. At 1.0, human and robot do approximately
        the same amount of displacement.

        :return: a list with two items:
          * waypoints: the list of pyRobots poses (actually, a list of (x,y,theta)).
          * pose: the body pose (right arm + torso) of the robot as a dictionary
          (cf setpose documentation).
        """

        raw = self.robot.execute([genom_request("mhp", "ComputeObjectTransferRos", [human, standing, mobility])])

        ok, res = raw
        if not ok:
            return None
        nbPoints = int(res[0])
        points = map(float, res[2:-8])
        right_hand_dofs = map(float, res[-8:])

        def makeposedict(x,y,rz):
            return (x, y, 0, 0, 0, rz)

        wps = map(makeposedict, points[::3], points[1::3], points[2::3])
        pose = {'RARM': right_hand_dofs[:7],
                'TORSO': [right_hand_dofs[7]]}

        return (wps[:nbPoints], pose)


