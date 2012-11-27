import time

import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.action import *

class PR2StateManager:
    """ This class helps with the state of the PR2 robot.

    Currently provides:
     * getjoint(name) to get the value of a given joint
     * distance2obstacle() that returns the distance in meter to the closest obstacle in
     front of the robot.
     """

    def __init__(self, robot):
        self.robot = robot

    @helper("state")
    def isseen(self, obj):
        """ Returns true if an object is currently seen by the
        robot.

        The semantics of "seen" can vary: in the current implementation,
        it means that the 2D ARToolkit tag has been seen in the
        last 2 seconds.

        :param object: the object (SPARK ID) to check
        :returns: true if the object is visible, false otherwise.
        """
        if not obj:
            return False

        try:
            spark = self.robot.poco_modules["spark"]
        except KeyError:
            logger.warning("'isseen' requires 'spark' Genom module, but it"
                           " does not seem to be started. Assuming object"
                           "not visible.")
            return False

        state = spark.poster("PositionsInfopositionsStatus")

        lastseen = -1
        for i in range(len(state)):
            if state[i] == obj:
                lastseen = int(state[i+5])

        if lastseen == -1:
            raise Exception("Object " + obj + " is not a special SPARK object (cf hri_knowledge.cpp:117. Argh!)")

        if lastseen == 0:
            return False
        else:
            seenago = int(time.time()) - lastseen
            logger.debug("%s last seen %dsec ago" % (obj, seenago))
            if seenago < 2:
                return True
            return False



    @helper("state")
    def getjoint(self, name):
        
        if self.robot.hasROS():
            import rospy
            from sensor_msgs.msg import JointState
            
            data = rospy.wait_for_message("/joint_states", JointState)
            idx = data.name.index(name)
            return data.position[idx]
        else:
            return 0.0

    @helper("state")
    def fingerpressed(self, hand = "right"):
        """ Returns true if the pression sensors of the gripper (right by
        default) detect something.

        Warning: no real calibration made! This only takes one pressure sensor
        as input, and is based on experimental testing.
        """
        
        import roslib; roslib.load_manifest('pyrobots_actionlib')
        import rospy
        from pr2_msgs.msg import PressureState
        
        data = rospy.wait_for_message("/pressure/" + hand[0] + "_gripper_motor", PressureState)
        av1 = sum(data.l_finger_tip) / len(data.l_finger_tip)
        av2 = sum(data.r_finger_tip) / len(data.r_finger_tip)
        av = (av1 + av2) / 2
        if av > 3500:
            return True
        else:
            return False

    @helper("state")
    def distance2obstacle(self):
        
        import rospy
        from sensor_msgs.msg import LaserScan
        
        data = rospy.wait_for_message("/base_scan", LaserScan)

        nb_scans = int((data.angle_max - data.angle_min) / data.angle_increment)

        return data.ranges[nb_scans / 2]

