import time

import logging; logger = logging.getLogger("robot." + __name__)

from robots.lowlevel import *
from robots.action import *

class StateManager(object):

    def __init__(self, robot):
        self.robot = robot
 
    @helper("state")
    def isseen(self, obj):
        """ Returns true if an object is currently seen by the
        robot.

        The exact semantics of "seen" depend on the robot and the
        type of object (human, artifact...)

        The general method relies on the knowledge base, but robot-specific
        state managers may override this behaviour.

        :param object: the ID of the object to check
        :returns: true if the object is visible, false otherwise.
        """
        if self.robot.knowledge:
            return "%s isVisible true" % obj in self.robot.knowledge

        raise NotImplementedError("isseen requires a knowledge base to be available")

    @helper("state")
    def getjoint(self, name):
        """ Returns the value (in radians) of a given joint (1 DoF) angle
        """
        raise NotImplementedError

    @helper("state")
    def getpose(self):
        """ Get the full pose of the robot, as a dict.

        The list of possible parts is:
        {"HEAD":[pan, tilt],
         "TORSO":[height or bend],
         "RARM":[n dofs starting at shoulder],
         "LARM":[n dofs starting at shoulder]
         }

         Depending on the robot, some part may ne be available.
        """
        raise NotImplementedError

    @helper("state")
    def fingerpressed(self, gripper = "right"):
        """ Returns true if the (by default, right) gripper is 'holding' something.

        'holding' semantics may be as simple as 'gripper closed' or more elaborate
        like 'the pressure sensors detect something'.

        Raises NotImplementedError if this is not supported by the robot.

        :param gripper: the gripper, either 'left' or 'right'. By default, right.
        """
        raise NotImplementedError

    @helper("state")
    def distance2obstacle(self):
        """ Returns the distance, in meters, to the closest obstacle facing the robot.
        """
        raise NotImplementedError

class PR2StateManager(StateManager):
    """ This class helps with the state of the PR2 robot.

    Currently provides:
     * getjoint(name) to get the value of a given joint
     * distance2obstacle() that returns the distance in meter to the closest obstacle in
     front of the robot.
     """

    def __init__(self, robot):
        super(PR2StateManager, self).__init__(robot)

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
        
        if self.robot.supports(ROS):
            import rospy
            from sensor_msgs.msg import JointState
            
            data = rospy.wait_for_message("/joint_states", JointState)
            idx = data.name.index(name)
            return data.position[idx]
        else:
            return 0.0

    @helper("state")
    def getpose(self):
        try:
            import rospy
            from sensor_msgs.msg import JointState
            logger.debug("Reading PR2 pose... (10 sec timeout)")
            msg = rospy.client.wait_for_message('/joint_states', JointState, timeout = 10)
        except rospy.exceptions.ROSException:
            logger.error("Could not read topic /joint_states. Right ROS_MASTER_URI? "
            "PR2 started?")
            return None

        raw = msg.position

        pose = {}
        pose["HEAD"] = [round(raw[14],3), round(raw[15],3)] # head_pan_joint, head_tilt_joint
        pose["TORSO"] = [round(raw[12], 3)] # torso_lift_joint

        # Arm joints order:
        # shoulder_pan_joint
        # shoulder_lift_joint
        # upper_arm_roll_joint 
        # elbow_flex_joint
        # forearm_roll_joint
        # wrist_flex_joint
        # wrist_roll_joint

        pose["RARM"] = [round(raw[18], 3), round(raw[19], 3), round(raw[17], 3), round(raw[21], 3), round(raw[20], 3), round(raw[22], 3), round(raw[23], 3), ]
        pose["LARM"] = [round(raw[32], 3), round(raw[33], 3), round(raw[31], 3), round(raw[35], 3), round(raw[34], 3), round(raw[36], 3), round(raw[37], 3), ]
        return pose

    @helper("state")
    def fingerpressed(self, gripper = "right"):
        """ Returns true if the pression sensors of the gripper (right by
        default) detect something.

        Warning: no real calibration made! This only takes one pressure sensor
        as input, and is based on experimental testing.
        """
        
        import roslib; roslib.load_manifest('pyrobots_actionlib')
        import rospy
        from pr2_msgs.msg import PressureState
        
        data = rospy.wait_for_message("/pressure/" + gripper[0] + "_gripper_motor", PressureState)
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


class NaoStateManager(StateManager):

    def __init__(self, robot):
        super(NaoStateManager, self).__init__(robot)


    @helper("state")
    def hasinhand(self, hand="right"):

        if not self.robot.knowledge:
            raise Exception("nao.state.hasinhand requires a running knowledge base")

        max_distance_from_hand = 0.2 #m

        frame = "r_gripper" if hand == "right" else "l_gripper"

        # 1- look at the hand
        self.robot.look_at([0,0,0.1,frame])
        # 2- get all the visible objets from the knowledge base
        self.robot.wait(2)
        visibles = self.robot.knowledge["* isVisible true"]
        logger.debug("Visible objects: %s" % str(visibles))

        # 3- keep only objects that are actually close to the hand
        inhand = []
        for obj in visibles:
            pose = self.robot.poses.ros.inframe(obj, frame)
            dist = pose['x']**2 + pose['y']**2 + pose['z']**2
            if dist < max_distance_from_hand**2:
                inhand += obj

        logger.debug("Objects in hand: %s" % str(inhand))
        return inhand

    @helper("state")
    def getjoint(self, joint):
        """ Returns the value (in radians) of a given joint (1 DoF) angle
        """
        ok, res = self.robot.execute([
            naoqi_request("motion", "getAngles", [joint, True])
        ])
        return res[0]

    @helper("state")
    def getpose(self):
        """Returns to current whole-body joint state of the robot.
        """

        joint = self.getjoint
        pose = {}
        pose["HEAD"] = [round(joint('HeadYaw'),3), -round(joint('HeadPitch'),3)] # head_pan_joint, head_tilt_joint
        pose["TORSO"] = [round(joint('RHipPitch'), 3)] # torso pitch is set at hips.
        pose["LARM"] = [round(joint('LShoulderPitch'), 3), round(joint('LShoulderRoll'), 3), round(joint('LElbowYaw'), 3), round(joint('LElbowRoll'), 3), round(joint('LWristYaw'), 3)]
        pose["RARM"] = [round(joint('RShoulderPitch'), 3), round(joint('RShoulderRoll'), 3), round(joint('RElbowYaw'), 3), round(joint('RElbowRoll'), 3), round(joint('RWristYaw'), 3)]
        return pose


    @helper("state")
    def fingerpressed(self, gripper = "right"):
        """ For Nao, only returns if the fingers are (firmly) closed or not.

        :param gripper: the gripper, either 'left' or 'right'. By default, right.
        """
        joint = "LHand" if gripper == "left" else "RHand"

        ok, angle = self.robot.execute([
            naoqi_request("motion", "getAngles", [joint, True])
        ])
 
        ok, stiff = self.robot.execute([
            naoqi_request("motion", "getStiffnesses", [joint, True])
        ])
        return (angle[0] < 0.7 and stiff[0] > 0.8)

 
    @helper("state")
    def distance2obstacle(self):
        """ Returns the distance, in meters, to the closest obstacle facing the robot.
        """
        raise NotImplementedError


