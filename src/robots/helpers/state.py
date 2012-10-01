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
    def getjoint(self, name):
        
        import roslib; roslib.load_manifest('pyrobots_actionlib')
        import rospy
        from sensor_msgs.msg import JointState
        
        data = rospy.wait_for_message("/joint_states", JointState)
        idx = data.name.index(name)
        return data.position[idx]

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
        if data.l_finger_tip[11] > 4000 or data.r_finger_tip[11] > 4000:
            return True
        else:
            return False

    @helper("state")
    def distance2obstacle(self):
        
        import roslib; roslib.load_manifest('pyrobots_actionlib')
        import rospy
        from sensor_msgs.msg import LaserScan
        
        data = rospy.wait_for_message("/base_scan", LaserScan)

        nb_scans = int((data.angle_max - data.angle_min) / data.angle_increment)

        return data.ranges[nb_scans / 2]

