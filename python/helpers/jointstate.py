import roslib; roslib.load_manifest('novela_actionlib')
import rospy
from sensor_msgs.msg import JointState

def getjoint(name):
        data = rospy.wait_for_message("/joint_states", JointState)
        idx = data.name.index(name)
        return data.position[idx]

