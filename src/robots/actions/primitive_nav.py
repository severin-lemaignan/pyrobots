import logging; logger = logging.getLogger("robot." + __name__)

from robots.action import *
from configuration import setpose

def ros_translate(robot, x, y = 0.0, speed = 0.1):
    import math
    import rospy
    from geometry_msgs.msg import Twist

    rate = rospy.Rate(10.0)

    pub = rospy.Publisher('/base_controller/command', Twist)

    base_cmd = Twist()

    distance = math.sqrt(x*x + y*y)

    base_cmd.linear.x = (x/distance) * speed
    base_cmd.linear.y = (y/distance) * speed

    init_pose = robot.poses.myself()

    done = False
    while not done:
        pub.publish(base_cmd)
        rate.sleep()

        dist_moved = robot.poses.distance(init_pose, robot.poses.myself())
        logger.debug("Moved " + str(dist_moved) + "m")

        if (dist_moved > distance):
            done = True

    return (done, None)


@tested("26/06/2012")
@action
def translate(robot, x, y = 0, speed = 0.1):
    """Move the robot base in XY, relatively to its current position.
    
    Be careful: this low-level action does NO obstacle avoidance.

    ROS version is a Python translation of 
    http://www.ros.org/wiki/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

    Currently broken: the GotoQ request does not behave as expected.

    :param x: X (forward/backward) displacement, in meter
    :param y: (default: 0) Y (left/right) displacement, in meter
    :param speed: speed, in m/s. Default: 0.1
    """
    #gotoQ version, broken
    #return setpose(robot, {'BASE':[x,y,0,0,0,0]}, relative = True)
    return [python_request(ros_translate, [x, y, speed])]

@tested("27/06/2012")
@action
def dock(robot, distance = 0.2, max_distance = 0.5):

    def execute_dock(robot):
        if robot.state.distance2obstacle() > max_distance:
            return (False, "No obstacle to dock on")

        return ros_translate(robot, robot.state.distance2obstacle() - distance)

    return [python_request(execute_dock, [])]


@broken
@action
def rotate(robot, theta):
    """Get the robot to turn on itself of the given angle
    (relatively to its current heading).
    
    Be careful: this low-level action does NO obstacle avoidance.

    Currently broken: the GotoQ request does not behave as expected.
    
    :param theta: an angle in radian
    """

    return setpose(robot, {'BASE':[0,0,0,0,0,theta]}, relative = True)

