from robots.action import action, broken
from configuration import setpose

@broken
@action
def translation(robot, x, y = 0):
    """Move the robot base in XY, relatively to its current position.
    
    Be careful: this low-level action does NO obstacle avoidance.

    Currently broken: the GotoQ request does not behave as expected.

    :param x: X (forward/backward) displacement, in meter
    :param y: (default: 0) Y (left/right) displacement, in meter
    """
    
    return setpose(robot, {'BASE':[x,y,0,0,0,0]}, relative = True)


@broken
@action
def rotation(robot, theta):
    """Get the robot to turn on itself of the given angle
    (relatively to its current heading).
    
    Be careful: this low-level action does NO obstacle avoidance.

    Currently broken: the GotoQ request does not behave as expected.
    
    :param theta: an angle in radian
    """

    return setpose(robot, {'BASE':[0,0,0,0,0,theta]}, relative = True)

