from configuration import setpose

@action
def translation(robot, x, y = 0):
    """Move the robot base in XY, relatively to its current position.
    
    Be careful: this low-level action does NO obstacle avoidance.

    :param x: X (forward/backward) displacement, in meter
    :param y: (default: 0) Y (left/right) displacement, in meter
    """
    
    return setpose({'BASE':[x,y,0,0,0,0]}, relative = True)


@action
def rotation(robot, theta):
    """Get the robot to turn on itself of the given angle
    (relatively to its current heading).
    
    Be careful: this low-level action does NO obstacle avoidance.
    
    :param theta: an angle in radian
    """

    return setpose({'BASE':[0,0,0,0,0,theta]}, relative = True)

