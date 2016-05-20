pyRobots: a toolkit for robot executive control
===============================================
[![DOI](https://zenodo.org/badge/983/chili-epfl/pyrobots.svg)](https://zenodo.org/badge/latestdoi/983/chili-epfl/pyrobots)
[![Documentation Status](https://readthedocs.org/projects/pyrobots/badge/?version=latest)](http://pyrobots.readthedocs.org)

`pyRobots` provides a set of Python decorators to easily turn standard functions
into background tasks which can be cancelled at anytime and to make your controller
*resource aware* (no, a robot can not turn left AND right at the same time).

It also provides a event-based mechanism to monitor specific conditions and
asynchronously trigger actions.

It finally provides a library of convenient tools to manage poses in a uniform way
(quaternions, Euler angles and 4D matrices, I look at you) and to interface with
existing middlewares (ROS, naoqi, aseba...).

`pyRobots` took some inspiration from the
[URBI](https://github.com/aldebaran/urbi) language.

Installation
------------

```
$ pip install pyRobots
```

(or, of course, from the source)

Main features
-------------

- Turns any Python function into a background *action* with the decorator
  `@action`.
- Robot actions are non-blocking by default: they are instanciated as futures
  (lightweight threads),
- Actions can be cancelled at any time via signals (the `ActionCancelled` signal
  is raised).
- Lock specific resources with a simple `@lock(...)` in front of the actions.
  When starting, actions will wait for resources to be available if needed.
- Supports *compound resources* (like `WHEELS` == `LEFTWHEEL` + `RIGHTWHEEL`)
- Create event with `robot.whenever(<condition>).do(<action>)`
- Poses are managed explicitely and can easily be transformed from one reference
  frame to another one (integrates with ROS TF when available).
- Extensive logging support to debug and replay experiments.

Support for a particular robot only require to subclass `GenericRobot` for this
robot (and, obviously, to code the actions you want your robot to perform).

Documentation
-------------

[Head to readthedocs](http://pyrobots.readthedocs.org). Sparse for now.

Minimum Working Example
-----------------------

...that includes the creation of a specific robot

```python
import time
from robots import GenericRobot
from robots.decorators import action, lock
from robots.resources import Resource
from robots.signals import ActionCancelled

# create a 'lockable' resource for our robot
WHEELS = Resource("wheels")

class MyRobot(GenericRobot):

    def __init__(self):
        super(MyRobot, self).__init__()

        # create (and set) one element in the robot's state. Here a bumper.
        self.state.my_bumper = False

        # do whatever other initialization you need :-)

    def send_goal(self, pose):
        # move your robot using your favorite middleware
        print("Starting to move towards %s" % pose)

    def stop(self):
        # stop your robot using your favorite middleware
        print("Motion stopped")

    def whatever_lowlevel_method_you_need(self):
        pass

@lock(WHEELS)
@action
def move_forward(robot):
    """ We write action in a simple imperative, blocking way.
    """

    # the target pose: simply x += 1.0m in the robot's frame. pyRobots
    # will handle the frames transformations as needed.
    target = [1.0, 0., 0., "base_link"]

    try:
        robot.send_goal(target)

        while(robot.pose.distance(robot.pose.myself(), target) > 0.1):
            # robot.sleep is exactly like time.sleep, except it lets the pyrobots
            # signals pass through.
            robot.sleep(0.5)

        print("Motion succeeded")

    except ActionCancelled:
        # if the action is cancelled, clean up your state
        robot.stop()


with MyRobot() as robot:

    # Turn on DEBUG logging.
    # Shortcut for logging.getLogger("robots").setLevel(logging.DEBUG)
    robot.debug()

    robot.whenever("my_bumper", value = True).do(move_forward)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

```

