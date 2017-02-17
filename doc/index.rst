pyRobots: a toolkit for robot executive control
===============================================

As you may well know if you ever tried to use them to implement under-specified,
dynamic tasks, `state machines <http://wiki.ros.org/smach>`_ are not always the
most convenient tool to code robot controllers.

``pyRobots`` provides a set of Python decorators to easily turn standard
functions into background **asynchronous tasks** which can be **pre-empted at
anytime** and to make your controller **resource-aware** (no, a robot can not
turn left AND right at the same time).

It also provides an **event-based** mechanism to monitor specific conditions and
asynchronously trigger actions.

It finally provides a **library of tools** to manage poses in a uniform way
(quaternions, Euler angles and 4D matrices, I look at you) and to **interface
with existing middlewares** (ROS, naoqi, aseba...). Note that ``pyRobots`` is
**not itself a robotic middleware**.

``pyRobots`` is inspired by the `URBI <https://github.com/urbiforge/urbi>`_
language.

Main features
-------------

-  Turns any Python function into a background *action* with the
   decorator ``@action``;
-  Robot actions are non-blocking by default: they are instanciated as
   futures (lightweight threads);
-  Actions can be pre-empted (cancelled) at any time via signals (the
   :class:`.ActionCancelled` signal is raised):

   .. code-block:: python

        @action
        def safe_walk(robot):
            try:
                robot.walk()
            except ActionCancelled:
                robot.go_back_to_rest_pose()

        action = robot.safe_walk()
        time.sleep(1)
        action.cancel()
-  Create event with ``robot.whenever(<condition>).do(<action>)``;
-  Lock specific resources with a simple ``@lock(...)`` in front of the
   actions. When starting, actions will wait for resources to be
   available if needed:

   .. code-block:: python

        L_ARM = Resource()
        R_ARM = Resource()
        ARMS = CompoundResource(L_ARM, R_ARM)

        @action
        @lock(ARMS)
        def lift_box(robot):
            #...

        @action
        @lock(L_ARM)
        def wave_hand(robot):
            #...

        @action
        @lock(L_ARM, wait=False)
        def scratch_head(robot):
            #...

        robot.lift_box()
        robot.wave_hand() # waits until lift_box is over
        robot.scratch_head() # skipped if lift_box or
                            # wave_hand are still running

-  Supports *compound resources* (like ``WHEELS`` == ``LEFTWHEEL`` +
   ``RIGHTWHEEL``);
-  Poses are managed explicitely and can easily be transformed from one
   reference frame to another one (integrates with ROS TF when
   available);
-  Extensive logging support to debug and replay experiments.

Support for a particular robot only require to subclass :class:`.GenericRobot`
for this robot (and, obviously, to code the actions you want your robot to
perform).

:doc:`Combined with a knowledge base <kb>`, ``pyRobots`` makes an interesting
starting point for a high-level cognitive controller for robots.

Code Documentation
------------------

The documentation is currently sparse. Please fill `bug reports
<https://github.com/chili-epfl/pyrobots/issues>`_ everytime you can not figure
out a specific bit.

Main entry points
+++++++++++++++++

- :py:class:`robots.robot.GenericRobot`


Full package documentation
++++++++++++++++++++++++++

.. toctree::
    :maxdepth: 2
    
    api/robots.rst


Minimum Working Example
-----------------------

...that includes the creation of a specific robot

.. code:: python

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

Indices and tables
------------------


* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

