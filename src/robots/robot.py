# coding=utf-8
import logging; logger = logging.getLogger("robots.robot")
if (hasattr(logging, "NullHandler")): # python >= 2.7
	logger.addHandler(logging.NullHandler())

import time
import pkgutil, sys
from functools import partial

from robots.helpers.misc import valuefilter
from robots.poses import PoseManager
from robots.introspection import introspection
from robots.events import Events
from robots.mw import * # ROS, NAOQI...
from robots.concurrency import RobotActionExecutor, ACTIVE_SLEEP_RESOLUTION


class State(dict):
    __getattr__= dict.__getitem__
    __setattr__= dict.__setitem__
    __delattr__= dict.__delitem__

class GenericRobot(object):
    """ This class manages functionalities that are shared across every robot 'backends' (ROS, Aseba,...)

    You are expected to derive your own robot implementation from this class,
    and it is advised to use instances of :class:`GenericRobot` within a context
    manager (ie ``with MyRobot as robot: ...`` construct).

    Its role comprises of:

    - automatic addition of proxy methods for the robot actions
    - actions execution (spawning threads for actions via ``self.executor``
    - pose management through the ``robot.poses`` instance variable
    - event monitoring through the ``robot.on(...).do(...)`` interface

    :class:`GenericRobot` defines several important instance variables,
    documented below.

    :ivar state: the state vector of the robot. By default, a simple dictionary.
      You can overwrite it with a custom object, but it is expected to provide a
      dictionary-like interface.
    :ivar poses: an instance of :class:`.PoseManager`.
    :ivar executor: instance of :class:`.RobotActionExecutor`
      responsible for spawning and starting threads for the robot actions. You
      should not need to access it directly.

    Example of a custom robot:

    .. code:: python
    
        from robots import GenericRobot
    
        class MyRobot(GenericRobot):
    
            def __init__(self):
                super(MyRobot, self).__init__()
    
                # create (and set) one element in the robot's state. Here a bumper.
                # (by default, self.state is a dictionary. You can safely
                # overwrite it with any dict-like object.
                self.state["my_bumper"] = False
    
                # do whatever other initialization you need for your robot
    
            # Implement here all the accessors you need to talk to the robot
            # low-level, like:

            def send_goal(self, pose):
                # move your robot using your favorite middleware
                print("Starting to move towards %s" % pose)
    
            def stop(self):
                # stop your robot using your favorite middleware
                print("Motion stopped")
    
            def whatever_other_lowlevel_method_you_need(self):
                #...
                pass
   
        # create actions
        @action
        def move_forward(robot):
            #...
            pass

        with MyRobot() as robot:
    
            # Turn on DEBUG logging.
            # Shortcut for logging.getLogger("robots").setLevel(logging.DEBUG)
            robot.debug()
    
            # subscribe to events...
            robot.whenever("my_bumper", value = True).do(move_forward)
    
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
    
    .. note:: A note on debugging
    
        Several methods are there to help with debugging:
    
        - :meth:`loglevel`: default to ``INFO``. ``logging.DEBUG`` can be useful.
    
          - :meth:`silent`: alias for ``loglevel(logging.WARNING)``
          - :meth:`info`: alias for ``loglevel(logging.INFO)``
          - :meth:`debug`: alias for ``loglevel(logging.DEBUG)``
    
        - :meth:`running`: prints the list of running tasks (with their IDs)
        - :meth:`actioninfo`: give details on a given action, including the exact line being currently executed
    
    """

    def __init__(self, 
                 actions = None, 
                 supports = 0, 
                 dummy = False, 
                 immediate = False,
                 configure_logging = True):
        """
        :param list actions: a list of packages that contains modules with
          actions (ie, modules with functions decorated with ``@action``). Proxies to
          these actions are appended to the instance of GenericRobot upon
          construction.
        :param supports: (default: 0) a mask of middlewares the robot
          supports. Supported middlewares are listed in
          ``robots.mw.__init__.py``.  For example ``supports = ROS|POCOLIBS``
          means that both ROS and Pocolibs are supported. This requires the
          corresponding Python bindings to be available.
        :param boolean dummy: if ``True`` (defults to ``False``), the robot is in
          'dummy' mode: no actual actions are performed. The exact meaning of
          'dummy' is left to the subclasses of GenericRobot.
        :param boolean immediate: if ``True`` (defaults to ``False``), actions are
          executed in the main thread instead of their own separate threads.
          Useful for some specific debugging scenarios.
        :param boolean configure_logging: if ``True`` (default), configures
          a default colorized console logging handler.
        """

        self.dummy = dummy

        if configure_logging:
            self.configure_console_logging()

        # normally, start logging level for all robots at INFO (instead of
        # Python's default of WARNING)
        if not dummy:
            self.loglevel(logging.INFO)
        else:
            self.loglevel(logging.DEBUG)

        # initially, empty state (a state is actually a simple dictionary, with
        # direct member accessors). Users are expected to override this member
        self.state = State()

        self.executor = RobotActionExecutor()


        self.immediate = immediate

        self._filteredvalues = {} # holds the filters for sensors that need filtering (like scale, IR sensors...)

        self.poses = PoseManager(self)

        self.events = Events(self)
        # make the 'Events.on(...)' method available at robot level
        self.on = self.events.on
        self.whenever = self.events.whenever

        ## Initialization of low-level middlewares
        self.mw = supports
        if self.supports(ROS):
            import sys, os.path
            try:
                import rospy
            except ImportError:
                logger.fatal("ROS support required, but rospy not installed! Quitting.")
                sys.exit(1)
            nodename = os.path.basename(sys.argv[0])
            if not nodename: # robot created from Python REPL
                nodename = "pyrobots_repl"
            else:
                # replace dot (like in 'toto.py') by underscore to be valid ROS id
                nodename = nodename.replace(".","_")
            logger.info("Initializing ROS node <%s>" % nodename)
            rospy.init_node(nodename, disable_signals=True)
            # Using ROS: automatically configure the logging to use
            # ROS RX Console, but first make other handlers quieter
            for i,handler in enumerate(logger.handlers):
                logger.handlers[i].level=logging.WARNING
            import robots.roslogger
            rxconsole = robots.roslogger.RXConsoleHandler()
            logging.getLogger("robot").addHandler(rxconsole)

            from .poses import ROSFrames
            self.rosframes = ROSFrames()
            self.poses.add_frame_provider(self.rosframes)

        # Dynamically add available actions (ie, actions defined with @action in
        # actions/* submodules.
        self.load_actions(actions)

        if introspection:
            introspection.ping()

    def loglevel(self, level = logging.INFO):
        logging.getLogger("robots").setLevel(level)

    def silent(self):
        self.loglevel(logging.WARNING)

    def info(self):
        self.loglevel(logging.INFO)

    def debug(self):
        self.loglevel(logging.DEBUG)


    def running(self):
        """ Print the list of running actions.
        """
        logger.info(str(self.executor))

    def actioninfo(self, id):
        """ Print details on a running action (including the current line
        number).
        """
        logger.info(self.executor.actioninfo(id))

    @staticmethod
    def configure_console_logging():
        from robots.helpers.ansistrm import ConcurrentColorizingStreamHandler

        console = ConcurrentColorizingStreamHandler()
        formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
        console.setFormatter(formatter)
        logging.getLogger("robots").addHandler(console)

    def supports(self, middleware):
        return bool(self.mw & middleware) and not self.dummy

    def load_actions(self, actions):
        if not actions:
            logger.warning("No action packages specified when creating an instance of GenericRobot. Likely an error!")

        else:
            for action in self._available_actions(actions):
                setattr(self, action.__name__, partial(action, self))
                logger.info("Added " + action.__name__ + " as available action.")


    def wait_for_state_update(self, timeout = None):
        """ Blocks until the robot state has been updated.

        This is highly dependent on the low-level mechanisms of your robot, and
        should almost certainly be overriden in your implementation of a
        GenericRobot subclass.

        The default implementation simply waits ``ACTIVE_SLEEP_RESOLUTION``
        seconds.
        """
        time.sleep(ACTIVE_SLEEP_RESOLUTION)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        self.cancel_all()
        self.events.close()

        if self.supports(ROS):
            import rospy
            rospy.signal_shutdown("executive controller closing")


    @staticmethod
    def sleep(duration):
        """ Active sleep. Must used by actions to make sure they can be quickly
        cancelled.
        """

        tot_time = 0
        while tot_time // duration == 0:
            time.sleep(ACTIVE_SLEEP_RESOLUTION)
            tot_time += ACTIVE_SLEEP_RESOLUTION

        time.sleep(duration % ACTIVE_SLEEP_RESOLUTION)

    def wait(self, var, **kwargs):
        """ Alias to wait on a given condition. Cf :class:`robots.events.Events`
        for details on the acceptable conditions.
        """
        self.on(var, **kwargs).wait()

    def cancel_all(self):
        """ Sends a 'cancel' signal (ie, the
        :class:`.ActionCancelled` exception is raised) to all
        running actions.

        Note that, if called within a running action, this action *is cancelled
        as well*. If this is not what you want, use
        :meth:`cancel_all_others` instead.

        Actions that are not yet started (eg, actions waiting on a resource
        availability) are simply removed for the run queue.

        """
        self.executor.cancel_all()

    def cancel_all_others(self):
        """ Sends a 'cancel' signal (ie, the
        :class:`.ActionCancelled` exception is raised) to all
        running actions, *except for the action that call
        :meth:`cancel_all_others`* (note that its currently running subactions
        *will be cancelled*).

        Actions that are not yet started (eg, actions waiting on a resource
        availability) are simply removed for the run queue.

        """
        self.executor.cancel_all_others()


    def filtered(self, name, val):
        """ Helper to easily filter values (uses an accumulator to average a
        given 'name' quantity)

        """

        filter = self._filteredvalues.setdefault(name, valuefilter())
        filter.append(val)
        return filter.get()

    @staticmethod
    def _available_actions(pkgs):
        """ Iterate over all loaded modules, and retrieve actions (ie functions
        with the @action decorator).
        """
        actions = []

        for pkg in pkgs:
            if isinstance(pkg, str):
                if pkg not in sys.modules:
                    try:
                        __import__(pkg)
                    except ImportError:
                        raise RuntimeError("While collecting robot actions, I encountered an unknown module <%s>!" % pkg)

                path = sys.modules[pkg].__path__
                for loader, module_name, is_pkg in  pkgutil.walk_packages(path):
                    __import__(pkg + "." + module_name)
                    m = sys.modules[pkg + "." + module_name]
                    for member in [getattr(m, fn) for fn in dir(m)]:
                        if hasattr(member, "_action"):
                            actions.append(member)
            else:
                # we assume a list of methods has been passed 
                if hasattr(pkg, "_action"):
                    actions.append(pkg)

        return actions

