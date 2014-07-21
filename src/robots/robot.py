
import logging; logger = logging.getLogger("robots.robot")
logger.addHandler(logging.NullHandler())

import time
import weakref
import pkgutil, sys
from functools import partial

from robots.helpers.helpers import valuefilter, enum
from robots.helpers.position import PoseManager
from robots.introspection import introspection
from robots.events import Events
from robots.mw import * # ROS, NAOQI...
from robots.robot_actions import RobotActionExecutor, ACTIVE_SLEEP_RESOLUTION


class State(dict):
    __getattr__= dict.__getitem__
    __setattr__= dict.__setitem__
    __delattr__= dict.__delitem__

class GenericRobot(object):
    """
    This class manages functionalities that are shared across every robot 'backends' (ROS, Aseba,...)

    Its main features are:
    - automatic addition of proxy methods for the robot actions
    - pose management through the 'robot.pose' member
    - event monitoring through the 'robot.on(...).do(...)' interface
    - support for introspection (cf README.md for details)


    Main helpers for debugging:
    - GenericRobot.loglevel(<logging level>): default to INFO. logging.DEBUG can be useful.
        - GenericRobot.silent(): alias for GenericRobot.loglevel(logging.WARNING)
        - GenericRobot.info(): alias for GenericRobot.loglevel(logging.INFO)
        - GenericRobot.debug(): alias for GenericRobot.loglevel(logging.DEBUG)
    - GenericRobot.running(): prints the list of running tasks (with their IDs)
    - GenericRobot.taskinfo(<task id>): give details on a given task, including the exact line being currently executed
 """

    def __init__(self, 
                 actions = None, 
                 supports = 0, 
                 dummy = False, 
                 immediate = False,
                 configure_logging = True):
        """
        :param list actions: a list of packages that contains modules with
        actions (ie, modules with functions decorated with @action). Proxies to
        these actions are appended to the instance of GenericRobot upon
        construction.
        :param supports: (default: 0) a mask of middlewares the robot
        supports. Supported middlewares are listed in
        robots.mw.__init__.py.  For example 'supports = ROS|POCOLIBS'
        means that both ROS and Pocolibs are supported. This requires the
        corresponding Python bindings to be available.
        :param boolean dummy: if True (defults to False), the robot is in
        'dummy' mode: no actual actions are performed. The exact meaning of
        'dummy' is left to the subclasses of GenericRobot.
        :param boolean immediate: if True (default to False), actions are
        executed in the main thread instead of their own separate threads.
        Useful for some specific debugging scenarios.
        :paran boolean configure_logging: if True (default), configures
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

        self.pose = PoseManager(self)

        self.events = Events(self)
        # make the 'Events.on(...)' method available at robot level
        self.on = self.events.on
        self.every = self.events.every

        ## Initialization of low-level middlewares
        self.mw = supports
        if self.supports(ROS):
            from mw.ros import ROSActions
            self.rosactions = ROSActions()
            # Using ROS: automatically configure the logging to use
            # ROS RX Console, but first make other handlers quieter
            for i,handler in enumerate(logger.handlers):
                logger.handlers[i].level=logging.WARNING;
            import robots.roslogger
            rxconsole = robots.roslogger.RXConsoleHandler()
            logging.getLogger("robot").addHandler(rxconsole)
                  
        if self.supports(NAOQI):
            from mw._naoqi import NAOqiActions
            self.naoqiactions = NAOqiActions()





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
        """ Print the list of running tasks.
        """
        logger.info(str(self.executor))

    def taskinfo(self, id):
        """ Print the list of running tasks.
        """
        logger.info(self.executor.taskinfo(id))

    def configure_console_logging(self):
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

        The default implementation simply waits `ACTIVE_SLEEP_RESOLUTION`
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
            self.rosactions.close()


    def sleep(self, duration):
        """ Active sleep. Can be used by actions to make sure they can be quickly cancelled.
        """

        tot_time = 0
        while tot_time // duration == 0:
            time.sleep(ACTIVE_SLEEP_RESOLUTION)
            tot_time += ACTIVE_SLEEP_RESOLUTION

        time.sleep(duration % ACTIVE_SLEEP_RESOLUTION)

    def wait(self, var, **kwargs):
        """ Alias to wait on a given condition. Cf EventMonitor for details on
        the acceptable conditions.
        """
        self.on(var, **kwargs).wait()

    def cancel_all(self):
        """ Send a 'cancel' signal (ie, the ActionCancelled exception is raise)
        to all running actions.

        Note that, if called within a running action, this action *is cancelled
        as well*. If this is not what you want, use
        :py:meth:`cancel_all_others` instead.

        Actions that are not yet started (eg, actions waiting on a resource
        availability) are simply removed for the run queue.

        """
        self.executor.cancel_all()

    def cancel_all_others(self):
        """ Send a 'cancel' signal (ie, the ActionCancelled exception is raise)
        to all running actions, *except for the action that call
        'cancel_all_others'* (note that its currently running subactions *will
        be cancelled*).

        Actions that are not yet started (eg, actions waiting on a resource
        availability) are simply removed for the run queue.

        """
        self.executor.cancel_all_others()


    def filtered(self, name, val):
        """ Helper to easily filter values (uses an accumulator to average a given 'name' quantity)
        """

        filter = self._filteredvalues.setdefault(name, valuefilter())
        filter.append(val)
        return filter.get()

    def _available_actions(self, pkgs):
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

