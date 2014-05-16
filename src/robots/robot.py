import logging; logger = logging.getLogger("robots.robot")

import time
import weakref
import pkgutil, sys
from functools import partial

from robots.helpers.helpers import valuefilter, enum
from robots.helpers.position import PoseManager
from robots.introspection import introspection
from robots.events import Events
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
    """

    def __init__(self, actions = None, dummy = False, immediate = False):
        """
        :param list actions: a list of packages that contains modules with
        actions (ie, modules with functions decorated with @action). Proxies to
        these actions are appended to the instance of RobotLowLevel upon
        construction.
        
        :param boolean dummy: if True (defults to False), the robot is in
        'dummy' mode: no actual actions are performed. The exact meaning of
        'dummy' is left to the subclasses of GenericRobot.
        :param boolean immediate: if True (default to False), actions are
        executed in the main thread instead of their own separate threads.
        Useful for some specific debugging scenarios.

        """

        # initially, empty state (a state is actually a simple dictionary, with
        # direct member accessors). Users are expected to override this member
        self.state = State()

        self.executor = RobotActionExecutor()

        self.dummy = dummy

        self.immediate = immediate

        self._filteredvalues = {} # holds the filters for sensors that need filtering (like scale, IR sensors...)

        self.pose = PoseManager(self)

        self.events = Events(self)
        # make the 'Events.on(...)' method available at robot level
        self.on = self.events.on

        # Dynamically add available actions (ie, actions defined with @action in
        # actions/* submodules.
        self.load_actions(actions)

        if introspection:
            introspection.ping()

    def loglevel(self, level = logging.INFO):
        logging.getLogger("robots").setLevel(level)

    def running(self):
        """ Print the list of running tasks.
        """
        logger.info(str(self.executor))

    def taskinfo(self, id):
        """ Print the list of running tasks.
        """
        logger.info(self.executor.taskinfo(id))


    def load_actions(self, actions):
        if not actions:
            logger.error("No action packages specified when creating an instance of RobotLowLevel. Likely an error!")

        else:
            for action in self._available_actions(actions):
                setattr(self, action.__name__, partial(action, self))
                logger.info("Added " + action.__name__ + " as available action.")


    def wait_for_state_update(self, timeout = None):
        raise NotImplementedError()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        self.events.close()

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

        Actions that are not yet started (eg, actions waiting on a resource
        availability) are simply removed for the run queue.

        """
        self.executor.cancel_all()

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

