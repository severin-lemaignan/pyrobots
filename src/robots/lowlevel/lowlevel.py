import logging; logger = logging.getLogger("robot.lowlevel")

import weakref
import pkgutil, sys
from functools import partial

from ranger.helpers.helpers import valuefilter, enum
from ranger.helpers.position import PoseManager
from ranger.introspection import introspection
from ranger.events import Events
from ranger.robot_actions import RobotActionExecutor


_robot = None
def get_robot(dummy = False, immediate = False):
    """ Use this function to retrieve the (singleton) low-level
    robot accessor.

    :param immediate: (default: False) in immediate mode, robot's action are not
    asynchronous. Useful for debugging for instance.
    """
    global _robot
    if not _robot:
        _robot = RobotLowLevel(dummy, immediate)
    return _robot




class RobotLowLevel(object):
    """
    This class manages functionalities that are shared across every robot 'backends' (ROS, Aseba,...)

    Its main features are:
    - automatic addition of proxy methods for the robot actions
    - pose management through the 'robot.pose' member
    - event monitoring through the 'robot.on(...).do(...)' interface
    - support for introspection (cf README.md for details)
    """

    def __init__(self, actions = None, immediate = False):
        """
        :param list actions: a list of packages that contains modules with
        actions (ie, modules with functions decorated with @action). Proxies to
        these actions are appended to the instance of RobotLowLevel upon
        construction.
        :param boolean immediate: if True (default to False), actions are
        executed in the main thread instead of their own separate threads.
        Useful for some specific debugging scenarios.

        """

        self.executor = RobotActionExecutor()

        self.immediate = immediate

        self._filteredvalues = {} # holds the filters for sensors that need filtering (like scale, IR sensors...)

        self.pose = PoseManager(self)

        self.events = Events(self)
        # make the 'Events.on(...)' method available at robot level
        self.on = self.events.on

        # Dynamically add available actions (ie, actions defined with @action in
        # actions/* submodules.
        if not actions:
            logger.error("No action packages specified when creating an instance of RobotLowLevel. Likely an error!")

        for action in self._available_actions(actions):
            setattr(action, "_executor", weakref.ref(self.executor)) # add a reference to the robot's action executor
            setattr(self, action.__name__, partial(action, self))
            logger.info("Added " + action.__name__ + " as available action.")

        if introspection:
            introspection.ping()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        self.events.close()

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
            path = sys.modules[pkg].__path__
            for loader, module_name, is_pkg in  pkgutil.walk_packages(path):
                m = sys.modules[pkg + "." + module_name]
                for member in [getattr(m, fn) for fn in dir(m)]:
                    if hasattr(member, "_action"):
                        actions.append(member)

        return actions

