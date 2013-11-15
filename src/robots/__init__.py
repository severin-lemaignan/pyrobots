import sys
import pkgutil
import time

import logging; robotlog = logging.getLogger("robot." + __name__)

class NullHandler(logging.Handler):
    """Defines a NullHandler for logging, in case pyrobots is used in an 
    application that doesn't use logging.
    """
    def emit(self, record):
        pass

h = NullHandler()
robotlog.addHandler(h)

from exception import RobotError

from helpers.position import PoseManager
from helpers import postures
from helpers.geometric_planning import PlanningManager
from helpers.state import PR2StateManager

from robots.lowlevel import *
from robots.action import RobotAction

class Robot(object):
    """ This 'low-level' class implements all what is required to actually execute
    actions on the robot.
    
    It supports execution of ROS actions, Pocolibs (genom) requests, Python code and some
    special actions like 'wait'.
    
    :param knowledge: (default: None) pass here an instance of a pyoro.Oro 
    object to connect to a knowledge base.
    :param supports: (default: 0) a mask of middlewares the robot supports. For
    example 'supports = ROS|POCOLIBS' means that both ROS and Pocolibs are
    supported. This requires the corresponding Python bindings to be available.
    :param dummy: (default: False) If true, no actual middleware is used. All
    actions exectued are simply reported to the logger.
    """

    def __init__(self, host = None, port = None, supports = 0, knowledge = None, dummy = False):

        self.knowledge = knowledge 
        self.dummy = dummy or not supports
        if self.dummy:
            robotlog.warn("Running in 'dummy' mode. No actual action may be carried.")
        self.mw = supports

        self.invalid_context = False # when true, all tasks are skipped. Used to cancel an action, for instance

        self._pending_python_requests = {}

        if self.supports(POCOLIBS):
            from lowlevel.pocolibs import PocolibsActions
            self.pocoactions = PocolibsActions(host, port)

        if self.supports(ROS):
            from lowlevel.ros import ROSActions
            self.rosactions = ROSActions()
            # Using ROS: automatically configure the logging to use
            # ROS RX Console
            import roslogger
            rxconsole = roslogger.RXConsoleHandler()
            logging.getLogger("robot").addHandler(rxconsole)


        # Import all modules under robots/actions/
        import actions
        path = sys.modules['robots.actions'].__path__
        for loader, module_name, is_pkg in  pkgutil.walk_packages(path):
            __import__('robots.actions.' + module_name)

        # Dynamically add available actions (ie, actions defined with @action in
        # actions/* submodules.
        for action in self.available_actions():
            self.add_action(action)

        self.poses = PoseManager(self)
        self.planning = PlanningManager(self)

    def supports(self, middleware):
        return bool(self.mw & middleware) and not self.dummy
    
    def hasmodule(self, module):
        if self.supports(POCOLIBS):
            return pocoactions.hasmodule(module)
        else:
            return False
      
    def __enter__(self):
	return self

    def __exit__(self, toto, tata, titi):
        self.close()

    def close(self):
        robotlog.info('Closing pyRobots...')

        if self.knowledge:
            robotlog.info('Closing the knowledge base')
            self.knowledge.close()
            self.knowledge = None

        if self.supports(POCOLIBS):
            self.pocoactions.close()

        if self.supports(ROS):
            self.rosactions.close()

        robotlog.info('pyRobots properly closed. Bye bye!')

    def available_actions(self):
        """ Iterate over all loaded modules, and retrieve actions (ie functions
        with the @action decorator).
        """
        actions = []
        path = sys.modules["robots.actions"].__path__
        for loader, module_name, is_pkg in  pkgutil.walk_packages(path):
            m = sys.modules["robots.actions." + module_name]
            for member in [getattr(m, fn) for fn in dir(m)]:
                if hasattr(member, "_action"):
                    actions.append(RobotAction(member, m))
        return actions
    
    def add_action(self, action):
        def innermethod(*args, **kwargs):
            if action.broken():
                ok = raw_input("Attention! " + str(action) + " is marked as broken."
                               "Are you sure you want to proceed? (y/N)")
                if ok != 'y':
                    return
            robotlog.debug("Calling action " + str(action))
            actions = action.fn(self, *args, **kwargs)
            return self.execute(actions)
                
        innermethod.__doc__ = action.fn.__doc__ if action.fn.__doc__ else action.name + \
            "\nThis method has been dynamically added to your robot."
        innermethod.__name__ = action.name
        setattr(self,innermethod.__name__,innermethod)

        def get_actions_innermethod(*args, **kwargs):
            robotlog.debug("Returning action for " + str(action))
            actions = action.fn(self, *args, **kwargs)
            return actions
                
        get_actions_innermethod.__doc__ = action.fn.__doc__ if action.fn.__doc__ else action.name + \
            "\nThis method has been dynamically added to your robot."
        get_actions_innermethod.__name__ = "getactions" + action.name
        setattr(self,get_actions_innermethod.__name__,get_actions_innermethod)

    def _ack(self, evt):
        """ NOP function that serves as fallback callback
        """
        #TODO: We should here remove the corresponding entry in pocolibs_pending_request.
        #To do so, we need to construct special callbacks that unset the correct entry.
        pass

    def _execute_pocolibs(self, action):

        if not self.supports(POCOLIBS):
            raise RobotError("This action '" + action["request"] + "' "
                     "requires Pocolibs, but this ActionPerformer "
                     "has been started without it.")

        return self.pocoactions.execute(action)

    def _execute_ros(self, action):

        if not self.supports(ROS):
            raise RobotError("This action '" + action["request"] + "' "
                     "requires ROS, but this ActionPerformer "
                     "has been started without it.")

        return self.rosactions.execute(action)


    def _execute_python(self, action):
        
        return action["functor"](self, *action["args"])

    def _execute_background(self, action):
        
        if action["abort"]:
            try:
                self._pending_python_requests[action["class"]].stop()
                robotlog.warning("Aborted Python background task " + action["class"].__name__)
            except KeyError:
                pass #Likely a task already aborted
            return

        robotlog.debug("Starting Python background task " + action["class"].__name__ + " with params " + str(action["args"]))
        instance = action["class"](self, *action["args"])
        self._pending_python_requests[action["class"]] = instance
        instance.start()

    def cancelall(self):
        self._cancel_all_background_actions()

        if self.supports(POCOLIBS):
            self.pocoactions.cancelall()

        if self.supports(ROS):
            self.rosactions.cancelall()

    def _cancel_all_background_actions(self):
        robotlog.warning("Aborting all background tasks...")
        for action in self._pending_python_requests.values():
            action.stop()
            robotlog.warning(action.__class__.__name__ + " instance aborted.")
        robotlog.warning("Done aborting all background tasks.")

    def _execute_knowledge(self, action):
        if action["action"] == "add":
            robotlog.debug("Adding facts to the knowledge base: " + str(action["args"]))
            self.knowledge.add(action["args"], action["memory_profile"])

        if action["action"] == "retract":
            robotlog.debug("Removing facts to the knowledge base: " + str(action["args"]))
            self.knowledge -= action["args"]
       
    def _execute_special(self, action):
        if action["action"] == "wait":
            robotlog.info("Waiting for " + str(action["args"]))
            time.sleep(action["args"])
    
    def execute(self, actions):
    
        if self.invalid_context:
            robotlog.info("Invalid context: probably canceling a desire")
            return (False, "Context is invalid")

        if self.dummy:
            robotlog.info("#Dummy mode# Executing actions " + str(actions))
            return None
            
        result = (True, None) # by default, assume everything went fine with no return value
        if actions:
            for action in actions:
                robotlog.info(action["name"] + " (" + action["middleware"] + " action) started.")
                robotlog.debug(action["name"] + " details: " + str(action))
                if action["middleware"] == "pocolibs":
                    ok, res = self._execute_pocolibs(action)
                    if not ok:
                        robotlog.warning(action["name"] + " failed with message: " + str(res))
                        return (False, res)
                    if res:
                        result = (ok, res)
                elif action["middleware"] == "ros":
                    res = self._execute_ros(action)
                    if res:
                        result = res
                elif action["middleware"] == "background":
                    res = self._execute_background(action)
                    if res:
                        result = res
                elif action["middleware"] == "python":
                    ok, res = self._execute_python(action)
                    if not ok:
                        robotlog.warning(action["name"] + " failed with message: " + str(res))
                        return (False, res)
                    if res:
                        result = (ok, res)
                elif action["middleware"] == "knowledge":
                    res = self._execute_knowledge(action)
                    if res:
                        result = res
                elif action["middleware"] == "special":
                    res = self._execute_special(action)
                    if res:
                        result = res
                else:
                    robotlog.error("Unsupported middleware. Skipping the action.")

        return result
        
class PR2(Robot):
    def __init__(self, knowledge = None, dummy = False, init = False):
        super(self.__class__,self).__init__(['pr2c2'], 9472, supports = ROS | POCOLIBS, knowledge = knowledge, dummy = dummy)
        robotlog.info("PR2 actions loaded.")

        self.id = "PR2_ROBOT"
        self.postures = postures.read("pr2_postures.json")

        self.state = PR2StateManager(self)

        if init:
            robotlog.info("Initializing modules...")
            self.init()
            robotlog.info("Initialization done.")

class JidoSimu(Robot):
    def __init__(self, knowledge = None, dummy = False, init = False, host = "joyce"):
        super(self.__class__,self).__init__([(host, 9472), (host, 9473)], port = None, supports= POCOLIBS, knowledge = knowledge, dummy = dummy)
        robotlog.info("Action loaded for Jido on MORSE simulator.")

        self.id = "JIDO_ROBOT"
        self.postures = postures.read("jido_postures.json")

        if init:
            robotlog.info("Initializing modules...")
            self.init()
            robotlog.info("Initialization done.")


import __main__ as main
if not hasattr(main, '__file__'):
    # Running in interactive mode:
    # Add a console logger
    from robots.helpers.ansistrm import ColorizingStreamHandler
    robotlog.setLevel(logging.DEBUG)
    console = ColorizingStreamHandler()
    formatter = logging.Formatter('%(name)s: %(message)s')
    console.setFormatter(formatter)
    robotlog.addHandler(console)
