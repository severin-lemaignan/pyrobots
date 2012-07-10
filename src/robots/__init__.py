import sys
import pkgutil
import time

import logging; robotlog = logging.getLogger("robot." + __name__)
robotlog.setLevel(logging.DEBUG)

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
from helpers.geometric_planning import PlanningManager
from helpers.state import PR2StateManager

class Robot(object):
    """ This 'low-level' class implements all what is required to actually execute
    actions on the robot.
    
    It supports execution of ROS actions, Pocolibs (genom) requests, Python code and some
    special actions like 'wait'.
    
    :param knowledge: (default: None) pass here an instance of a pyoro.Oro 
    object to connect to a knowledge base.
    :param dummy: (default: False) If true, no actual middleware is used. All
    actions exectued are simply reported to the logger.
    """
    def __init__(self, host = None, port = None, use_pocolibs = True, use_ros = True, knowledge = None, dummy = False):

        self.knowledge = knowledge 
        self.dummy = dummy
        self.use_pocolibs = use_pocolibs if not dummy else False
        self.servers = {} # holds the list of tclserv servers when using pocolibs
        self.use_ros = use_ros if not dummy else False

        if not dummy:

            if use_pocolibs:
                import pypoco
                from pypoco import PocoRemoteError
                self.servers, self.poco_modules = pypoco.discover(host, port)
                self._pending_pocolibs_requests = {}
                self._pending_python_requests = {}
    
            if use_ros:
                import roslib; roslib.load_manifest('pyrobots_actionlib')
                import rospy
                rospy.init_node('pyrobots')
                import actionlib
                from actionlib_msgs.msg import GoalStatus
                self.GoalStatus = GoalStatus

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

    def hasROS(self):
        return self.use_ros
        
    def hasPocolibs(self):
        return self.use_pocolibs
    
    def hasmodule(self, module):
        return True if module in self.poco_modules else False
      
    def __enter__(self):
	return self

    def __exit__(self, toto, tata, titi):
        self.close()

    def close(self):
        if self.knowledge:
            robotlog.info('Closing the knowledge base')
            self.knowledge.close()
            self.knowledge = None
        robotlog.info('Closing the lowlevel!')
        for s in self.servers:
            self.servers[s].close()

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
                    robotlog.debug("Added " + m.__name__ + "." + member.__name__ + \
                                " as available action.")
                    if hasattr(member, "_broken"):
                        robotlog.warning("Action " + m.__name__ + "." + \
                                         member.__name__ + " is marked as broken! " \
                                         "Use it carefully.")
                    actions.append(member)
        return actions
    
    def add_action(self, fn):
        def innermethod(*args, **kwargs):
            action = "%s" % fn.__name__
            if hasattr(action, "_broken"):
                ok = raw_input("Attention! " + action + " is marked as broken."
                               "Are you sure you want to proceed? (y/N)")
                if ok != 'y':
                    return
            robotlog.debug("Calling action " + action)
            actions = fn(self, *args, **kwargs)
            return self.execute(actions)
                
        innermethod.__doc__ = fn.__doc__ if fn.__doc__ else fn.__name__ + \
            "\nThis method has been dynamically added to your robot."
        innermethod.__name__ = fn.__name__
        setattr(self,innermethod.__name__,innermethod)

        def get_actions_innermethod(*args, **kwargs):
            action = "%s" % fn.__name__
            robotlog.debug("Returning action for " + action)
            actions = fn(self, *args, **kwargs)
            return actions
                
        get_actions_innermethod.__doc__ = fn.__doc__ if fn.__doc__ else fn.__name__ + \
            "\nThis method has been dynamically added to your robot."
        get_actions_innermethod.__name__ = "getactions" + fn.__name__
        setattr(self,get_actions_innermethod.__name__,get_actions_innermethod)

    def _ack(self, evt):
        """ NOP function that serves as fallback callback
        """
        #TODO: We should here remove the corresponding entry in pocolibs_pending_request.
        #To do so, we need to construct special callbacks that unset the correct entry.
        pass

    def _execute_pocolibs(self, action):
        """ Execute a set of request.

        :param reqs: a list of request to execute. Each of them should be a list of
        - a module name
        - a request name
        - an (optinal) set of parameters
        - an optional flag to decide if the request must be blocking or not.
        """
        
        from pypoco import PocoRemoteError
	
        if not self.use_pocolibs:
            raise RobotError("This action '" + action["request"] + "' "
                     "requires Pocolibs, but this ActionPerformer "
                     "has been started without it.")

        if action["abort"]:
            # We want to abort a previous request.
            self._pending_pocolibs_requests[action["module"] + "." + action["request"]].abort()
            robotlog.info("Aborted " + action["module"] + "." + action["request"])
            return

        args = []
        for arg in action["args"]:
            if type(arg) == bool:
                if arg:
                    args.append("GEN_TRUE")
                else:
                    args.append("GEN_FALSE")
            else:
                args.append(arg)

        module = self.poco_modules[action["module"]]
        method = getattr(module, action["request"])

        if not action['wait_for_completion']:
            # asynchronous mode! 
            if action["callback"]:
                args = [action["callback"]] + args
            else:
                # we pass a (dummy) callback
                args = [self._ack] + args

        try:
            rqst = method(*args)
        except PocoRemoteError:
            robotlog.error(">>>>>>>>>>>>>> POCOREMOTE ERROR - Skipping it <<<<<<<<<<")
            robotlog.error(">>>>>>>>>>>>>> was: %s with args: %s <<<<<<<<<<" % (action["request"], str(action["args"])))
            return (False, None)
        if not action["wait_for_completion"]:
            # For asynchronous requests, we keep a request (PocoRequest object) if we need to abort the request.
            self._pending_pocolibs_requests[action["module"] + "." + action["request"]] = rqst
            return (True, None)

        robotlog.debug("Execution done. Return value: " + str(rqst))

        ok, res = rqst
        if ok == 'OK':
            return (True, res)
        else:
            return (False, res)

    def _execute_ros(self, action):
        """ Execute a ros action.

        :param reqs: 
        - an action name

        """

        if not self.use_ros:
            raise RobotError("This action '" + action["request"] + "' "
                     "requires ROS, but this ActionPerformer "
                     "has been started without it.")

        client = action['client']
        goal = action['goal']
        
        #state = self.GoalStatus
        #result = client.get_result()

    
        robotlog.debug("Sending goal " + str(action["goal"]) + " to " + str(client))
        # Sends the goal to the action server 
        client.send_goal(goal, done_cb = action["callback"], feedback_cb = action["feedback"])

        if action['wait_for_completion']:
            # Waits for the server to finish performing the action
            client.wait_for_result()

            # Checks if the goal was achieved
            if client.get_state() == self.GoalStatus.SUCCEEDED:
                robotlog.debug('ROS Action succeeded')
                return (True, None)
            else:
                robotlog.error("Action failed! " + client.get_goal_status_text())
                return (False, client.get_goal_status_text())
        else:
            return (True, None)

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
        super(self.__class__,self).__init__(['pr2c2', 'pr2c1'], 9472, use_ros = True, use_pocolibs = True, knowledge = knowledge, dummy = dummy)
        robotlog.info("PR2 actions loaded.")

        self.id = "PR2_ROBOT"

        self.state = PR2StateManager(self)

        if init:
            robotlog.info("Initializing modules...")
            self.init()
            robotlog.info("Initialization done.")

class JidoSimu(Robot):
    def __init__(self, knowledge = None, dummy = False, init = False, host = "joyce"):
        super(self.__class__,self).__init__([(host, 9472), (host, 9473)], port = None, use_ros = False, use_pocolibs = True, knowledge = knowledge, dummy = dummy)
        robotlog.info("Action loaded for Jido on MORSE simulator.")

        self.id = "JIDO_ROBOT"

        if init:
            robotlog.info("Initializing modules...")
            self.init()
            robotlog.info("Initialization done.")


import __main__ as main
if not hasattr(main, '__file__'):
    # Running in interactive mode:
    # Add a console logger
    handler = logging.StreamHandler()
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
    handler.setFormatter(formatter)
    robotlog.addHandler(handler)
