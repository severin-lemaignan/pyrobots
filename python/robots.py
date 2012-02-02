import time
import pypoco
from pypoco import PocoRemoteError

import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from exception import RobotError

class Robot:
    """ This 'low-level' class implements all what is required to actually execute
    actions on the robot.
    
    It supports execution of ROS actions, Genom requests, Python code and some
    special actions like 'wait'.
    """
    def __init__(self, host = None, port = None, use_pocolibs = True, use_ros = True):

        self.use_pocolibs = use_pocolibs
        self.use_ros = use_ros

        if use_pocolibs:
                self.servers, self.poco_modules = pypoco.discover(host, port)

                self._pending_pocolibs_requests = {}
                self._pending_python_requests = {}

        if use_ros:
            import roslib; roslib.load_manifest('novela_actionlib')
            import rospy
            rospy.init_node('novela_actionlib')
            import actionlib
            from actionlib_msgs.msg import GoalStatus
            self.GoalStatus = GoalStatus

    def close(self):
        logger.info('Closing the lowlevel!')
        for s in self.servers:
            self.servers[s].close()

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
        
        if not self.use_pocolibs:
            raise RobotError("This action '" + action["request"] + "' "
                     "requires Pocolibs, but this ActionPerformer "
                     "has been started without it.")

        if action["abort"]:
            # We want to abort a previous request.
            self._pending_pocolibs_requests[action["module"] + "." + action["request"]].abort()
            logger.warning("Aborted " + action["module"] + "." + action["request"])
            return

        logger.info("Executing " + action["request"] + " on " + action["module"] + " with params " + str(action["args"]))
        module = self.poco_modules[action["module"]]
        method = getattr(module, action["request"])

        args = action["args"] if action["args"] else []
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
            print(">>>>>>>>>>>>>> POCOREMOTE ERROR - Skipping it <<<<<<<<<<")
            print(">>>>>>>>>>>>>> was: %s with args: %s <<<<<<<<<<" % (action["request"], str(action["args"])))
            return None
        if not action["wait_for_completion"]:
            # For asynchronous requests, we keep a request (PocoRequest object) if we need to abort the request.
            self._pending_pocolibs_requests[action["module"] + "." + action["request"]] = rqst

        logger.info("Execution done.")
        logger.debug(str(rqst))
        return rqst

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

    
        print("Sending goal " + str(action["goal"]) + " to " + str(client))
        # Sends the goal to the action server 
        client.send_goal(goal, done_cb = action["callback"])
           
        if action['wait_for_completion']:	
            # Waits for the server to finish performing the action
            client.wait_for_result()

            # Checks if the goal was achieved
            if client.get_state() == self.GoalStatus.SUCCEEDED:
                print('Action succeeded')
            else:
                print("Action failed!")

    def _execute_python(self, action):
        
        if action["abort"]:
            try:
                self._pending_python_requests[action["class"]].stop()
                logger.warning("Aborted Python background task " + action["class"].__name__)
            except KeyError:
                pass #Likely a task already aborted
            return

        logger.info("Starting Python background task " + action["class"].__name__ + " with params " + str(action["args"]))
        instance = action["class"](self, *action["args"])
        self._pending_python_requests[action["class"]] = instance
        instance.start()
        
    def _execute_special(self, action):
        if action["action"] == "wait":
            logger.info("Waiting for " + str(action["args"]))
            time.sleep(action["args"])
    
    def execute(self, fn, *args, **kwargs):
    
        logger.debug(str(fn))	
        actions = fn(*args, **kwargs)
        result = None
        if actions:
            logger.debug(str(actions))
            for action in actions:
                logger.info("Executing " + str(action))
                if action["middleware"] == "pocolibs":
                    res = self._execute_pocolibs(action)
                    if res:
                        result = res
                elif action["middleware"] == "ros":
                    res = self._execute_ros(action)
                    if res:
                        result = res
                elif action["middleware"] == "python":
                    res = self._execute_python(action)
                    if res:
                        result = res
                elif action["middleware"] == "special":
                    res = self._execute_special(action)
                    if res:
                        result = res
                else:
                    logger.warning("Unsupported middleware. Skipping the action.")
        return result
        
class Pr2(Robot):
    def __init__(self):
        super(self.__class__,self).__init__(['pr2c2', 'pr2c1'], 9472, use_ros = True, use_genom = True)
