import logging; logger = logging.getLogger("lowlevel")

#import pypoco

class ActionPerformer:

    def __init__(self, tclserv, use_ros = True):

        servers, self.poco_modules = pypoco.discover(tclserv)

        if use_ros:
            import roslib; roslib.load_manifest('navigation_actionlib')
            import rospy
            rospy.init_node('python_supervisor')

    def _execute_pocolibs(self, action):
        """ Execute a set of request.

        :param reqs: a list of request to execute. Each of them should be a list of
            - a module name
            - a request name
            - an (optinal) set of parameters
            - an optional flag to decide if the request must be blocking or not.
        """
        logger.info("Executing " + action["request"] + " on " + action["module"] + " with params " + str(action["args"]))
        module = self.poco_modules[action["module"]]
        method = getattr(module, action["request"])
        args = action["args"]
        method(*args)
        logger.info("Execution done.")

    def _execute_ros(self, action):

        client = action["client"]
        logger.info("Sending goal " + str(action["goal"]) + " to " + str(client))

        try:
            client.send_goal(action["goal"])
        except rospy.ROSInterruptException:
			print "program interrupted before completion"

        client.wait_for_result()


    def execute(self, module, *args, **kwargs):

        actions = module.main_action(*args, **kwargs)
        for action in actions:
            if action["middleware"] == "pocolibs":
                self._execute_pocolibs(action)
            elif action["middleware"] == "ros":
                self._execute_ros(action)
            else:
                logger.warning("Unsupported middleware. Skipping the action.")
