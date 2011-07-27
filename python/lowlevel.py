import logging; logger = logging.getLogger("lowlevel")

#import pypoco

class ActionPerformer:

    def __init__(self, tclserv, rosmaster = None):

        #servers, self.poco_modules = pypoco.discover(tclserv)

        if rosmaster:
            logger.warning("ROS not yet supported")

    def _execute_pocolibs(self, action):
        """ Execute a set of request.

        :param reqs: a list of request to execute. Each of them should be a list of
            - a module name
            - a request name
            - an (optinal) set of parameters
            - an optional flag to decide if the request must be blocking or not.
        """
        logger.info("Executing " + action["request"] + " on " + action["module"] + " with params " + str(action["args"]))
        #module = self.poco_modules[action["module"]]
        #method = getattr(module, action["request"])
        #args = action["args"]
        #method(*args)
        logger.info("Execution done.")

    def execute(self, module, *args, **kwargs):

        actions = module.main_action(*args, **kwargs)
        for action in actions:
            if action["middleware"] == "pocolibs":
                self._execute_pocolibs(action)
            else:
                logger.warning("Unsupported middleware. Skipping the action.")
