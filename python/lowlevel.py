import logging; logger = logging.getLogger("lowlevel")
logger.setLevel = logging.DEBUG
import rospy
import pypoco
import actionlib

class ActionPerformer:

    def __init__(self, host, port, use_ros = True):

        servers, self.poco_modules = pypoco.discover(host, port)

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

	ok = client.wait_for_server(rospy.Duration(5.0))
	if not ok:
		#logger.error("Could not connect to the ROS client! Aborting action")
		print("Could not connect to the ROS client! Aborting action")
		return

        #logger.info("Sending goal " + str(action["goal"]) + " to " + str(client))
        print("Sending goal " + str(action["goal"]) + " to " + str(client))

        try:
            client.send_goal(action["goal"])
        except rospy.ROSInterruptException:
			print "program interrupted before completion"

	print 'ok1'
        client.wait_for_result(rospy.Duration.from_sec(5.0))

	if not client.get_state() == actionlib.GoalStatus.SUCCEEDED:
		print('ok2: ' + str(client.get_result()))
	else:
		#logger.error("Action failed!")
		print("Action failed!")
	
	#if (getState() == actionlib.SimpleClientGoalState.SUCCEEDED):
		#ROS_INFO('PR2 is arrived at the destination')
	#else:
		#ROS_INFO('PR2 isn\'t arrived at the destination for some reason')

    def execute(self, module, *args, **kwargs):

        actions = module.main_action(*args, **kwargs)
        for action in actions:
            if action["middleware"] == "pocolibs":
                self._execute_pocolibs(action)
            elif action["middleware"] == "ros":
                self._execute_ros(action)
            else:
                logger.warning("Unsupported middleware. Skipping the action.")
