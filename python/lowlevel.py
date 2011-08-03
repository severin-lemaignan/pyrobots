import logging; logger = logging.getLogger("lowlevel")
logger.setLevel = logging.DEBUG
import rospy
import actionlib
import pypoco


class ActionPerformer:

	def __init__(self, host, port, use_ros = True):

		servers, self.poco_modules = pypoco.discover(host, port)

		if use_ros:
			import roslib; roslib.load_manifest('novela_actionlib')
			#import rospy
			rospy.init_node('novela_actionlib')
			#import actionlib

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

		client = action['client']
                goal = action['goal']
		
        	#logger.info("Sending goal " + str(action["goal"]) + " to " + str(client))
        	print("Sending goal " + str(action["goal"]) + " to " + str(client))
                # Sends the goal to the action server 
                client.send_goal(goal)
        	
                # Waits for the server to finish performing the action
        	client.wait_for_result()

		# Prints out the result of executing the action
		return client.get_client_result() # A MoveBaseResult
		
		if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
			print(str(client.get_result()))
		else:
			#logger.error("Action failed!")
			print("Action failed!")
	
	def execute(self, module, *args, **kwargs):

		actions = module.main_action(*args, **kwargs)
		for action in actions:
			if action["middleware"] == "pocolibs":
				self._execute_pocolibs(action)
			elif action["middleware"] == "ros":
				self._execute_ros(action)
			else:
				logger.warning("Unsupported middleware. Skipping the action.")
