import time
import logging; logger = logging.getLogger("lowlevel")
logger.setLevel = logging.DEBUG
import pypoco

class ActionPerformer:

	def __init__(self, host, port, use_ros = True):

		self.servers, self.poco_modules = pypoco.discover(host, port)

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
		pass	

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
		if not action['wait_for_completion']:
			# asynchronous mode! we pass a (dummy) callback
			args = [self._ack] + args
		method(*args)
		logger.info("Execution done.")

	def _execute_ros(self, action):

		client = action['client']
                goal = action['goal']

		""" Execute a ros action.

                :param reqs: 
                - an action name

                """
		
        	print("Sending goal " + str(action["goal"]) + " to " + str(client))
                # Sends the goal to the action server 
                client.send_goal(goal)
        
		if action['wait_for_completion']:	
			# Waits for the server to finish performing the action
			client.wait_for_result()

			# Checks if the goal was achieved
			if client.get_state() == self.GoalStatus.SUCCEEDED:
				print('Action succeeded')
			else:
				print("Action failed!")
	
	def _execute_special(self, action):
		if action["action"] == "wait":
			logger.info("Waiting for " + str(action["args"]))
			time.sleep(action["args"])

	def execute(self, fn, *args, **kwargs):

		actions = fn(*args, **kwargs)
		if actions:
			for action in actions:
				logger.info("Executing " + str(action))
				if action["middleware"] == "pocolibs":
					self._execute_pocolibs(action)
				elif action["middleware"] == "ros":
					self._execute_ros(action)
				elif action["middleware"] == "special":
					self._execute_special(action)
				else:
					logger.warning("Unsupported middleware. Skipping the action.")

