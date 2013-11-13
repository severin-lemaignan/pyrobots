import logging; robotlog = logging.getLogger("robot." + __name__)

from robots.exception import RobotError

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

class ROSActions:

    def __init__(self):
        rospy.init_node('pyrobots')
        self.GoalStatus = GoalStatus
        self._pending_ros_actionservers = []

    def cancelall(self):
        for client in self._pending_ros_actionservers:
            client.cancel_all_goals()
        self._pending_ros_actionservers = []
        

    def execute(self, action):
        """ Execute a ros action.

        :param reqs: 
        - an action name

        """
        if action['client']:
            client = action['client']
            self._pending_ros_actionservers.append(client) # store the action servers if we need to cancel the goals

            goal = action['goal']
            
            #state = self.GoalStatus
            #result = client.get_result()

        
            robotlog.debug("Sending goal " + str(action["goal"]) + " to " + str(client))
            # Sends the goal to the action server 
            client.send_goal(goal, done_cb = action["callback"], feedback_cb = action["feedback"])

            if action['wait_for_completion']:
                # Waits for the server to finish performing the action
                client.wait_for_result()

                if client in self._pending_ros_actionservers: # may be absent after a general cancelling
                    self._pending_ros_actionservers.remove(client)

                # Checks if the goal was achieved
                if client.get_state() == self.GoalStatus.SUCCEEDED:
                    robotlog.debug('ROS Action succeeded')
                    return (True, None)
                else:
                    robotlog.error("Action failed! " + client.get_goal_status_text())
                    return (False, client.get_goal_status_text())
            else:
                return (True, None)

        elif action['publisher']:
            publisher = action['publisher']
            robotlog.debug("Publishing " + str(action["goal"]) + " to " + str(publisher))
            publisher.publish(action['goal'])
            return (True, None)



    def close(self):
        robotlog.info('Closing the ROS lowlevel')
        robotlog.warning('Closing ROS: nothing to do?')

