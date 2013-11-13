# coding=utf-8
import logging; logger = logging.getLogger("robot." + __name__)

from robots.lowlevel import *
from robots.action import *


@action
@workswith(ROS)
@workswith({POCOLIBS:["textos"]})
def say(robot, msg, callback = None, feedback =None):
    """ Says loudly the message.

    Several TTS systems are tested:
    - first, try the Acapela TTS (through the acapela-ros Genom module)
    - then the ROS 'sound_play' node
    - eventually, the Genom 'textos' module

    :param msg: a text to say.
    """
    def execute(robot):
        logger.info("Robot says: " + msg)
        if robot.supports(ROS):
            import roslib; roslib.load_manifest('sound_play')
            import rospy, os, sys
            from sound_play.libsoundplay import SoundClient
            soundhandle = SoundClient()
            soundhandle.say(msg)
            return (True, None)
        elif robot.hasmodule("textos"):
            return robot.execute([
                genom_request(
                    "textos", 
                    "Say",
                    [msg],
                wait_for_completion = False if callback else True,
                callback = callback)])
        else:
            logger.warning("No ROS, no textos module: can not do speech synthesis.")
            return (True, None)

    if robot.supports(ROS):
        import rosnode
        nodes = rosnode.get_node_names()
        if "/acapela" in nodes:
            import actionlib
            from acapela.msg import SayGoal, SayAction

            # use Acapela TTS
            client = actionlib.SimpleActionClient('/acapela/Say', SayAction)

            ok = client.wait_for_server()
            if not ok:
                print("Could not connect to the Acapela ROS action server! Aborting action")
                return

            # Creates a goal to send to the action server.  
            goal = SayGoal()
            goal.message = msg
            

            return [ros_request(client, 
                    goal, 
                    wait_for_completion = False if callback else True,
                    callback = callback,
                    feedback=feedback
                )] # Return a non-blocking action. Useful to be able to cancel it later!

        elif "/nao_speech" in nodes:
            if callback:
                logger.error("Speech synthesis with Nao does not yet support callbacks.")
            import rospy
            from std_msgs.msg import String
            publisher = rospy.Publisher('speech', String)
            return [ros_request(publisher, String(msg))]


    return [python_request(execute)]

