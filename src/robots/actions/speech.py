# coding=utf-8
import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)


from robots.action import *


@action
def say(robot, msg):
    """ Says loudly the message.

    Speech synthesis relies on the ROS wrapper around Festival.
    :param msg: a text to say.
    """
    def execute(robot):
        logger.info("Robot says: " + msg)
        if robot.hasROS:
            import roslib; roslib.load_manifest('sound_play')
            import rospy, os, sys
            from sound_play.msg import SoundRequest
            from sound_play.libsoundplay import SoundClient
            soundhandle = SoundClient()
            soundhandle.say(msg)
            return (True, None)
        else:
            logger.warning("No ROS, can not do speech synthesis.")
            return (False, None)

    return [python_request(execute)]

