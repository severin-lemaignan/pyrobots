# coding=utf-8
import logging

try:
    import rospy
    from rosgraph_msgs.msg import Log
except ImportError:
    logging.warning("ROS not available: no ROS logging")

roslevel = {'DEBUG':1, 'INFO':2, 'WARNING':4, 'ERROR':8, 'CRITICAL':16}

class RXConsoleHandler(logging.Handler):
    def __init__(self, topic = "/rosout"):

        logging.Handler.__init__(self)

        rospy.sleep(0.5) # wait a bit to make sure our ROS node is up

        self.pub = rospy.Publisher(topic, Log)
        self.log = Log()

        self.level = logging.DEBUG

        self.log.level = 2
        self.log.name = "pyrobots logger"
        self.log.msg = "Welcome in pyRobots"
        self.log.file = ""
        self.log.function = ""
        self.log.line = 0
        self.log.header.stamp = rospy.rostime.Time.now()
        self.pub.publish(self.log)

    def emit(self, record):

        if record.levelname in roslevel:
            level = roslevel[record.levelname]
        else:
            level = 2 # Default to 'INFO'. Should be improved to default to closest 'symbolic' level

        log = Log(level = level,
                  name = record.name,
                  msg = record.msg,
                  file = record.filename,
                  function = record.funcName,
                  line = record.lineno)
        log.header.stamp = rospy.rostime.Time.now()

        self.pub.publish(log)

if __name__ == "__main__":
    logger = logging.getLogger("test")
    logger.addHandler(RXConsoleHandler())
    logger.setLevel(logging.DEBUG)
    
    logger.debug("Debug message")
    logger.info("Info message")
    logger.warning("Warning message")
    logger.error("Error message")
    logger.critical("Fatal message")


