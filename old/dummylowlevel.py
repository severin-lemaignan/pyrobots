""" This is a dummy low-level execution controler, used to test scripts without 
the robot.

Commands are simply output on the console.
"""
import logging; logger = logging.getLogger("novela." + __name__)
logger.setLevel(logging.DEBUG)

class ActionPerformer:

    def __init__(self, host, port, use_ros = True):
        pass
    
    def close(self):
        logger.info('Closing the dummy lowlevel!')

    def execute(self, fn, *args, **kwargs):
    
        actions = fn(*args, **kwargs)
        if actions:
            for action in actions:
                logger.info("Executing " + str(action))
                if action["middleware"] not in ["pocolibs", "ros", "special"]:
                    logger.warning("Unsupported middleware. Skipping the action.")

