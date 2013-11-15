import logging; robotlog = logging.getLogger("robot." + __name__)

import naoqi

from robots.exception import RobotError

MOTION_METHODS = ["angleInterpolation", 
                  "angleInterpolationWithSpeed",
                  "angleInterpolationBezier",
                  "setAngles",
                  "changeAngles",
                  "goToPosture",
                  "applyPosture",
                  "closeHand",
                  "openHand",
                  "positionInterpolation",
                  "positionInterpolations",
                  "setPosition",
                  "changePosition",
                  ]

class NAOqiActions:

    def __init__(self, ip="nao.local", port=9559):
        self.proxies = {}

        try:
            self.proxies['memory'] = naoqi.ALProxy("ALMemory", ip, port)
            self.proxies['motion'] = naoqi.ALProxy("ALMotion", ip, port)
            self.proxies['posture'] = naoqi.ALProxy("ALRobotPosture", ip, port)
        except Exception as e:
            robotlog.error("Error when creating one of the NAOqi proxy:" + str(e))
            raise e

        res = self.proxies['motion'].setCollisionProtectionEnabled("Arms", True)
        if res:
            robotlog.info("Collision avoidance enabled on the arms.")
        else:
            robotlog.warning("Unable to enable collision avoidance on the arms!")

    def motorsEnabled(self, joints = "Body"):
        return 0.0 not in self.proxies["motion"].getStiffnesses(joints)

    def enableMotors(self, joints = "Body"):
        
        self.proxies["motion"].setStiffnesses(joints, 1.0)


    def cancelall(self):
        robotlog.warning("NAOqi cancellation of background tasks not supported")

    def execute(self, action):
        """ Execute a NAOqi action.
        """
        proxy = self.proxies[action['proxy']]

        robotlog.debug("Calling method " + action["name"])

        if action["method"] in MOTION_METHODS:
            if not self.motorsEnabled(action["parts"]):
                self.enableMotors(action["parts"])

        method = getattr(proxy, action["method"])

        if action['wait_for_completion']:
            try:
                res = method(*action["args"])
                robotlog.debug('NAOqi action successfully completed with result: ' + str(res))
                return (True, res)
            except RuntimeError as e:
                robotlog.error("Action failed! " + str(e))
                return (False, str(e))
        else:
            robotlog.error("Async requests to NAOqi are not yet supported!")
            return (False, None)



    def close(self):
        robotlog.info('Closing the NAOqi lowlevel')
        robotlog.warning('Closing NAOqi: nothing to do?')

