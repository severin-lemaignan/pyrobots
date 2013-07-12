import logging; robotlog = logging.getLogger("robot." + __name__)
robotlog.setLevel(logging.DEBUG)

import naoqi

from robots.exception import RobotError


class NAOqiActions:

    def __init__(self, ip="nao.local", port=9559):
        self.proxies = {}

        try:
            self.proxies['memory'] = naoqi.ALProxy("ALMemory", ip, port)
            self.proxies['motion'] = naoqi.ALProxy("ALMotion", ip, port)
        except Exception as e:
            robotlog.error("Error when creating one of the NAOqi proxy:" + str(e))
            raise e

    def cancelall(self):
        robotlog.warning("NAOqi cancellation of background tasks not supported")

    def execute(self, action):
        """ Execute a NAOqi action.
        """
        proxy = self.proxies[action['proxy']]

        robotlog.debug("Calling method " + action["name"])

        method = getattr(proxy, action["method"])

        if action['wait_for_completion']:
            try:
                res = method(*action["args"])
                robotlog.debug('NAOqi action successfully completed')
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

