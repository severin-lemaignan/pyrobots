import logging; robotlog = logging.getLogger("robot." + __name__)
robotlog.setLevel(logging.DEBUG)

import pypoco
from pypoco import PocoRemoteError

from robots.exception import RobotError

class PocolibsActions:

    def __init__(self, host = "localhost", port = 9472):
        self.servers, self.poco_modules = pypoco.discover(host, port)
        self._pending_pocolibs_requests = {}

    def cancelall(self):
        """ Not implemented, but should be easy. Cf 'abort' in self.execute()
        """
        if self._pending_pocolibs_requests:
            robotlog.warning("Cancelling of Pocolibs requests not implemented!")

    def hasmodule(self, module):
        return True if module in self.poco_modules else False

    def execute(self, action):
        """ Execute a set of request.

        :param reqs: a list of request to execute. Each of them should be a list of
        - a module name
        - a request name
        - an (optinal) set of parameters
        - an optional flag to decide if the request must be blocking or not.
        """
        if action["abort"]:
            # We want to abort a previous request.
            self._pending_pocolibs_requests[action["module"] + "." + action["request"]].abort()
            robotlog.info("Aborted " + action["module"] + "." + action["request"])
            return

        args = []
        for arg in action["args"]:
            if type(arg) == bool:
                if arg:
                    args.append("GEN_TRUE")
                else:
                    args.append("GEN_FALSE")
            else:
                args.append(arg)

        module = self.poco_modules[action["module"]]
        method = getattr(module, action["request"])

        if not action['wait_for_completion']:
            # asynchronous mode! 
            if action["callback"]:
                args = [action["callback"]] + args
            else:
                # we pass a (dummy) callback
                args = [self._ack] + args

        try:
            rqst = method(*args)
        except PocoRemoteError as e:
            robotlog.error("Pocolibs action %s (with args: %s) failed. Error message: %s" % (action["request"], str(action["args"]), str(e)))
            return (False, None)
        if not action["wait_for_completion"]:
            # For asynchronous requests, we keep a request (PocoRequest object) if we need to abort the request.
            self._pending_pocolibs_requests[action["module"] + "." + action["request"]] = rqst
            return (True, None)

        robotlog.debug("Execution done. Return value: " + str(rqst))

        ok, res = rqst
        if ok == 'OK':
            return (True, res)
        else:
            return (False, res)

    def close(self):
        robotlog.info('Closing the Pocolibs lowlevel')
        for s in self.servers:
            self.servers[s].close()

