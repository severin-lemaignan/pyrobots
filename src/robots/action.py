
import logging; logger = logging.getLogger("robot." + __name__)

from robots.lowlevel import mw_names
from robots.exception import RobotError

def action(fn):
	"""The @action decorator.
	"""
	fn._action = True
	return fn

def helper(access):
    """The @helper decorator.

    :param access: the name of the variable that gives access to
    the helper. For instance, if access='planning' for the helper
    'plan()', the helper can be invoked by: 'robot.planning.plan()'
    """
    def decorator(fn):
        fn._helper = True
        fn._helper_access = access
        return fn
    return decorator

def tested(date):
    """Marks the last time this method has been tested.
        """
    def decorator(fn):
        fn._tested = date
        return fn
    return decorator

def broken(fn):
    """Marks an action as 'broken', to ease tracking
    """
    fn._action = True
    fn._broken = True
    return fn

def workswith(requirements):
    """ Explicit the middleware/platform requirements for the action.

        If several configurations are possible, use this decorator as many time
        as required. In other terms, for a given action, each occurence of this
        decorator is a logical OR with previous occurences.

        :param requirements: may be either a middleware as listed in
        lowlevel.__init__.py (ROS, POCOLIBS...), a list of such middleware (in
        that case, it is considered as a logical AND: the presence of each of
        the middleware is required), or a dictionary that list specfic
        modules/nodes for specific middlewares. For instance:
            {lowlevel.ROS:["/head_traj_controller/point_head_action"],
             lowlevel.POCOLIBS:["platine", "spark"]}
        For ROS, it takes a list of actionlib servers names, for POCOLIBS, a
        list of modules.

    """
    def decorator(fn):
        if hasattr(fn, "_requirements"):
            fn._requirements.append(requirements)
        else:
            fn._requirements = [requirements]
        return fn
    return decorator

def same_requirements_as(action):
    """ Indicates that the action has the same middleware/modules
    requirements as another action.
    """
    def decorator(fn):
        fn._same_requirements_as = action
        return fn
    return decorator


def genom_request(module, request, args = None, wait_for_completion = True, abort = False, callback=None):
    if callback:
        wait_for_completion = False

    return {"name": module + "." + request,
            "middleware": "pocolibs",
            "module": module,
            "request": request,
            "args": args,
        "abort": abort,
        "wait_for_completion": wait_for_completion,
        "callback": callback}

def background_task(taskclass, args = None, wait_for_completion = True, abort = False, callback=None):
    return {"name": taskclass.__name__,
            "middleware": "background",
            "class": taskclass,
            "args": args,
            "abort": abort,
            "wait_for_completion": wait_for_completion,
            "callback": callback}

def ros_request(client, goal, wait_for_completion = True, callback = None, feedback = None):
    """
    :param name: an arbitrary name that describe what is this action (for logging purposes)
    :param callback: an (optional) callback that is called when the action is completed.
    :param feedback: an (optional) callback that is called everytime the feedback topic is updated.
    """
    try:
        name = client.action_client.ns
    except AttributeError:
        #probably in dummy mode!
        name = client

    return {"name": name,
            "middleware": "ros",
            "client": client,
            "goal": goal,
            "wait_for_completion": wait_for_completion,
            "callback": callback,
            "feedback": feedback}

def naoqi_request(proxy, method, args = None, wait_for_completion = True):

    return {"name": proxy + "." + method,
            "middleware": "naoqi",
            "proxy": proxy,
            "method": method,
            "args": args,
            "wait_for_completion": wait_for_completion}

def python_request(functor, args = [], wait_for_completion = True, callback = None):
    """
    :param callback: an (optional) callback that is called when the action is completed.
    """
    return {"name": functor.__name__,
            "middleware": "python",
            "functor": functor,
            "args": args,
            "wait_for_completion": wait_for_completion,
            "callback": callback}

def add_knowledge(stmts, lifespan = None):
    return [{"name": "knowledge.add",
            "middleware": "knowledge",
            "action": "add",
            "args": stmts,
            "lifespan": lifespan}]

def retract_knowledge(stmts):
    return [{"name": "knowledge.retract",
             "middleware": "knowledge",
            "action": "retract",
            "args": stmts}]

def wait(seconds):
    """ This special action simply waits for a given amount of second before 
    sending the next action.
    """
    return {"name": "wait",
            "middleware": "special",
            "action": "wait",
            "args": seconds}


class RobotAction:
    def __init__(self, fn, module=None):

        self.fn = fn
        self.name = self.fn.__name__
        self.module = module
        self.fqn = module.__name__ + "." + self.name

        self.requirements = None
        if hasattr(self.fn, "_requirements"):
            self.requirements = [self._normalize_requirements(req) for req in self.fn._requirements]
            self.print_requirements()
        elif hasattr(self.fn, "_same_requirements_as"):

            # recursively look for an action that defines requirements
            def find_requirements_holder(fn):
                if hasattr(fn, "_requirements"):
                    return fn._requirements
                elif hasattr(fn, "_same_requirements_as"):
                    return find_requirements_holder(fn._same_requirements_as)
                return None

            reqs = find_requirements_holder(self.fn._same_requirements_as)

            if reqs:
                self.requirements = [self._normalize_requirements(req) for req in reqs]
            else:
                raise RobotError("Action <%s> should have the same requirements as <%s>, but I could not determine <%s> requirements." % (self.fqn, self.fn._same_requirements_as.__name__, self.fn._same_requirements_as.__name__))

        logger.debug("Added " + self.fqn + \
                    " as available action.")
        if self.broken():
            logger.warning("Action " + self.fqn + " is marked as broken! " \
                                "Use it carefully.")

    def __str__(self):
        return self.fqn

    def _normalize_requirements(self, req):

        # case req= lowlevel.ROS
        if isinstance(req, int):
            return {req:[]}

        res = {}

        # case req = [ROS, POCOLIBS]
        if isinstance(req, list):
            for e in req:
                res[e] = []
            return res

        # case req = {ROS:"/toto", POCOLIBS:["titi", "tata"]}
        if isinstance(req, dict):
            for mw, modules in req.items():
                if not isinstance(modules, list):
                    modules = [modules]
                res[mw] = modules
            return res

    def broken(self):
        return hasattr(self.fn, "_broken")

    def print_requirements(self):

        def printmw(req):
            res = []
            for mw, modules in req.items():
                desc = mw_names[mw]
                if modules:
                    desc += " (with " + ", ".join(modules) + ")"
                res.append(desc)
            return " and ".join(res)

        req = self.requirements
        
        if not req:
            logger.debug("Action <%s> does not specify platform requirements" % self.fqn)
        else:
            res = "Action <%s> requires " % self.fqn
            if len(req) == 1:
                res += printmw(req[0])
            else:
                res += "either [" + "] or [".join([printmw(option) for option in req])

            res += "]."
            logger.debug(res)

if __name__ == "__main__":

    import sys
    from robots.lowlevel import *

    def toto():
        pass

    toto._requirements = [ROS, [POCOLIBS, ROS], {ROS:[], POCOLIBS:"pr2SoftMotion"}, {ROS:["toto", "tata"]}]

    action = RobotAction(toto, sys.modules["__main__"])
    action.print_requirements()
