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
    return {"name": client.action_client.ns,
            "middleware": "ros",
            "client": client,
            "goal": goal,
            "wait_for_completion": wait_for_completion,
            "callback": callback,
            "feedback": feedback}

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

def add_knowledge(stmts, memory_profile = "LONGTERM"):
    return [{"name": "knowledge.add",
            "middleware": "knowledge",
            "action": "add",
            "args": stmts,
            "memory_profile": memory_profile}]

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

