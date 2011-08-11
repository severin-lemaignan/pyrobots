def action(fn):
	"""The @action decorator
	"""
	fn._action = True
	return fn

def genom_request(module, request, args = None, wait_for_completion = True, abort = False):
	return {"middleware": "pocolibs",
            "module": module,
            "request": request,
            "args": args,
	    "abort": abort,
	    "wait_for_completion": wait_for_completion}

def ros_request(client, goal, wait_for_completion = True):
	return {"middleware": "ros",
            "client": client,
            "goal": goal,
	    "wait_for_completion": wait_for_completion}

def wait(seconds):
	""" This special action simply waits for a given amount of second before 
	sending the next action.
	"""
	return {"middleware": "special",
            "action": "wait",
            "args": seconds}

