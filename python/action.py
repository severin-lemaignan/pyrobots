def action(fn):
	"""The @action decorator
	"""
	fn._action = True
	return fn

def genom_request(module, request, args = None, wait_for_completion = True, abort = False, callback=None):
	return {"middleware": "pocolibs",
            "module": module,
            "request": request,
            "args": args,
	    "abort": abort,
	    "wait_for_completion": wait_for_completion,
	    "callback": callback}

def background_task(taskclass, args = None, wait_for_completion = True, abort = False, callback=None):
	return {"middleware": "python",
            "class": taskclass,
            "args": args,
	    "abort": abort,
	    "wait_for_completion": wait_for_completion,
	    "callback": callback}

def ros_request(client, goal, wait_for_completion = True, callback = None):
	return {"middleware": "ros",
            "client": client,
            "goal": goal,
	    "wait_for_completion": wait_for_completion,
	    "callback": callback}

def wait(seconds):
	""" This special action simply waits for a given amount of second before 
	sending the next action.
	"""
	return {"middleware": "special",
            "action": "wait",
            "args": seconds}

