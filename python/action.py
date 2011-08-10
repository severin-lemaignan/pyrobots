
def action(fn):
	"""The @action decorator
	"""
	return fn

def genom_request(module, request, args = None):
	return {"middleware": "pocolibs",
            "module": module,
            "request": request,
            "args": args}

def ros_request(client, goal):
	return {"middleware": "ros",
            "client": client,
            "goal": goal}

def wait(seconds):
	""" This special action simply waits for a given amount of second before 
	sending the next action.
	"""
	return {"middleware": "special",
            "action": "wait",
            "args": seconds}

