
def action(fn):
	"""The @action decorator
	"""
	import sys
	sys.modules[fn.__module__].main_action = fn

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


