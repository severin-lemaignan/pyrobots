from action import genom_request

def getabspose(object_name):

	actions = [ genom_request("spark", "GetJointAbsPose", object_name) ]

	return actions

def setabspose(object_name):

	res = getabspose(object_name)
	ans = res[1]
	x_goal, y_goal, z_goal = float(ans[3]), float(ans[4]), float(ans[5])

	return [x_goal, y_goal, z_goal]
	
