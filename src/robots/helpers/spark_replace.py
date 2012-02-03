from action import action, genom_request


@action
def place_object(obj, x, y, z = 0, yaw = 0.0, pitch = 0.0, roll = 0.0):

	return [genom_request("spark",
				"PlaceObject",
				[obj, x, y, z, yaw , pitch , roll]
		)]


@action
def place_agent(agent, x, y, yaw = 0.0, pitch = 0.0, roll = 0.0):

        return [genom_request("spark",
                                "PlaceAgent",
                                [agent, x, y, yaw , pitch , roll, 'STANDING', 1]
                )]
