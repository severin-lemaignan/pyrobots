import random
from action import action, genom_request

@action
def look_at(place):
    try:
	frame = place['frame']
    catch KeyError:
	frame = "/map"

    return look_at_xyz(place['x'], place['y'], place['z'], frame)

@action
def look_at_xyz(x,y,z, frame = "/map"):
    """ Look at via pr2SoftMotion.
    """
    print("Looking at " + str([x,y,z]) + " in " + frame)
    actions = [
        genom_request(	"pr2SoftMotion",
            "MoveHead",
            [x,y,z,frame]
        )
    ]

    return actions

