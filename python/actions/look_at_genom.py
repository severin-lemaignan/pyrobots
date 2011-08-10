import random
from action import action, genom_request

@action
def look_at(place):
    return look_at_xyz(place['x'], place['y'], place['z'])

@action
def look_at_xyz(x,y,z, frame = "/map"):
    """ Look at via pr2SoftMotion.
    """
    actions = [
        genom_request(	"pr2SoftMotion",
            "MoveHead",
            [x,y,z,frame]
        )
    ]

    return actions

