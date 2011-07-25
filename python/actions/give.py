import random
from action import action, genom_request

used_plan_id = []

def getplanid():
    """ Returns a random plan id (for Amit planification routines) which is
    guaranteed to be 'fresh'.
    """
    plan_id = random.randint(1, 1000)
    while plan_id in used_plan_id:
        plan_id = random.randint(1, 1000)
    used_plan_id.append(plan_id)
    return plan_id

@action
def give(performer, obj, receiver):
    """ The 'Amit' GIVE.
    """
    plan_id = getplanid()
    actions = [
        genom_request("mhp",
            "Plan_HRI_Task",
            [plan_id, "GIVE_OBJECT", obj, performer,  receiver, 0, 0, 0]
        ),
        genom_request(	"mhp",
            "Get_SM_Traj_HRI_Task",
            [plan_id]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["OPEN"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [0]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [1]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["CLOSE"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [3]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [4]
        ),
        genom_request(	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ),
        genom_request(	"pr2SoftMotion",
            "GripperGrabRelease",
            ["RELEASE"]
        )
    ]

    return actions

