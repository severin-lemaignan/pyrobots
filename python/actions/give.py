used_plan_id = []

def getplanid():
    """ Returns a random plan id (for Amit planification routines) which is
    guaranteed to be 'fresh'.
    """
    plan_id = random.randint(1, 1000)
    while plan_id in GenomAction.used_plan_id:
        plan_id = random.randint(1, 1000)
    GenomAction.used_plan_id.append(plan_id)
    return plan_id

def give(performer, obj, receiver):
    """ The 'Amit' GIVE.
    """
    plan_id = GenomAction.getplanid()
    actions = [
        [	"mhp",
            "Plan_HRI_Task",
            [plan_id, "GIVE_OBJECT", obj, performer,  receiver, 0, 0, 0]
        ],
        [	"mhp",
            "Get_SM_Traj_HRI_Task",
            [plan_id]
        ],
        [	"pr2SoftMotion",
            "GripperGrabRelease",
            ["OPEN"]
        ],
        [	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [0]
        ],
        [	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ],
        [	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [1]
        ],
        [	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ],
        [	"pr2SoftMotion",
            "GripperGrabRelease",
            ["CLOSE"]
        ],
        [	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [3]
        ],
        [	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ],
        [	"mhp",
            "Write_this_SM_Traj_to_Poster",
            [4]
        ],
        [	"pr2SoftMotion",
            "TrackQ",
            ["mhpArmTraj", "PR2SM_TRACK_POSTER", "RARM"]
        ],
        [	"pr2SoftMotion",
            "GripperGrabRelease",
            ["RELEASE"]
        ]
    ]

    return actions

