import logging; logger = logging.getLogger("novela." + __name__)
from helpers import trajectory, position, cb, postures
from action import action, genom_request

from actions.administration import wait

@action
def basket(robot, duration = 4):
        poses = postures.read()
        #duration in seconds
        from actions.configuration import setpose
        j = 0

        # Where is the human?
	human = None
	while not human:
	        human = position.gethumanpose(robot)      
		if not human:
			logger.warning("HUMAN not detected!!!")
			raw_input("Press enter to retry")
		else:
		    break
	
        state = "NONE"

        while j < duration:

                # Where is the human?
                ok, res = robot.execute(position.getrelativepose,"XAVIER_HUMAN", "PR2_ROBOT")
                yaw, pitch, roll, x, y, z = [float(x) for x in res]

                print(" Basket game: human delta = " + str(y))

                # Mirror moving of the robot
                if y > 0.2 and state != "RIGHT":
                   robot.execute(setpose, poses["TRASHGAME_RIGHT"])
                   state = "RIGHT"
                   j += 1
                elif y < -0.2 and state != "LEFT":
                   robot.execute(setpose, poses["TRASHGAME_LEFT"])
                   state = "LEFT"
                   j += 1




@action
def gym():

    move_1 = trajectory.Trajectory('hands_up')
    move_2 = trajectory.Trajectory('arms_against_torso')
    move_3 = trajectory.Trajectory('handsup_folded')
    move_4 = trajectory.Trajectory('alternative_handsup_folded')
    move_5 = trajectory.Trajectory('move_head')
    move_6 = trajectory.Trajectory('rarm_swinging')
    move_7 = trajectory.Trajectory('larm_swinging')
    move_8 = trajectory.Trajectory('slow_arms_swinging')
    move_9 = trajectory.Trajectory('speed_arms_swinging')	

    
    actions = [	genom_request( 'pr2SoftMotion', 'GotoQ', 
            		['PR2', 0] + move_1.initcoords() ) ] +\
        	wait(0.2) +\
               [ genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_1.abspath(),"PR2SM_TRACK_FILE", "PR2"])] +\
        	wait(0.2) +\
               [ genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_1.abspath(),"PR2SM_TRACK_FILE", "PR2"])] +\
                wait(0.2) +\
        	[ genom_request( 'pr2SoftMotion', 'GotoQ', 
                        ['PR2', 0] + move_2.initcoords() )] +\
                wait(0.2) +\
                [genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_2.abspath(),"PR2SM_TRACK_FILE", "PR2"])] +\
                wait(0.2) +\
        	[genom_request( 'pr2SoftMotion', 'GotoQ', 
                        ['PR2', 0] + move_3.initcoords() )] +\
                wait(0.2) +\
        	[genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_3.abspath(),"PR2SM_TRACK_FILE", "PR2"])] +\
                wait(0.2) +\
        	[genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + move_4.initcoords() )] +\
                wait(0.2) +\
                [genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_4.abspath(),"PR2SM_TRACK_FILE", "PR2"])] +\
                wait(0.2) +\
        	[genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_4.abspath(),"PR2SM_TRACK_FILE", "PR2"])] +\
                wait(0.2) +\
                [genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + move_5.initcoords() )] +\
                wait(0.2) +\
        	[genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_5.abspath(),"PR2SM_TRACK_FILE", "PR2"])]+\
                wait(0.2) +\
               [genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + move_8.initcoords() )] +\
                wait(0.2)+\
                [genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_8.abspath(),"PR2SM_TRACK_FILE", "PR2"])] +\
                wait(0.2) +\
        	[genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_8.abspath(),"PR2SM_TRACK_FILE", "PR2"])]
   
    return actions

@action
def handsup():

    traj = trajectory.Trajectory('hands_up')

    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['ARMS', 0] + traj.initcoords() ),
                wait(0.5),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                        [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
        wait(0.1),
        genom_request( 'pr2SoftMotion', 'TrackQ',
                        [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"])
        ]

    return actions

@action
def arms_against_torso():

    traj = trajectory.Trajectory('arms_against_torso')

    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['ARMS', 0] + traj.initcoords() ),
            wait(0.2),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
    ]

    return actions

@action
def handsup_folded():

    traj = trajectory.Trajectory('handsup_folded')

    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['ARMS', 0] + traj.initcoords() ),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
            ]

    return actions


@action
def alternative_handsup_folded():

    traj = trajectory.Trajectory('alternative_handsup_folded')

    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['ARMS', 0] + traj.initcoords() ),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
            ]

    return actions


@action
def move_head():

    traj = trajectory.Trajectory('move_head')

    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['PR2', 0] + traj.initcoords() ),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
            ]

    return actions

@action
def larm_swinging():

    traj = trajectory.Trajectory('larm_swinging')

    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['ARMS', 0] + traj.initcoords() ),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
            ]

    return actions



@action
def rarm_swinging():

    traj = trajectory.Trajectory('rarm_swinging')

    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['ARMS', 0] + traj.initcoords() ),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
            ]

    return actions


@action
def slow_arms_swinging():

    traj = trajectory.Trajectory('slow_arms_swinging')
    print (traj.initcoords())
    actions = [genom_request('pr2SoftMotion', 'SetSpeedLimit', [1]),
    genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['ARMS', 0] + traj.initcoords() ),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"])
            ]

    return actions


@action
def speed_arms_swinging():

    traj = trajectory.Trajectory('speed_arms_swinging')
    print (traj.initcoords())
    actions = [genom_request( 'pr2SoftMotion', 'GotoQ',
                    ['ARMS', 0] + traj.initcoords() ),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"]),
            wait(0.5),
            genom_request( 'pr2SoftMotion', 'TrackQ',
                    [ traj.abspath(),"PR2SM_TRACK_FILE", "ARMS"])

            ]

    return actions

