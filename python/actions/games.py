import logging; logger = logging.getLogger("novela." + __name__)
from helpers import trajectory, position, cb, postures
from action import action, genom_request

from actions.administration import wait

import math

def norm(vec):
    return math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

def ecart(posold, posnew, ref):
   """
   Returns the lateral displacement of 'posold' to 'posnew'
   viewed from 'ref'.
   
   :param posold, posnew, ref: [x,y] 2D vectors
   """
   vectref = [posold[0] - ref[0], posold[1] - ref[1]]
   vectnew = [posnew[0] - ref[0], posnew[1] - ref[1]]
   orthoref = [-vectref[1], vectref[0]]
   scal = orthoref[0] * vectnew[0] + orthoref[1] * vectnew[1]
   orthoref = [orthoref[0] / norm(orthoref) * scal, orthoref[1] / norm(orthoref) * scal]
   return orthoref[0]

@action
def basket(robot, duration = 20):
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
	
        oldhumpose = [human["x"], human["y"]]

        robotpose = position.mypose()
        refpose = [robotpose["x"], robotpose["y"]]

	frequency = 5 #Hz
   
        state = "NONE"

        while j < duration * frequency:

                # Where is the human?
                human = position.gethumanpose(robot)

                if not human:
                    #Human at origin!
                    logger.warning("HUMAN not detected anymore!!!")
                    robot.execute(wait, 1/frequency)
                    j += 1
                    continue

                newhumpose = [human["x"], human["y"]]

                
                # Evaluation of the distance that he is moved
                delta = ecart(oldhumpose, newhumpose, refpose)
                print(" Basket game: human delta = " + str(delta))

                # Mirror moving of the robot
                if delta > 0.5 and state != "RIGHT":
                   robot.execute(setpose, poses["TRASHGAME_RIGHT"])
                   state = "RIGHT"
                elif delta < -0.5 and state != "LEFT":
                   robot.execute(setpose, poses["TRASHGAME_LEFT"])
                   state = "LEFT"

                robot.execute(wait, 1/frequency)

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

    
    actions = [
		genom_request( 'pr2SoftMotion', 'GotoQ', 
            		['PR2', 0] + move_1.initcoords() ),
        	wait(0.2),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_1.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
        	wait(0.2),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_1.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                wait(0.2),
        	genom_request( 'pr2SoftMotion', 'GotoQ', 
                        ['PR2', 0] + move_2.initcoords() ),
                wait(0.2),
                #genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_2.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
        	#wait(0.2),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_2.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                wait(0.2),
        	genom_request( 'pr2SoftMotion', 'GotoQ', 
                        ['PR2', 0] + move_3.initcoords() ),
                wait(0.2),
                #genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_3.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
        	#wait(0.2),
        	genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_3.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                wait(0.2),
        	genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + move_4.initcoords() ),
                wait(0.2),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_4.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                wait(0.2),
        	genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_4.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                wait(0.2),
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + move_5.initcoords() ),
                wait(0.2),
                #genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_5.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                #wait(0.2),
        	genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_5.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                wait(0.2),
                #genom_request( 'pr2SoftMotion', 'GotoQ',
                #        ['PR2', 0] + move_6.initcoords() ),
                #wait(0.2),
                #genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_6.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                #wait(0.2),
        	#genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_6.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                #wait(0.2),
                #genom_request( 'pr2SoftMotion', 'GotoQ',
                #        ['PR2', 0] + move_7.initcoords() ),
                #wait(0.2),
                #genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_7.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                #wait(0.2),
                #genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_7.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                #wait(0.2),
                genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + move_8.initcoords() ),
                wait(0.2),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_8.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                wait(0.2),
        	genom_request( 'pr2SoftMotion', 'TrackQ',
                       [ move_8.abspath(),"PR2SM_TRACK_FILE", "PR2"])
                #wait(0.2),
                #genom_request( 'pr2SoftMotion', 'GotoQ',
                #        ['PR2', 0] + move_9.initcoords() ),
                #wait(0.2),
                #genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_9.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                #wait(0.2),
        	#genom_request( 'pr2SoftMotion', 'TrackQ',
                #       [ move_9.abspath(),"PR2SM_TRACK_FILE", "PR2"]),
                #wait(0.2)

        ]
    
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

