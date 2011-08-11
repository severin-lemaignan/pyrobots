from helpers import trajectory
from action import action, genom_request



@action
def gym():

	part_1 = Trajectory('gym_1')
	part_2 = Trajectory('gym_2')
	part_3 = Trajectory('gym_3')
	part_4 = Trajectory('gym_4')
	part_5 = Trajectory('gym_5')

        actions = [ genom_request( 'pr2SoftMotion', 'GotoQ',
                        ['PR2', 0] + part_1.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
			[ part_1.abspath(),'PR2SM_TRACK_FILE', 'PR2']),
                genom_request('pr2SoftMotion', 'GotoQ',
                        ['PR2, 0'] + part_2.initcoords() ),
		genom_request( 'pr2SoftMotion', 'TrackQ',
                        [ part_2.abspath(),'PR2SM_TRACK_FILE', 'PR2']),
                genom_request('pr2SoftMotion', 'GotoQ',
                        ['PR2, 0'] + part_3.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                        [ part_3.abspath(),'PR2SM_TRACK_FILE', 'PR2']),
                genom_request('pr2SoftMotion', 'GotoQ',
                        ['PR2, 0'] + part_4.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                        [ part_4.abspath(),'PR2SM_TRACK_FILE', 'PR2']),
                genom_request('pr2SoftMotion', 'GotoQ',
                        ['PR2, 0'] + part_5.initcoords() ),
                genom_request( 'pr2SoftMotion', 'TrackQ',
                        [ part_5.abspath(),'PR2SM_TRACK_FILE', 'PR2'])
		]
	return actions

