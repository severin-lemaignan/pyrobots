import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

isrosconfigured = False
try:
    import roslib; roslib.load_manifest('novela_actionlib')
    import rospy

    import actionlib
    import pr2_gripper_sensor_msgs.msg
    isrosconfigured = True
    
except ImportError: # Incorrect ROS setup!
    logger.warning("ROS is not configured!! Running in dummy mode")
    pass
    
from action import action, ros_request

###############################################################################
@action
def grab(robot, callback = None):
    
    client = None
    goal = None
    
    if isrosconfigured:
        #definition of the grab client
        client = actionlib.SimpleActionClient('r_gripper_sensor_controller/grab', pr2_gripper_sensor_msgs.msg.PR2GripperGrabAction)

        ok = client.wait_for_server()
        if not ok:
                #logger.error("Could not connect to the ROS client! Aborting action")
                print("Could not connect to the ROS client! Aborting action")
                return

        
        #definition of the grab goal
        goal = pr2_gripper_sensor_msgs.msg.PR2GripperGrabGoal()

        print (str (goal))
        
        if not client.wait_for_server():
            print("Could not connect to the ROS client! Aborting action")
        
        goal.command.hardness_gain = 0.09
        
    return [ros_request(client,
            goal,
            wait_for_completion = False if callback else True,
            callback = callback
            )]
            
###############################################################################
@action
def release(robot, callback = None):
    
    #definition of the release client
    client = actionlib.SimpleActionClient('r_gripper_sensor_controller/release', pr2_gripper_sensor_msgs.msg.PR2GripperReleaseAction)
    
    if not client.wait_for_server():
        print("Could not connect to the ROS client! Aborting action")
    
    #definition of the release goal
    goal = pr2_gripper_sensor_msgs.msg.PR2GripperReleaseGoal()
    
    
    #set the robot to release on a figner-side impact, fingerpad slip, or acceleration impact with hand/arm
    goal.command.event.trigger_conditions = goal.command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC
    #set the acceleration impact to trigger on to 5 m/s^2
    goal.command.event.acceleration_trigger_magnitude = 5.0
    #set our slip-gain to release on to .005
    goal.command.event.slip_trigger_magnitude = .01
    #ROS_INFO("Waiting for contact...")
    
    return [ros_request(client,
        goal,
        wait_for_completion = False if callback else True,
        callback = callback
        )]
            
###############################################################################	
@action
def detect(robot, callback = None):
    
    #definition of the detect client
    client = actionlib.SimpleActionClient('r_gripper_sensor_controller/event_detector', pr2_gripper_sensor_msgs.msg.PR2GripperEventDetectorAction)

    ok = client.wait_for_server()
    
    if not ok:
                #logger.error("Could not connect to the ROS client! Aborting action")
                print("Could not connect to the ROS client! Aborting action")
                return

        
    #definition of the detect goal
    goal = pr2_gripper_sensor_msgs.msg.PR2GripperEventDetectorGoal()
    
    goal.command.trigger_conditions = goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC
    goal.command.acceleration_trigger_magnitude = 5.0  # set the contact acceleration to n m/s^2
    goal.command.slip_trigger_magnitude = 0.5

    #ROS_INFO("Waiting for contact...")
    
    return [ros_request(client,
            goal,
            wait_for_completion = False if callback else True,
            callback = callback
        )]

###############################################################################	
@action
def detect_and_grab(robot, callback = None):

    actions = detect()

    actions += grab()

    return actions
