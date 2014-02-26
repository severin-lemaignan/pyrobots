import logging; logger = logging.getLogger("robot." + __name__)

import math
import numpy
import transformations

from robots.lowlevel import *
from robots.action import *
from robots.exception import UnknownFrameError, RobotError
import places

POS_EPSILON = 0.1 # Min distance in meters from origin to be considered as 'in game'

class PoseManager:
    """ A pose is for us a dict {'x':x, 'y':y, 'z':z, 'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw, 'frame':frame},
    ie a (x, y, z) cartesian pose in meter interpreted in a specific reference 
    frame, and a quaternion describing the orientation of the object in radians.
    
    This class helps with:
     * retrieving pose using ROS TF, SPARK or a static library of positions,
     * converting from other convention to our convention,
     * converting back to other conventions.
     
     """
    
    def __init__(self, robot):
        
        if robot.supports(ROS):
            self.ros = ROSPositionKeeper()
        else:
            logger.warning("Initializing the PoseManager without ROS support." +\
                           "TF transformations won't be available.")
            self.ros = None
            
        if robot.supports(POCOLIBS) and robot.hasmodule('spark'):
            from robots.action import genom_request
            self.spark = SPARKPositionKeeper(robot)
        else:
            logger.warning("Initializing the PoseManager without SPARK support." +\
                           "Positions of SPARK objects won't be available.")
            self.spark = None

        if robot.supports(NAOQI):
            from robots.action import naoqi_request
            self.naoqi = NAOqiPositionKeeper(robot, self)
        else:
            logger.warning("Initializing the PoseManager without NAOqi support." +\
                           "Positions of NAOqi objects/bodies won't be available.")
            self.naoqi = None

        
   
    @staticmethod
    def quaternion_from_euler(rx, ry, rz):
        return transformations.quaternion_from_euler(rx, ry, rz, 'sxyz')
   
    def normalizedict(self, pose):
        if not 'x' in pose:
            pose['x'] = 0.0
        else:
            pose['x'] = float(pose['x'])
            
        if not 'y' in pose:
            pose['y'] = 0.0
        else:
            pose['y'] = float(pose['y'])
            
        if not 'z' in pose:
            pose['z'] = 0.0
        else:
            pose['z'] = float(pose['z'])
            
        if not 'qx' in pose:
            pose['qx'] = 0.0
        else:
            pose['qx'] = float(pose['qx'])
        
        if not 'qy' in pose:
            pose['qy'] = 0.0
        else:
            pose['qy'] = float(pose['qy'])
        
        if not 'qz' in pose:
            pose['qz'] = 0.0
        else:
            pose['qz'] = float(pose['qz'])
            
        if not 'qw' in pose:
            pose['qw'] = 1.0
        else:
            pose['qw'] = float(pose['qw'])
            
        if not 'frame' in pose:
            pose['frame'] = 'map'
        return pose
        
    def normalizelist(self, pose):
        if len(pose) == 3:
            x,y,z = pose
            return self.normalizedict({'x':x, 'y':y, 'z':z})

        if len(pose) == 4 and isinstance(pose[3], basestring): 
            # we assume the last arg is the frame
            x,y,z, frame = pose
            return self.normalizedict({'x':x, 'y':y, 'z':z, 'frame': frame})
            
        if len(pose) == 6:
            x,y,z, rx, ry, rz = pose
            qx,qy,qz,qw = self.quaternion_from_euler(rx, ry, rz)
            return self.normalizedict({'x':x, 'y':y, 'z':z, 'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw})
            
        if len(pose) == 7:
            if isinstance(pose[6], basestring): 
                # we assume the last arg is the frame
                x,y,z, rx, ry, rz, frame = pose
                qx,qy,qz,qw = self.quaternion_from_euler(rx, ry, rz)
                return self.normalizedict({'x':x, 'y':y, 'z':z, 'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw, 'frame': frame})
            else:
                x,y,z, qx, qy, qz, qw = pose
                return self.normalizedict({'x':x, 'y':y, 'z':z, 'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw})
        
        if len(pose) == 8 and isinstance(pose[7], basestring): 
            # we assume the last arg is the frame
            x,y,z, qx, qy, qz, qw, frame = pose
            return self.normalizedict({'x':x, 'y':y, 'z':z, 'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw, 'frame': frame})
       
        raise RobotError("Don't know what to do with pose as array %s" % str(pose))
    
    def normalize(self, pose):
        if isinstance(pose, list) or isinstance(pose, tuple):
            return self.normalizelist(pose)
        if isinstance(pose, dict):
            return self.normalizedict(pose)
        
        raise RobotError("normalize() takes either lists or dict as input.")
    
    def __getitem__(self, raw):
        """ Implements the PoseManager[] operator as an alias for PoseManager.get()
        """
        return self.get(raw)
        
    def get(self, raw):
        """ takes a loosly defined 'pose' as input and returns a properly formatted
        and normalized pose.
        
        Input may be:
         * a SPARK entity (either an object name of a pair (object name, part name))
         * a ROS TF frame
         * an incomplete pose dictionary
         * a list or tuple (x,y,z), (x,y,z,frame) or (z,y,z,rx,ry,rz) or (x,y,z,qx,qy,qz,qw)
        """
        
        pose = None
        
        if isinstance(raw, basestring):
            
            p = places.places()
            # Is it a known symbolic place?
            if raw in p.keys():
                return self.get(p[raw]) # normalize it
            
            #Either a SPARK object name or a ROS TF frame
            #Trying with ROS
            if self.ros:
                pose = self.ros.getabspose(raw)
                if pose:
                    return self.normalize(pose)

            #Trying with SPARK
            if self.spark:
                pose = self.spark.getabspose(raw)
                if pose:
                    return self.normalize(pose)

             #Trying with NAOqi
            if self.naoqi:
                pose = self.naoqi.getabspose(raw)
                if pose:
                    return self.normalize(pose)
            
            raise UnknownFrameError("Unknown object or frame '%s'" % raw)
        
        else:
            try:
                # SPARK tuple (object name, part)?
                obj, part = raw
                if isinstance(obj, basestring) and isinstance(part, basestring):
                    if self.spark:
                        pose = self.spark.getabspose(raw)
                    if not pose:
                        raise UnknownFrameError("Unknown object or part '%s'" % raw)
                    else:
                        return self.normalizelist(pose)
                else:
                    raise UnknownFrameError("Unable to recognize the pose %s" % raw)
            except TypeError:
                raise RobotError("Unable to process the pose '%s'" % raw)
            except ValueError:
                # List or dict (x,y,z,...)?
                try:
                    return self.normalize(raw)
                except RobotError as re:
                    raise RobotError("Unable to process the pose '%s' (original error was:%s)" % (raw, str(re)))

    @helper("poses")
    def myself(self):
        """
        Returns the current robot's pose, ie the pose of the ROS TF 'base_link'
        frame.
        """
        if self.ros:
            try:
                return self.ros.getabspose("base_link")
            except RobotError as e:
                if "base_link" in e.value:
                    logger.error("The robot is not localized!")
                    return None
        return None

    @helper("poses")
    def human(self, human, part = 'Pelvis'):
        """
        Returns the pose of a human, either using Pocolibs' SPARK or ROS TF.

        If using SPARK, a body part may be specified (default to 'Pelvis'). For
        the head, the part is "HeadX".

        If using ROS, we return the pose of a TF frame called 'face_<human>' or
        'human_<human>'.

        """
        # Where is the human?
        if self.spark:
            return self.spark.getabspose(human, part)
        elif self.ros:
            pose = self.ros.getabspose("face_%s" % human)
            return pose if pose else self.ros.getabspose("human_%s" % human)
        else:
            logger.error("Human localization currently requires SPARK or ROS")
            return None

    @helper("poses")
    def distance(self, pose1, pose2):
        """ Returns the euclidian distance between two pyRobots poses.
        """
        p1 = self.get(pose1)
        p2 = self.get(pose2)

        return math.sqrt(math.pow(p2["x"] - p1["x"], 2) + \
                         math.pow(p2["y"] - p1["y"], 2) + \
                         math.pow(p2["z"] - p1["z"], 2))

    def _xyz_to_mat44(self, pos):
        return transformations.translation_matrix((pos['x'], pos['y'], pos['z']))

    def _xyzw_to_mat44(self, ori):
        return transformations.quaternion_matrix((ori['qx'], ori['qy'], ori['qz'], ori['qw']))

    def _to_mat4(self, pose):
        return numpy.dot(self._xyz_to_mat44(pose), self._xyzw_to_mat44(pose))

    @helper("poses")
    def inframe(self, pose, frame):
        """ Transform a pose from one frame to another one.

        Uses transformation matrices. Could be refactored to use directly
        quaternions.
        """
        pose = self.get(pose)

        if pose['frame'] == "map":
            orig = numpy.identity(4)
        else:
            orig = self._to_mat4(self.get(pose["frame"]))

        if frame == "map":
            dest = numpy.identity(4)
        else:
            dest = numpy.linalg.inv(self._to_mat4(self.get(frame)))

        poseMatrix = self._to_mat4(pose)

        transf = numpy.dot(dest, orig)
        transformedPose = numpy.dot(transf, poseMatrix)

        qx,qy,qz,qw = transformations.quaternion_from_matrix(transformedPose)
        x,y,z = transformations.translation_from_matrix(transformedPose)

        return {"x":x,
                "y":y,
                "z":z,
                "qx":qx,
                "qy":qy,
                "qz":qz,
                "qw":qw,
                "frame": frame}


    
class ROSPositionKeeper:
    def __init__(self):
        self.isrosconfigured = True

        try:
            import rospy
            import tf
        except ImportError: # Incorrect ROS setup!
            self.isrosconfigured = False
            return

        self.tf = tf.TransformListener(True, rospy.Duration(10)) # interpolation, cache duration

        try:
            self.tf.waitForTransform("/base_link", "/map", rospy.Time(), rospy.Duration(1.0))
        except tf.Exception: # likely a timeout
            logger.error("Timeout while waiting for a TF transformation between /map "
                         "and /base_link. ROS positions won't be available.")
            self.isrosconfigured = False
            return

    @tested("14/06/2012")
    @helper("poses.ros")
    def asROSpose(self, pose):
        """ Returns a ROS PoseStamped from a pyRobots pose.

        :param pose: a standard pyRobots pose (SPARK id, TF frame, [x,y,z],
        [x,y,z,rx,ry,rz], [x,y,z,qx,qy,qw,qz], {'x':..., 'y':...,...})

        :return: the corresponding ROS PoseStamped
        """

        from geometry_msgs.msg import PoseStamped
        import rospy

        poseStamped = PoseStamped()

        poseStamped.header.frame_id = pose["frame"]
        poseStamped.header.stamp = rospy.Time()
        poseStamped.pose.position.x = pose["x"]
        poseStamped.pose.position.y = pose["y"]
        poseStamped.pose.position.z = pose["z"]
        poseStamped.pose.orientation.x = pose["qx"]
        poseStamped.pose.orientation.y = pose["qy"]
        poseStamped.pose.orientation.z = pose["qz"]
        poseStamped.pose.orientation.w = pose["qw"]

        return poseStamped

    def inframe(self, pose, frame):
        """ Transforms a given pose in the given frame.
        """
        if not self.isrosconfigured:
            return None


        poseStamped = self.asROSpose(pose)
        poseStamped.header.stamp = self.tf.getLatestCommonTime(pose["frame"], frame)
        newPoseStamped = self.tf.transformPose(frame, poseStamped)

        return {"x":newPoseStamped.pose.position.x,
                "y":newPoseStamped.pose.position.y,
                "z":newPoseStamped.pose.position.z,
                "qx":newPoseStamped.pose.orientation.x,
                "qy":newPoseStamped.pose.orientation.y,
                "qz":newPoseStamped.pose.orientation.z,
                "qw":newPoseStamped.pose.orientation.w,
                "frame": frame}



    def getabspose(self, frame):
        if not self.isrosconfigured:
            return None

        if not self.tf.frameExists("map"):
            logger.fatal("Lost our map!!")	
            return None
        if self.tf.frameExists(frame):
            t = self.tf.getLatestCommonTime("map", frame)
            position, quaternion = self.tf.lookupTransform("map", frame, t)
            return dict(zip(["x","y","z","qx","qy","qz","qw","frame"], position + quaternion + ("map",)))

        logger.debug("No such frame " + frame + " in TF")
        return None


    def xyz2pantilt(self, pose, headframe="/head_pan_link"):
        """
        Convert a xyz target to pan and tilt angles for the head.

        :param headframe: the frame of the head
        :returns: (pan, tilt) in radians
        """
        pose = self.inframe(pose, headframe)
        pan = numpy.arctan2(pose['y'], pose['x'])
        tilt = numpy.arctan2(pose['z'], pose['x'])
        
        logger.debug("Computed head pan: %s, tilt: %s" % (pan, tilt))
        return (pan,tilt)

class SPARKPositionKeeper:
    def __init__(self, robot):
        self.robot = robot
        
    def getrelativepose(self, robot1, robot2):

        raw = robot.execute([genom_request("spark", "GetRobotPoseRelativeToAnotherRobot", [robot1, robot2])])
        return _process_result(raw)

    def getabspose(self, obj, part = "HeadX"): #HeadX is useful to find the human head. Ignored by SPARK for other objects.

        raw = self.robot.execute([genom_request("spark", "GetJointAbsPose", [obj, part])])
        return self._process_result(raw)
        
    def _process_result(self, raw):
        
        ok, res = raw
        if not ok:
            # Object probably does not exist
            return None
        yaw, pitch, roll, x, y, z = [float(x) for x in res]
        
        if abs(x) < POS_EPSILON and abs(y) < POS_EPSILON:
            # Still at origin
            logger.info("Object still at origin. Considered as 'not in game'")
            return None
            
        return (x, y, z, roll, pitch, yaw)
        

class NAOqiPositionKeeper:
    def __init__(self, robot, posemanager):
        from naoqi import motion

        self.FRAME_ROBOT = motion.FRAME_ROBOT
        self.FRAME_WORLD = motion.FRAME_WORLD
        self.robot = robot

        self.posemanager = posemanager

        ok, self.bodies = self.robot.execute([naoqi_request("motion", "getBodyNames", ["Body"])])
        if not ok:
            logger.error("Can not connect to NAOqi motion proxy! pyRobots is likely not able to work with NAOqi.")
        ok, sensors = self.robot.execute([naoqi_request("motion", "getSensorNames", ["Body"])])
        
        self.bodies += sensors
        self.bodies += ["Torso", "Head", "LArm", "RArm", "LLeg", "RLeg"]


        logger.debug("List of NAOqi bodies: %s" % str(self.bodies))

    @helper("poses.naoqi")
    def as6Dpose(self, pose):
        wx, wy, wz = transformations.euler_from_quaternion([pose['qx'], pose['qy'], pose['qz'], pose['qw']], axes= 'sxyz')

        return [pose['x'], pose['y'], pose['z'], wx, wy, wz]

 
    @helper("poses.naoqi")
    def xyz2pantilt(self, pose):
        """
        :param x,y,z: in meters
        :returns: (pan, tilt) in radians
        """
        pose = self.posemanager.inframe(pose, 'Head')

        x = pose['x']
        y = pose['y']
        z = pose['z']

        pan = numpy.arctan2(y, x)
        tilt = numpy.arctan2(z, x)

        logger.debug("Computed head pan: %s, tilt: %s" % (pan, tilt))
        return (pan,tilt)



    def getabspose(self, obj):

        # On NAO, the 'base_link' is the torso
        if obj == "base_link":
            obj = "Torso"

        if obj not in self.bodies:
            return None

        ok, res = self.robot.execute([naoqi_request("motion", "getPosition", [obj, self.FRAME_WORLD, True])])
        
        if not ok:
            logger.warning("Unable to get the position of %s" % obj)
            return None

        return res

def isin(point,polygon):
    """
    Determines if a 2D point is inside a given 2D polygon or not.
    
    :param point: a (x,y) pair
    :param polygon: a list of (x,y) pairs.
    
    Copied from: http://www.ariel.com.au/a/python-point-int-poly.html
    """

    x,y = point
    n = len(polygon)
    inside =False

    p1x,p1y = polygon[0]
    for i in range(n+1):
        p2x,p2y = polygon[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

def isonstage(target):
    """
    Returns true if target is currently on stage.

    The stage coordinates are read from the symbolic positions library
    (STAGE_{A|B|C|D])

    :param target: an object that responses to target["x"] and target["y"]
    """
    
    if not target:
        return False
        
    from helpers import places
    
    a = places.read()["STAGE_A"]
    b = places.read()["STAGE_B"]
    c = places.read()["STAGE_C"]
    d = places.read()["STAGE_D"]
    
    poly = [(a["x"], a["y"]), (b["x"], b["y"]), (c["x"], c["y"]), (d["x"], d["y"])]

    point = (target["x"], target["y"])

    return isin(point, poly)

