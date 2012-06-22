import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.action import *
from robots.exception import RobotError
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
        
        if robot.hasROS():
            self.ros = ROSPositionKeeper()
        else:
            logger.warning("Initializing the PoseManager without ROS support." +\
                           "TF transformations and conversions between" +\
                           " quaternions and euler angles won't be available.")
            self.ros = None
            
        if robot.hasPocolibs() and robot.hasmodule('spark'):
            from robots.action import genom_request
            self.spark = SPARKPositionKeeper(robot)
        else:
            logger.warning("Initializing the PoseManager without SPARK support." +\
                           "Positions of SPARK objects won't be available.")
            self.spark = None
        
    def quaternion_from_euler(self, rx, ry, rz):
        if not self.ros:
            raise RobotError("ROS support required for conversions between quaternions" +\
            " and euler angles.")
        from tf import transformations
        return transformations.quaternion_from_euler(rx, ry, rz, 'sxyz')
    
    def euler_from_quaternion(self, rx, ry, rz):
        if not self.ros:
            raise RobotError("ROS support required for conversions between quaternions" +\
            " and euler angles.")
        from tf import transformations
        return transformations.euler_from_quaternion(pose['qx'], pose['qy'], pose['qz'], pose['qw'], 'sxyz')
        
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
            pose['qw'] = 0.0
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
       
        raise RobotError("Don't know what to do with pose as array %s" % pose)
    
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
            if self.ros:
                pose = self.ros.getabspose(raw)
            
            if not pose:
                #Trying with SPARK
                if self.spark:
                    pose = self.spark.getabspose(raw)
                if not pose:
                    raise RobotError("Unknown object or frame '%s'" % raw)
                else:
                    return self.normalize(pose)
            else:
                return self.normalize(pose)
            
        else:
            try:
                # SPARK tuple (object name, part)?
                obj, part = raw
                if isinstance(obj, basestring) and isinstance(part, basestring):
                    if self.spark:
                        pose = self.spark.getabspose(raw)
                    if not pose:
                        raise RobotError("Unknown object or part '%s'" % raw)
                    else:
                        return self.normalizelist(pose)
                else:
                    raise RobotError("Unable to recognize the pose %s" % raw)
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
        Returns the current PR2 base pose.
        """
        return self.get("base_link")

    @helper("poses")
    def human(self, human, part = 'Pelvis'):
        """
        Head -> part="HeadX"
        """
        # Where is the human?
        return self.get((human, part))
    
class ROSPositionKeeper:
    def __init__(self):
        self.isrosconfigured = True

        try:
            import rospy
            import tf
        except ImportError: # Incorrect ROS setup!
            self.isrosconfigured = False
            return

        self.tf = tf.TransformListener()

        try:
            self.tf.waitForTransform("/base_link", "/map", rospy.Time(), rospy.Duration(1.0))
        except tf.Exception: # likely a timeout
            logger.error("Timeout while waiting for TF transformations!"
                         " Did you initialize the PR2?\n ROS positions won't be available.")
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
        poseStamped.header.stamp = self.tf.getLatestCommonTime("/map", pose["frame"])
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

        if self.tf.frameExists(frame) and self.tf.frameExists("/map"):

            poseStamped = self.asROSpose(pose)
            newPoseStamped = self.tf.transformPose(frame, poseStamped)

            return {"x":newPoseStamped.pose.position.x,
                    "y":newPoseStamped.pose.position.y,
                    "z":newPoseStamped.pose.position.z,
                    "qx":newPoseStamped.pose.orientation.x,
                    "qy":newPoseStamped.pose.orientation.y,
                    "qz":newPoseStamped.pose.orientation.z,
                    "qw":newPoseStamped.pose.orientation.w,
                    "frame": frame}

        logger.error("Could not transform the pose from /map to the frame " + frame) #TODO: For some reason, the logger do not work
        return None


    def getabspose(self, frame):
        if not self.isrosconfigured:
            return None

        if self.tf.frameExists(frame) and self.tf.frameExists("/map"):
            t = self.tf.getLatestCommonTime("/map", frame)
            position, quaternion = self.tf.lookupTransform("/map", frame, t)
            return dict(zip(["x","y","z","qx","qy","qz","qw","frame"], position + quaternion + ("map",)))

        logger.error("Could not read the pose of " + frame + " in /map") #TODO: For some reason, the logger do not work
        return None


    def xyz2pantilt(self, x, y, z, frame = "map"):
        """ BROKEN! Computation of pan and tilt is wrong.

        Convert a xyz target to pan and tilt angles for the head.

        :param x: the x coordinate
        :param y: the y coordinate
        :param z: the z coordinate
        :param frame: the frame in which coordinates are interpreted
        :returns: (pan, tilt) in radians
        """

        import numpy
        import math
        #from geometry_msgs.msg import PointStamped
        #import rospy
        #from tf import transformations

        #goal = PointStamped()
        #goal.header.frame_id =frame
        #goal.header.stamp = rospy.Time(0);
        #goal.point.x = x
        #goal.point.y = y
        #goal.point.z = z
        #frame = 'map'

        #goalInFrame = PointStamped()
        #self.tf.waitForTransform(frame, 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
        #goalInFrame = self.tf.transformPoint('base_footprint',goal)

        #x = goalInFrame.point.x
        #y = goalInFrame.point.y
        #z = goalInFrame.point.z
        
        ##self.tf = TransformListener()
        ## +0.067 to take into account the translation between base_footprint and head_pan_link
        #pan= numpy.arctan2(y, x+0.067)
        #self.tf.waitForTransform(frame, 'head_pan_link', rospy.Time(0), rospy.Duration(4.0))
        #(transTilt,rotTilt) = self.tf.lookupTransform('head_pan_link', frame, rospy.Time(0))
        #matBaseTilt= numpy.dot(transformations.translation_matrix(transTilt), transformations.quaternion_matrix(rotTilt))
        #xyzTilt = tuple(numpy.dot(matBaseTilt, numpy.array([x, y, z, 1.0])))[:3]
        #tilt= numpy.arctan2(-xyzTilt[2], numpy.sqrt(math.pow(xyzTilt[0],2)+math.pow(xyzTilt[1],2)))
    
        t = self.tf.getLatestCommonTime("/head_pan_link", frame)
        base, quaternion = self.tf.lookupTransform("/head_pan_link", frame, t)
        
        target = [base[0] + x, base[1] + y, base[2] + z]
        pan = numpy.arctan2(target[0], target[1])
        tilt = numpy.arctan2(target[0], target[2])
        
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
        if ok != 'OK':
            # Object probably does not exist
            return None
        yaw, pitch, roll, x, y, z = [float(x) for x in res]
        
        if abs(x) < POS_EPSILON and abs(y) < POS_EPSILON:
            # Still at origin
            logger.info("Object still at origin. Considered as 'not in game'")
            return None
            
        return (x, y, z, roll, pitch, yaw)
        

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

