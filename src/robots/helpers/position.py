import logging; logger = logging.getLogger("ranger.position")

import math
import numpy
import transformations

class UnknownFrameError(RuntimeError):
    pass


FRAMES = ["map", "base_link", "eyes_link", "station", "beacon"]

POS_EPSILON = 0.1 # Min distance in meters from origin to be considered as 'in game'

class PoseManager:
    """ A pose is for us a dict {'x':x, 'y':y, 'z':z, 'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw, 'frame':frame},
    ie a (x, y, z) cartesian pose in meter interpreted in a specific reference 
    frame, and a quaternion describing the orientation of the object in radians.
    
    This class helps with:
     * converting from other convention to our convention,
     * converting back to other conventions.
     
     """
    
    def __init__(self, robot):
        self.robot = robot
   
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
       
        raise RuntimeError("Don't know what to do with pose as array %s" % str(pose))
    
    def normalize(self, pose):
        if isinstance(pose, list) or isinstance(pose, tuple):
            return self.normalizelist(pose)
        if isinstance(pose, dict):
            return self.normalizedict(pose)
        
        raise RuntimeError("normalize() takes either lists or dict as input.")
    
    def __getitem__(self, raw):
        """ Implements the PoseManager[] operator as an alias for PoseManager.get()
        """
        return self.get(raw)
        
    def get(self, raw):
        """ takes a loosly defined 'pose' as input and returns a properly formatted
        and normalized pose.
        
        Input may be:
         * a frame
         * an incomplete pose dictionary
         * a list or tuple (x,y,z), (x,y,z,frame) or (z,y,z,rx,ry,rz) or (x,y,z,qx,qy,qz,qw)
        """
        
        pose = None
        
        if isinstance(raw, basestring):
            pose = self.getabspose(raw)
            if pose:
                return self.normalize(pose)
        
            raise UnknownFrameError("Unknown object or frame '%s'" % raw)
        
        else:
            return self.normalize(raw)

    def myself(self):
        """
        Returns the current robot's pose, ie the pose of the ROS TF 'base_link'
        frame.
        """
        return self.getabspose("/base_link")

    def distance(self, pose1, pose2 = "base_link"):
        """ Returns the euclidian distance between two pyRobots poses.

        If the second pose is omitted, "base_link" is assumed (ie, distance
        between a pose and the robot itself).
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

    def getabspose(self, frame):
        """
        Returns the a frame pose in the /map frame.
        """
        frame = frame.lstrip("/")

        if frame == "base_link":
            return [self.robot.x, self.robot.y, 0.0, 0.0, 0.0, self.robot.theta]

        if frame == "map" or frame == "station":
            # the charging station is the origin of the map
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if frame == "eyes_link":
            # position of the (middle of) the eyes
            return self.inframe(
                    [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, "base_link"],
                    "map")

        if frame.startswith("beacon"):
            beacon_id = int(frame.split("_")[1])
            try:
                beacon = self.robot.beacons[beacon_id]
            except KeyError:
                raise UnknownFrameError("Beacon %s never seen" % beacon_id)

            if beacon.obsolete():
                raise UnknownFrameError("Beacon %s has obsolete position (not seen since long time)" % beacon_id)

            return self.inframe(
                    [beacon.x, beacon.y, 0.0, 0.0, 0.0, beacon.theta, "base_link"],
                    "map")

        raise UnknownFrameError("Frame %s does not exist." % frame)

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

        return {"x":float(x),
                "y":float(y),
                "z":float(z),
                "qx":float(qx),
                "qy":float(qy),
                "qz":float(qz),
                "qw":float(qw),
                "frame": frame}


    def pantilt(self, pose, ref="/base_link"):
        """
        Convert a xyz target to pan and tilt angles from a given 
        viewpoint.

        :param pose: the target pose
        :param ref: the reference frame (default to base_link)
        :returns: (pan, tilt) in radians, in ]-pi, pi]
        """
        pose = self.inframe(pose, ref)
        pan = self.normalize_angle(numpy.arctan2(pose['y'], pose['x']))
        tilt = self.normalize_angle(numpy.arctan2(pose['z'], pose['x']))
        return (pan,tilt)

    @staticmethod
    def normalize_angle(angle):
        """ Returns equivalent angle such as  -pi < angle <= pi
        """
        angle = angle % (2 * math.pi) # => angle > 0
        return float(angle if angle <= math.pi else (-math.pi + angle % math.pi))


    def isin(self, point,polygon):
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

