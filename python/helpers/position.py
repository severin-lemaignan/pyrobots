from action import genom_request

def getabspose(object_name):

    actions = [ genom_request("spark", "GetJointAbsPose", object_name) ]

    return actions

def setabspose(object_name):

    res = getabspose(object_name)
    ans = res[1]
    x_goal, y_goal, z_goal = float(ans[3]), float(ans[4]), float(ans[5])

    return [x_goal, y_goal, z_goal]

def mypose():
    """
    Returns the current robot base pose in the map frame as a dictionary with
    [x,y,z,qx,qy,qz,qw].
    """
    import rospy
    from tf import TransformListener

    mytf = TransformListener()

    mytf.frameExists("/base_link") and mytf.frameExists("/map"):
            t = mytf.getLatestCommonTime("/base_link", "/map")
            position, quaternion = mytf.lookupTransform("/base_link", "/map", t)
            return dict(zip(["x","y","z","qx","qy","qz","qw"], position + quaternion))
    return None
    
    
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
    
    from helpers import places
    a = places.read()["STAGE_A"]
    b = places.read()["STAGE_B"]
    c = places.read()["STAGE_C"]
    d = places.read()["STAGE_D"]
    poly = [(a["x"], a["y"]), (b["x"], b["y"]), (c["x"], c["y"]), (d["x"], d["y"])]

    point = (target["x"], target["y"])

    return isin(point, poly)
