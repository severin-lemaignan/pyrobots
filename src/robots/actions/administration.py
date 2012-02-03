import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.action import action, genom_request
from robots.action import wait as basewait

@action
def setup_scenario(scenario):
    """ Allows to load a custom scene configuration ('sce' files) in SPARK
    at runtime.
    
    :param scenario: The complete path to a valid scenario file ('.sce')
    """
    return [genom_request("spark", "LoadSce", scenario)]

@action
def lock_object(object):
    """ Lock to position of an object in SPARK. It won't be updated
    anymore.
    
    :param object: The ID of the object to lock.
    """
    return [genom_request("spark", "SetObjectFixed", [object, "GEN_TRUE"])]

@action
def unlock_object(object):
    """ Unlock to position of an object in SPARK. It will be updated
    when seen.
    
    :param object: The ID of the object to unlock.
    """
    return [genom_request("spark", "SetObjectFixed", [object, "GEN_FALSE"])]


@action
def wait(seconds):
    """ This special action simply waits for a given amount of second before 
    sending the next action.
    
    :param seconds: the time to wait, in seconds
    """
    return [basewait(seconds)]

@action
def init():
    """ Initialize modules in correct order.
    """
    return [genom_request("pr2SoftMotion", "Init")]

