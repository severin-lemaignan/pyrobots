import logging; logger = logging.getLogger("novela." + __name__)
logger.setLevel(logging.DEBUG)

from action import action, genom_request
from action import wait as basewait

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
    """ Unlock to position of an object in SPARK. It won't be updated
    anymore.
    
    :param object: The ID of the object to lock.
    """
    return [genom_request("spark", "SetObjectFixed", [object, "GEN_FALSE"])]


@action
def wait(seconds):
	""" This special action simply waits for a given amount of second before 
	sending the next action.
	"""
	return [basewait(seconds)]

