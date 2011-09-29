import logging; logger = logging.getLogger("novela." + __name__)
logger.setLevel(logging.DEBUG)

from action import action, genom_request

@action
def setup_scenario(scenario):
    return [genom_request("spark", "LoadSce", scenario)]


