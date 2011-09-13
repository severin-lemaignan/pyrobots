import json
import logging; logger = logging.getLogger("novela." + __name__)
logger.setLevel(logging.DEBUG)

source = '../share/novela_places.json'

_places = None

def read():
    global _places
    if not _places:
        logger.info("Reading symbolic places from " + source)
        f = open(source,'r')
        json_data=f.read()
        _places = json.loads(json_data)
    return _places
