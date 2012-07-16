import os
import json
import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

#hack that tries to find out the current prefix and then the data directory
DATA_DIR = os.path.abspath(__file__).split('lib')[0] + 'share/pyrobots/'

_places = None

def read(filename):
    global _places
    if not _places:
        _places = {}
        sources = [DATA_DIR + filename]
        for source in sources:
            logger.info("Reading symbolic places from " + source)
            with open(source,'r') as f:
                json_data=f.read()
            _places.update(json.loads(json_data))
    return _places

