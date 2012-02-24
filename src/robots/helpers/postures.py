import os
import json
import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

#hack that tries to find out the current prefix and then the data directory
DATA_DIR = os.path.abspath(__file__).split('lib')[0] + 'share/pyrobots/'

#TODO: make something more generic!!
sources = [DATA_DIR + 'pr2_postures.json']
sources += [DATA_DIR + 'poses_hri_expe_feb2012.json']

_places = None

def read():
    global _places
    if not _places:
	_places = {}
	for source in sources:
        	logger.info("Reading symbolic places from " + source)
		with open(source,'r') as f:
			json_data=f.read()
		_places.update(json.loads(json_data))
    return _places

def load(source):
    global _places
    _places = {}
    logger.info("Reading symbolic places from " + source)
    with open(source,'r') as f:
	json_data=f.read()
	_places = json.loads(json_data)
    return _places

