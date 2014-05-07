__version__="1.99"

try:
    from robots.robot import GenericRobot
except ImportError:
    # import will fail the first time pyRobots is installed because this file
    # is executed from setup.py to retrieve the version
    pass

