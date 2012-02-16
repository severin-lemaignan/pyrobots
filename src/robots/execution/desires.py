import logging
logger = logging.getLogger("robots." + __name__)

class Desire(object):
    """ This class encapsulates a desire (or goal) as formalized in the
    OpenRobot ontology.
    
    An instance of Desire is constructed by passing an existing desire ID.
    """
    def __init__(self, situation, oro, robot):
        self._sit = situation
        self._oro = oro
        self._robot = robot
        
        self.owners = oro["* desires " + self._sit]
        if not self.owners:
            raise NotExistingDesireError("I couldn't find anyone actually desiring " + self._sit)
            
        #If only one owner, create an alias
        if len(self.owners) == 1:
            [self.owner] = self.owners
        else:
            self.owner = None
        
        #TODO: for now, we only keep the first performer (useful when several equivalent instances are returned)
        self.performer = oro[self._sit + " performedBy *"][0]
        if not self.performer:
            raise NoPerformerDesireError("I couldn't find anyone to perform " + self._sit)
        
        self.name = self.__class__.__name__
    
    def perform(self):
        if "myself" in self.performer:
            logger.info("Now performing " + self.name)
            self._oro.add(["myself currentlyPerforms " + self._sit], "EPISODIC")

class Move(Desire):
    def __init__(self, situation, oro, robot):
        super(Move, self).__init__(situation, oro, robot)
        
        self.to = oro[self._sit + " hasGoal *"]

    def perform(self):
        logger.info("Moving to: " + str(self.to))
        self._robot.goto(self.to[0]))
        super(Move, self).perform()

class Get(Desire):
    def __init__(self, situation, oro, robot):
        super(Get, self).__init__(situation, oro, robot)
        
        self.objects = oro[self._sit + " actsOnObject *"]
    
    def perform(self):
        logger.info("Wanna get smthg: " + str(self.objects))
        super(Get, self).perform()

class Show(Desire):
    def __init__(self, situation, oro, robot):
        super(Show, self).__init__(situation, oro, robot)
        
        self.objects = oro[self._sit + " actsOnObject *"]
        self.doer = oro[self._sit + " performedBy *"]
        self.to = oro[self._sit + " receivedBy *"]
    
    def perform(self):
        logger.info(str(self.doer) + " wants to show " + str(self.objects) + " to " + str(self.to))
        
        self._robot.show(self.doer[0], self.objects[0], self.to[0]))
        super(Show, self).perform()
    
class Give(Desire):
    def __init__(self, situation, oro, robot):
        super(Give, self).__init__(situation, oro, robot)
        
        self.objects = oro[self._sit + " actsOnObject *"]
        self.doer = oro[self._sit + " performedBy *"]
        self.to = oro[self._sit + " receivedBy *"]
    
    def perform(self):
        logger.info(str(self.doer) + " wants to give " + str(self.objects) + " to " + str(self.to))
        
        self._robot.give(self.doer[0], self.objects[0], self.to[0]))

        super(Give, self).perform()

class Hide(Desire):
    def __init__(self, situation, oro, robot):
        super(Hide, self).__init__(situation, oro, robot)
        
        self.objects = oro[self._sit + " actsOnObject *"]
        self.doer = oro[self._sit + " performedBy *"]
        self.to = oro[self._sit + " receivedBy *"]
    
    def perform(self):
        logger.info(str(self.doer) + " wants to hide " + str(self.objects) + " to " + str(self.to))
        
        self._robot.hide(self.doer[0], self.objects[0], self.to[0])

        super(Hide, self).perform()

class Look(Desire):
    def __init__(self, situation, oro, robot):
        super(Look, self).__init__(situation, oro, robot)
        
        self.objects = oro[self._sit + " actsOnObject *"]
        self.doer = oro[self._sit + " performedBy *"]
    
    def perform(self):
        logger.info(str(self.doer) + " wants to look at " + str(self.objects))
        logger.warning("Currently hard-coded to x,y,z")
        
        self._robot.look_at_xyz_with_moveHead(5.5, -5, 1, "map")

        super(Look, self).perform()

class NotExistingDesireTypeError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
        
class NotExistingDesireError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class NoPerformerDesireError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

def desire_factory(sit, oro, robot):

    act = oro.getDirectClassesOf(sit).keys()    
    try:
        [action] = act
    except ValueError:
        raise NotExistingDesireTypeError(sit + " matches several actions ( " + \
                str(act) + ")! Did you forget to assert that atomic action " + \
                "are subclasses of PurposefulAction?")
    
    try:
        goal = eval(action)(sit, oro, robot)
    except NameError:
        raise NotExistingDesireTypeError("I don't know the action \"" + action + "\"!")
    
    return goal
