import logging
logger = logging.getLogger("robots." + __name__)

class Desire(object):
    """ This class encapsulates a desire (or goal) as formalized in the
    OpenRobot ontology.
    
    An instance of Desire is constructed by passing an existing desire ID.
    """
    def __init__(self, situation, robot):
        self._sit = situation
        self._robot = robot
        
        self.owners = robot.knowledge["* desires " + self._sit]
        if not self.owners:
            raise NotExistingDesireError("I couldn't find anyone actually desiring " + self._sit)
            
        #If only one owner, create an alias
        if len(self.owners) == 1:
            [self.owner] = self.owners
        else:
            self.owner = None
        
        #TODO: for now, we only keep the first performer (useful when several equivalent instances are returned)
        self.performer = robot.knowledge[self._sit + " performedBy *"][0]
        if not self.performer:
            raise NoPerformerDesireError("I couldn't find anyone to perform " + self._sit)
        
        self.name = self.__class__.__name__
    
    def perform(self):
        if "myself" in self.performer:
            logger.info("Now performing " + self.name)
            self._robot.knowledge.add(["myself currentlyPerforms " + self._sit], "EPISODIC")

class Move(Desire):
    def __init__(self, situation, robot):
        super(Move, self).__init__(situation, robot)
        
        self.to = robot.knowledge[self._sit + " hasGoal *"]

    def perform(self):
        super(Move, self).perform()
        logger.info("Moving to: " + str(self.to))
        self._robot.goto(self.to[0])

class Get(Desire):
    def __init__(self, situation, robot):
        super(Get, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
    
    def perform(self):
        super(Get, self).perform()
        logger.info("Wanna get smthg: " + str(self.objects))
        self._robot.take(self.objects[0])

class Show(Desire):
    def __init__(self, situation, robot):
        super(Show, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        self.to = robot.knowledge[self._sit + " receivedBy *"]
    
    def perform(self):
        super(Show, self).perform()
        logger.info(str(self.doer) + " wants to show " + str(self.objects) + " to " + str(self.to))
        
        self._robot.show(self.objects[0], self.to[0])
    
class Give(Desire):
    def __init__(self, situation, robot):
        super(Give, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        self.to = robot.knowledge[self._sit + " receivedBy *"]
    
    def perform(self):
        super(Give, self).perform()
        logger.info(str(self.doer) + " wants to give " + str(self.objects) + " to " + str(self.to))
        
        self._robot.give(self.objects[0], self.to[0])


class Hide(Desire):
    def __init__(self, situation, robot):
        super(Hide, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        self.to = robot.knowledge[self._sit + " receivedBy *"]
    
    def perform(self):
        super(Hide, self).perform()
        logger.info(str(self.doer) + " wants to hide " + str(self.objects) + " to " + str(self.to))
        
        self._robot.hide(self.objects[0], self.to[0])


class Look(Desire):
    def __init__(self, situation, robot):
        super(Look, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " hasGoal *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
    
    def perform(self):
        super(Look, self).perform()
        logger.info(str(self.doer) + " wants to look at " + str(self.objects))
        logger.warning("Currently hard-coded to x,y,z")
        
        self._robot.look_at([5.5, -5, 1, "map"])


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

def desire_factory(sit, robot):

    act = robot.knowledge.getDirectClassesOf(sit).keys()    
    try:
        [action] = act
    except ValueError:
        raise NotExistingDesireTypeError(sit + " matches several actions ( " + \
                str(act) + ")! Did you forget to assert that atomic action " + \
                "are subclasses of PurposefulAction?")
    
    try:
        goal = eval(action)(sit, robot)
    except NameError:
        raise NotExistingDesireTypeError("I don't know the action \"" + action + "\"!")
    
    return goal
