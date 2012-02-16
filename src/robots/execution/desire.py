import logging
logger = logging.getLogger("supervision")

from genomactionlibrary import GenomAction

class Desire(object):
    """ This class encapsulates a desire (or goal) as formalized in the
    OpenRobot ontology.
    
    An instance of Desire is constructed by passing an existing desire ID.
    """
    def __init__(self, situation, oro, actionQueue):
        self._sit = situation
        self._oro = oro
        self._action_queue = actionQueue
        
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
    def __init__(self, situation, oro, actionQueue):
        super(Move, self).__init__(situation, oro, actionQueue)
        
        self.to = oro[self._sit + " hasGoal *"]

class Get(Desire):
    def __init__(self, situation, oro, actionQueue):
        super(Get, self).__init__(situation, oro, actionQueue)
        
        self.objects = oro[self._sit + " actsOnObject *"]
    
    def perform(self):
        logger.info("Wanna get smthg: " + str(self.objects))
        super(Get, self).perform()

class Show(Desire):
    def __init__(self, situation, oro, actionQueue):
        super(Show, self).__init__(situation, oro, actionQueue)
        
        self.objects = oro[self._sit + " actsOnObject *"]
        self.doer = oro[self._sit + " performedBy *"]
        self.to = oro[self._sit + " receivedBy *"]
    
    def perform(self):
        logger.info(str(self.doer) + " wants to show " + str(self.objects) + " to " + str(self.to))
        
        self._action_queue.put(GenomAction.show(self.doer[0], self.objects[0], self.to[0]))
        super(Show, self).perform()
    
class Give(Desire):
    def __init__(self, situation, oro, actionQueue):
        super(Give, self).__init__(situation, oro, actionQueue)
        
        self.objects = oro[self._sit + " actsOnObject *"]
        self.doer = oro[self._sit + " performedBy *"]
        self.to = oro[self._sit + " receivedBy *"]
    
    def perform(self):
        logger.info(str(self.doer) + " wants to give " + str(self.objects) + " to " + str(self.to))
        
        self._action_queue.put(GenomAction.give(self.doer[0], self.objects[0], self.to[0]))

        super(Give, self).perform()

class Hide(Desire):
    def __init__(self, situation, oro, actionQueue):
        super(Hide, self).__init__(situation, oro, actionQueue)
        
        self.objects = oro[self._sit + " actsOnObject *"]
        self.doer = oro[self._sit + " performedBy *"]
        self.to = oro[self._sit + " receivedBy *"]
    
    def perform(self):
        logger.info(str(self.doer) + " wants to hide " + str(self.objects) + " to " + str(self.to))
        
        self._action_queue.put(GenomAction.hide(self.doer[0], self.objects[0], self.to[0]))

        super(Give, self).perform()

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

def desire_factory(sit, oro, actionQueue):

    act = oro.getDirectClassesOf(sit).keys()    
    try:
        [action] = act
    except ValueError:
        raise NotExistingDesireTypeError(sit + " matches several actions ( " + \
                str(act) + ")! Did you forget to assert that atomic action " + \
                "are subclasses of PurposefulAction?")
    
    try:
        goal = eval(action)(sit, oro, actionQueue)
    except NameError:
        raise NotExistingDesireTypeError("I don't know the action \"" + action + "\"!")
    
    return goal
