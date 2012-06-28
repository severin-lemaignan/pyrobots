import time
import logging
logger = logging.getLogger("robots." + __name__)

def nop(void, void2=None):
    pass

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

class Bring(Desire):
    def __init__(self, situation, robot):
        super(Bring, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        self.to = robot.knowledge[self._sit + " receivedBy *"]

        self.in_safe_nav_pose = True
    
    def findobject(self, obj, max_attempts = 4):
        """ Moves left and right the head to try to see an object.
        Uses the knowledge base to check the object visibily.

        :returns: True is the object is seen, False if it failed.
        """

        self._robot.look_at([1.0,0.0,0.5,"base_link"])

        time.sleep(3)

        attempts = 1
        while not (["myself looksAt " + obj] in self._robot.knowledge) \
              and attempts <= max_attempts:

            # Looks once left, once right
            self._robot.look_at([1.0, 0.6 * ((attempts % 2) * 2 - 1) , 0.5,"base_link"])
            time.sleep(3)
            attempts += 1

        self._robot.look_at([1.0,0.0,1.0,"base_link"])
        return (attempts <= max_attempts)



    def navprogress(self, progress):
        print(str(progress.percentage_covered) + "% of traj covered (" +  str(progress.distance_to_go) + "m to go).")
        if progress.distance_covered > 0.5: # 50cm off the initial position
            self._robot.manipose()
            self.in_safe_nav_pose = True

    def perform(self):
        super(Bring, self).perform()
        self._robot.manipose(nop)
        logger.info(str(self.doer) + " wants to bring " + str(self.objects) + " to " + str(self.to))
        obj = self.objects[0]
        if len(self.objects) > 1:
            logger.info("Let take care of " + obj + " for now.")
        
        loc = self._robot.knowledge[obj + " isAt *"]
        if not loc:
            logger.warning("No place found for " + obj + "! I can not bring it")
            return
        logger.info(obj + " is on " + loc[0] + ". Let go there.")

        self._robot.goto(loc[0])
        self._robot.extractpose(nop)
        if not self._robot.dock(): # docking fails if no obstacle is seen within 1m
            self._robot.translate(0.3)

        logger.info("Ok, destination reached. Let's try to see " + obj)
        
        ok = self.findobject(obj, max_attempts = 2)
        
        if not ok:
            logger.warning("I can not see the object " + obj + "! Giving up.")
            return
        logger.info("Ok, object found. Let's try to pick it.")
        self._robot.pick(obj)
        self._robot.extractpose()
        self._robot.translate(-0.2) # undock
        self._robot.manipose()

        self.in_safe_nav_pose = False

        logger.info("And now, hand it over to " + self.to[0])

        self._robot.handover(self.to[0], feedback = self.navprogress)
        self._robot.manipose()
        self._robot.goto("BASE")

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
        logger.info(str(self.doer) + " wants to look at " + str(self.objects[0]))
        self._robot.look_at(self.objects[0])

class Display(Desire):
    def __init__(self, situation, robot):
        super(Display, self).__init__(situation, robot)

        self.window = robot.knowledge[self._sit + " involves *"][0]

    def perform(self):
        super(Display, self).perform()

        logger.info("Let's try to display " + str(self.window))

        self._robot.display(self.window)

class Display(Desire):
    def __init__(self, situation, robot):
        super(Display, self).__init__(situation, robot)

        self.window = robot.knowledge[self._sit + " involves *"][0]

    def perform(self):
        super(Display, self).perform()

        logger.info("Let's try to display " + str(self.window))

        self._robot.display(self.window)

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
