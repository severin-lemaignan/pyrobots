""" This module codes the behaviour of the robot in case of
an incoming question from the human.
"""

import time
import logging
logger = logging.getLogger("robot." + __name__)
import threading

from robots.exception import RobotError, UnknownFrameError
from robots.behaviours.desires import Show

def nop(void, void2=None):
    pass


class Interrogation(object):
    """ This class encapsulates an InterrogativeState as formalized 
    in the OpenRobot ontology.
    
    An instance of an InterrogativeState is constructed by passing an existing
    desire ID.
    """
    def __init__(self, situation, robot):
        self._sit = situation
        self._robot = robot
        
        self.owners = robot.knowledge["* experiences %s" % self._sit]
        if not self.owners:
            raise NotExistingDesireError("I couldn't find anyone actually experiencing " + self._sit)
            
        #If only one owner, create an alias
        if len(self.owners) == 1:
            [self.owner] = self.owners
        else:
            self.owner = None

        self.answered = True if ("%s hasAnswer true" % self._sit) in robot.knowledge else False

        self.objects = robot.knowledge["%s hasObject *" % self._sit]
        #If only one object, create an alias
        if len(self.objects) == 1:
            [self.object] = self.objects
        else:
            self.object = None

        self.name = self.__class__.__name__

    def get_verbalization(self, timeout = 15):
        attempts = 0
        verbalisation = ""
        while not verbalisation:
            verbalisation = self._robot.knowledge["%s verbalisesTo *" % self._sit]
            if verbalisation:
                verbalisation = verbalisation[0]
            else:
                attempts +=1
                time.sleep(1)

            if attempts > timeout:
                break

        return verbalisation

    
    def perform(self):
        logger.info("Now performing " + self.name)
        self._process()

    def _process(self):
        """ Default processing: simply answer what the dialogs module
        says.
        """
        if self.owner:
            try:
                self._robot.look_at(self.owner)
            except UnknownFrameError: # the speaker is not in sight?
                pass

class Place(Interrogation):
    def __init__(self, situation, robot):
        super(Place, self).__init__(situation, robot)

    def _process(self):

        logger.info("The human is looking for the localization of %s." % self.objects)

        if self.answered:
            self._robot.look_at(self.object)
            #if ("myself reaches %s" % self.object) in robot.knowledge:
            #TODO: random sit ID
            show = Show(situation = "showplace",
                        robot = self._robot,
                        owners = self.owners,
                        performer = 'myself',
                        objects = self.objects,
                        receivers = self.owners)

            #show.perform()

            #else:
            #    #TODO: pointing!
            #    pass

            try:
                self._robot.look_at(self.owner)
            except UnknownFrameError:
                pass

        else:
            logger.info("I do not known the answer :-(")

class NotExistingQuestionTypeError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

def question_factory(sit, robot):

    aim = robot.knowledge["%s hasAim *" % sit]
    aim = aim[0].split("_question_aim")[0].capitalize()

    try:
        logger.info("Human has a question related to a %s. Let's handle it." % aim)
        interrogation = eval(aim)(sit, robot)
    except NameError:
        logger.warning("No specific handler for this type of question: \"%s\". Using generic one." % aim )
        interrogation = Interrogation(sit, robot)
    
    return interrogation
