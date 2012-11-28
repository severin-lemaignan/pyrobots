import time
import logging
import threading
logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

from robots.actions.manipulation import haspickedsmthg
from robots.helpers import places

from robots.exception import RobotError, UnknownFrameError

from robots.action import genom_request

def nop(void, void2=None):
    pass

def lookforobject(robot, obj, max_attempts = 4, mode = "wide_stereo"):
    """ Moves left and right the head to try to see an object.

    :returns: True is the object is seen, False if it failed.
    """

    robot.switch_active_stereo_pair(mode)

    robot.look_at([1.0,0.0,0.5,"base_link"])

    robot.wait(2)

    attempts = 1
    #while not (["myself sees " + obj] in self._robot.knowledge) \
    while not robot.state.isseen(obj) \
            and attempts <= max_attempts:

        # Looks once left, once right
        robot.look_at([1.0, 0.3 * ((attempts % 2) * 2 - 1) , 0.5,"base_link"])
        robot.wait(2)
        if robot.state.isseen(obj):
            break

        robot.look_at([1.0, 0.6 * ((attempts % 2) * 2 - 1) , 0.5,"base_link"])
        robot.wait(2)
        if robot.state.isseen(obj):
            break

        robot.look_at([1.0, 0 , 0.5,"base_link"])
        robot.wait(1)
        attempts += 1


    if (attempts <= max_attempts): #ok, found
        robot.look_at(obj)
        return True
    elif mode == "wide_stereo":
        # try by looking more closely
        return lookforobject(robot, obj, max_attempts, mode = "narrow_stereo")
    else: # not found
        #
        robot.look_at([1.0,0.0,1.0,"base_link"])
        robot.switch_active_stereo_pair("wide_stereo")
        return False

class DesiresPerformer():

    def __init__(self, robot):
        self.robot = robot
        self.isperforming = False
        self.done = threading.Condition()

    def perform(self, desire):
        if self.isperforming:
            raise RobotError("Trying to execute 2 desires at the same time! Threading issue!")
        self.isperforming = True
        self.currentpriority = desire._priority
        desire.perform()

        self.done.acquire()
        self.isperforming = False
        self.robot.invalid_context = False # reset the context
        self.done.notifyAll()
        self.done.release()

    def trysuperseed(self, desire):
        """ Check if the priority of the given desire
        is higher than the desire currently performing.

        If yes, cancel the execution of the current
        desire.

        The new desire is not started. DesiresPerformer.perform() must
        be call to this end.
        """
        if not self.isperforming:
            return

        if desire._priority < self.currentpriority:
            self.robot.cancel_all_background_actions()
            self.robot.cancel_all_ros_actions()
            self.robot.invalid_context = True
            self.done.acquire()
            self.done.wait()
            self.done.release()

    def waitfortermination(self):
        if self.isperforming:
            self.done.acquire()
            self.done.wait()
            self.done.release()


class Desire(object):
    """ This class encapsulates a desire (or goal) as formalized in the
    OpenRobot ontology.
    
    An instance of Desire is constructed by passing an existing desire ID.
    """
    def __init__(self, situation, robot, owners = [], performer = None):
        self._sit = situation
        self._robot = robot
        self._priority = 10 # default priority. 0 is the highest priority.
        
        if owners:
            self.owners = owners
        else:
            self.owners = robot.knowledge["* desires " + self._sit]
            if not self.owners:
                raise NotExistingDesireError("I couldn't find anyone actually desiring " + self._sit)

        #If only one owner, create an alias
        if len(self.owners) == 1:
            [self.owner] = self.owners
        else:
            self.owner = None
        
        if performer:
            self.performer = performer
        else:
            #TODO: for now, we only keep the first performer (useful when several equivalent instances are returned)
            self.performer = robot.knowledge[self._sit + " performedBy *"][0]
            if not self.performer:
                raise NoPerformerDesireError("I couldn't find anyone to perform " + self._sit)
        
        self.name = self.__class__.__name__
    
    def perform(self):
        if "myself" in self.performer:
            logger.info("Now performing " + self.name)
            self._robot.knowledge.add(["myself currentlyPerforms " + self._sit], "EPISODIC")

        self._process()

    def _process(self):
        """ subclass responsability!
        """
        pass

class Move(Desire):
    def __init__(self, situation, robot, goal = None, track = True, distance = 1):
        super(Move, self).__init__(situation, robot)
        
        if goal:
            self.to = goal
        else:
            self.to = robot.knowledge[self._sit + " hasGoal *"][0]
            if not self.to:
                self.to = robot.knowledge[self._sit + " actsOnObject *"][0]

        # Do we have a special 'hard-coded' destination for manipulation?
        if "%s_MANIPULATION" % str(self.to) in places.places():
            self.target_is_object = False
            logger.warning("Found a special manipulation place for %s. Overriding the destination with it." % self.to)
            self.to = "%s_MANIPULATION" % self.to
        else:

            # Check if our target is an object or not. If it's an object, we
            # move close to it, but not *on* it.
            if self.to + " rdf:type cyc:PartiallyTangible" in robot.knowledge:
                self.target_is_object = True
            else:
                self.target_is_object = False # We assume then it's a general location. We move *on* it.

        self.track = track
        self.distance = distance

    def _process(self):
        logger.info("Moving to: " + self.to)

        robot = self._robot
        
        already_at_destination = False
        
        target = []

        if self.target_is_object:
            logger.info("Moving towards an object/human: I'll try not to hurt it.")
        else:
            logger.info("Moving towards a location: I'll try to go as near as possible.")

        try:
            target = robot.poses[self.to]
        except UnknownFrameError:
            robot.say("I don't know such a place...")
            return

        target["z"] = 0
        
        if self.target_is_object:

            target_distance = self.distance # we want to stop at 1m of the target.
            if robot.poses.distance(robot.poses.myself(), target) < target_distance:
                already_at_destination = True

        if not already_at_destination:
            robot.extractpose(nop)
            if self.track:
                robot.track(self.to)
            robot.manipose(nop)
            logger.info("Destination coordinates: " + str(target))
            if self.target_is_object:
                robot.moveclose(target)
            else:
                robot.goto(target)

            if self.track:
                robot.cancel_track() # TODO: cancel track a bit before arriving

        if self.target_is_object and robot.poses.human(self.to):
            robot.look_at(self.to)
        else:
            robot.look_at([1.0,0,1.0,"base_link"])

class Get(Desire):
    def __init__(self, situation, robot):
        super(Get, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        try:
            self.to = robot.knowledge[self._sit + " receivedBy *"][0]
        except IndexError:
            self.to = robot.knowledge["* desires " + self._sit][0] # if no destinary, use the desire issuer.
    
    def _process(self):

        robot = self._robot

        logger.info("Wanna get smthg: " + str(self.objects))
        #self._robot.basictake()

        picked, res = haspickedsmthg(robot) # Already smthg in hand?
        if picked:
            robot.say("Soon Mamoun will have implemented the object transfer. Cool!")
            return


        if not robot.poses.human(self.to):
            robot.say("Where are you?")
            robot.wait(5)
            
            if not robot.poses.human(self.to):
                robot.say("When you are ready, ask me again.")
                robot.manipose()
                robot.goto("BASE")
                return

        robot.take(self.to, self.objects[0])

        # look in the hand where the object is
        #self._robot.switch_active_stereo_pair("narrow_stereo")
        robot.extractpose()
        robot.look_at([0.2,0,0.1,"r_gripper_r_finger_link"])
        robot.wait(2)
        if not robot.state.isseen(self.objects[0]):
            robot.wait(2)

        #self._robot.switch_active_stereo_pair("wide_stereo")

        if not robot.state.isseen(self.objects[0]):
            robot.say("I do not see your object...")
        else:
            robot.attachobject(self.objects[0])

        if robot.poses.human(self.to):
            robot.look_at(self.to)

        robot.manipose()

class Show(Desire):
    def __init__(self, situation, robot, owners = [], performer = None, objects = [], receivers = []):

        super(Show, self).__init__(situation, robot, owners, performer)
        
        if objects:
            self.objects= objects
        else:
            self.objects = robot.knowledge[self._sit + " actsOnObject *"]

        if receivers:
            self.to = receivers
        else:
            self.to = robot.knowledge[self._sit + " receivedBy *"]
    
    def _process(self):
        robot = self._robot

        logger.info(str(self.performer) + " wants to show " + str(self.objects) + " to " + str(self.to))

        if "%s hasInHand %s" % (self.performer, self.object) in robot.knowledge:
            planid = robot.planning.manipulation(robot.planning.SHOW, self.objects[0], self.to[0], self.performer)
        else:
            planid = robot.planning.manipulation(robot.planning.SHOW, self.objects[0], self.to[0], self.performer)

        if planid:
            robot.show(planid)
            planid2 = robot.planning.actionplanning(robot.planning.PUT_ACCESSIBLE, self.objects[0], self.to[0], self.performer)
            robot.say("Here your object")
            robot.wait(2)

            if planid2:
                robot.put_accessible(planid)

            robot.extractpose()

        else:
            robot.say("The object is next to me")

        robot.manipose()

 
class Give(Desire):
    def __init__(self, situation, robot):
        super(Give, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        try:
            self.to = robot.knowledge[self._sit + " receivedBy *"][0]
        except IndexError:
            self.to = robot.knowledge["* desires " + self._sit][0] # if no destinary, use the desire issuer.
 
    def _process(self):
        logger.info(str(self.doer) + " wants to give " + str(self.objects) + " to " + str(self.to))
        self._robot.say("Let's give " + self.objects[0] + " to " + self.to)
        
        #self._robot.give(self.objects[0], self.to[0])
        self._robot.basicgive()
        self._robot.attachobject(self.objects[0], attach = False)

class Pick(Desire):
    def __init__(self, situation, robot):
        super(Pick, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]



    def giveup(self):
        robot = self._robot
        robot.look_at([1.0,0.0,0.5,"base_link"])

        robot.translate(-0.3) # undock
        robot.manipose()
        try:
            robot.look_at(self.to)
        except Exception: # Human not here?
            pass

    def pick(self):

        robot = self._robot
        obj = self.objects[0]

        logger.info(str(self.doer) + " wants to pick " + str(self.objects))

        objectinhand = False
        ok, res = haspickedsmthg(robot)
        if ok:
            logger.info("I already have something in my hand...")
            robot.say("My hands are full, I first bring that to you.")
            objectinhand = True
            
# Do not check the object type for now because of an inconsistency in ORO
#            try:
#                currentobj = robot.knowledge["myself hasInRightHand *"][0]
#            except IndexError: #We have smthg in hand, but it's not in the ontology...
#                currentobj = "UNKNOWN"
#
#            if currentobj == obj:
#                robot.say("I have it already. Good.")
#                objectinhand = True
#            else:
#                robot.say("My hands are full!")
#                return

        if not objectinhand:
            robot.setpose("TUCK_LARM")
            robot.manipose(nop)
            if len(self.objects) > 1:
                logger.info("Let take care of " + obj + " for now.")

            robot.say("Let's take it")

            # Object not visible? try to find it
            if not robot.state.isseen(obj):

                loc = robot.knowledge[obj + " isAt *"]
                if not loc:
                    logger.warning("No place found for " + obj + "! I can not bring it")
                    robot.say("Humm. I do not know where is the object...")
                    return False
                #robot.say(obj + " is on " + loc[0] + ". Let go there.")

                track_target = robot.poses[loc[0]]
                track_target["z"] += 1.0
                robot.track(track_target)

                move = Move(self._sit, robot, loc[0], track = False)
                move.perform()

                robot.extractpose(nop)
                robot.cancel_track()
                robot.look_at([1.0,0.0,0.5,"base_link"])

                #ok, bb = robot.execute([genom_request("spark", "GetBBPoints", [loc[0]])])
                #ok, bb = robot.poco_modules["spark"].GetBBPoints(loc[0])

                #if ok=="OK":
                #    support_height = float(bb[2])
                #    logger.info("The object is placed on a support that is at " + str(support_height) + "m.")

                #    if support_height < 0.6:
                #        robot.settorso(0.0)

                #    if support_height > 0.9:
                #        robot.settorso(0.3)

                robot.settorso(0.1, nop)
                hasdocked, res = robot.dock() # docking fails if no obstacle is seen within 1m
                if not hasdocked:
                    robot.translate(0.3)

                robot.say("Ok. Now, where is my object?")
                
                ok = lookforobject(robot, obj, max_attempts = 1)
                
                if not ok:
                    logger.warning("I can not see the object " + obj + "! Giving up.")
                    robot.say("I did not see your object... Let's try again.")
                    ok = lookforobject(robot, obj, max_attempts = 2)
                    if not ok:
                        logger.warning("Second Findobject also failed!")
                        try:
                            robot.look_at(self.to)
                        except Exception: # Human not here?
                            pass

                        robot.say("I give up!")
                        self.giveup()
                        return False

            robot.say("Ok, I see the object. Let's try to pick it.")
            ok, res = robot.pick(obj)

            if not ok:
                logger.warning("Pick failed! Msg:" + str(res) )
                robot.say("I think I missed the object... Let's try one more time.")
                robot.extractpose()
                lookforobject(robot, obj, max_attempts = 3)
                ok, res = robot.pick(obj)
                if not ok:
                    logger.warning("Second Pick also failed! Msg:" + str(res) )
                    try:
                        robot.look_at(self.to)
                    except Exception: # Human not here?
                        pass

                    robot.say("I give up!")
                    self.giveup()
                    return False

            robot.attachobject(obj)

            robot.extractpose()

            robot.translate(-0.2) # undock

        return True

    def _process(self):
        self.pick()

class Bring(Desire):
    def __init__(self, situation, robot):
        super(Bring, self).__init__(situation, robot)

        self.pickDesire = Pick(situation, robot)

        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        try:
            self.to = robot.knowledge[self._sit + " receivedBy *"][0]
        except IndexError:
            self.to = robot.knowledge["* desires " + self._sit][0] # if no destinary, use the desire issuer.


        self.in_safe_nav_pose = True

    def navprogress(self, progress):
        logger.debug("Waypoints node feedback: " + str(progress.percentage_covered) + "% of traj covered (" +  str(progress.distance_to_go) + "m to go).")
        if progress.distance_covered > 0.5: # 50cm off the initial position
            self._robot.manipose()
            self.in_safe_nav_pose = True

    def _process(self):

        robot = self._robot
        obj = self.objects[0]
        robot.say("Bring bring bring") #this first sound is always discarded...

        logger.info(str(self.doer) + " wants to bring " + str(self.objects) + " to " + str(self.to))

        ####################################
        #### First, pick the object

        if not self.pickDesire.pick():
            return


        ####################################
        #### Bring the object to the human

        robot.manipose(nop)

        self.in_safe_nav_pose = False

        #if [self.to + " experiences " + robot.knowledge["* hasFeature lazy"][0]] in robot.knowledge:
        #    logger.info("Human feels lazy! Let's move to him.")
        #    mobility = 0.0
        #else:
        #    logger.info("Human looks fine. Let's both of us move.")
        mobility = 0.2

        if not robot.poses.human(self.to):
            robot.say("Where are you?")
            robot.wait(5)
            if not robot.poses.human(self.to):
                robot.say("When you are ready, ask me again.")
                robot.manipose()
                robot.goto("BASE")
                return

        robot.handover(self.to, mobility = mobility, feedback = self.navprogress)
        robot.wait(2)
        ok, res = haspickedsmthg(robot)
        if ok:
            robot.say("You do not want your object? Fine.")
        else:
            robot.attachobject(obj, attach = False)

        robot.manipose()

class Put(Desire):
    def __init__(self, situation, robot):
        super(Put, self).__init__(situation, robot)
        
        self.doer = robot.knowledge[self._sit + " performedBy *"]

        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"][0]
        self.to = robot.knowledge[self._sit + " receivedBy *"][0]


    def _process(self):

        robot = self._robot

        logger.info(str(self.doer) + " wants to put " + str(self.objects) + " on " + str(self.to))

        obj = self.objects[0]
        if len(self.objects) > 1:
            logger.info("Currently, I'm only able to process one object (" + obj + ") for 'put' action.")

        objectinhand = False
        ok, res = haspickedsmthg(robot)
        if not ok:
            logger.info("No object in hand!")
            robot.manipose(nop)
            robot.say("My hands are empty!")
            return

# Do not check the object type for now because of an inconsistency in ORO
#            try:
#                currentobj = robot.knowledge["myself hasInRightHand *"][0]
#            except IndexError: #We have smthg in hand, but it's not in the ontology...
#                currentobj = "UNKNOWN"
#
#            if currentobj == obj:
#                robot.say("I have it already. Good.")
#                objectinhand = True
#            else:
#                robot.say("My hands are full!")
#                return

        track_target = robot.poses[self.to]
        track_target["z"] += 1.0
        robot.track(track_target)

        move = Move(self._sit, robot, self.to, track = False)
        move.perform()

        robot.cancel_track()
        robot.look_at([1.0,0.0,0.5,"base_link"])

        robot.extractpose(nop)
        hasdocked, res = robot.dock() # docking fails if no obstacle is seen within 1m
        if not hasdocked:
            robot.translate(0.2)

        robot.say("Ok. Now, let's put it.")

        robot.attachobject(obj, False) # detach the object

        robot.put(obj, self.to)

        robot.extractpose()

        robot.translate(-0.2) # undock


class Test(Desire):
    def __init__(self, situation, robot):
        super(Test, self).__init__(situation, robot)
        
        self.doer = robot.knowledge[self._sit + " performedBy *"]
    def _process(self):
        logger.info("hello robot 22222")
        obj = "LOW_TABLE_LARGE"
        poseDist = 0.1
        self._robot.put("GREY_TAPE",obj)

class Stop(Desire):
    def __init__(self, situation, robot):
        super(Stop, self).__init__(situation, robot)
        
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        self._priority = 1
    
    def _process(self):
        self._robot.say("Alright, I stop")
        # stop has a high priority: it will cancel all currently running actions
        self._robot.manipose()
        self._robot.look_at([1,0,0.7,"base_link"])



class Hide(Desire):
    def __init__(self, situation, robot):
        super(Hide, self).__init__(situation, robot)
        
        self.objects = robot.knowledge[self._sit + " actsOnObject *"]
        self.doer = robot.knowledge[self._sit + " performedBy *"]
        self.to = robot.knowledge[self._sit + " receivedBy *"]
    
    def _process(self):
        logger.info(str(self.doer) + " wants to hide " + str(self.objects) + " to " + str(self.to))
        
        planid = robot.planning.actionplanning(robot.planning.HIDE, self.objects[0], self.to[0], self.doer[0])

        if planid:
            robot.hide(planid)
            robot.extractpose()
            robot.manipose()
            robot.say("He he, I've hidden it!")
            


class Look(Desire):

    UP = "UpZone"
    LEFT = "LeftZone"
    RIGHT = "RightZone"
    DOWN = "DownZone"
    FORWARD = "ForwardZone"
    BEHIND = "BehindZone"
    ON = "OnZone"
    UNDER = "UnderZone"
    ABOVE = "AboveZone"
    IN = "InZone"

    def __init__(self, situation, robot):
        super(Look, self).__init__(situation, robot)
        
        self.robot = robot

        self.to = robot.knowledge["* desires " + self._sit][0]

        self.doer = robot.knowledge[self._sit + " performedBy *"]

        self.object = None

        self.zone = None # type ('above', 'under'...), relative to object

        self.objects = robot.knowledge[self._sit + " hasGoal *"]
        if self.objects:
            if len(self.objects) > 1:
                logger.warning("Looking for more than one object: %s. Taking only the first one into account." % self.objects)
            self.object = self.objects[0]

            try:
                robot.poses[self.object]
            except UnknownFrameError:
                logger.info("Unknown frame %s. Checking if it is a zone..." % self.object)
                self.zone, self.object = self.findzone(self.objects)
        else:
            candidate_zones = robot.knowledge[self._sit + " involves *"]
            self.zone, self.object = self.findzone(candidate_zones)

        self.qualification = robot.knowledge[self._sit + " actionQualification *"]

    def findzone(self, zones):
        for z in zones:
            types = self.robot.knowledge.getDirectClassesOf(z)

            # Special zone? ( up, down, behind...)
            for t in types:
                if t in [self.UP, self.DOWN, self.LEFT, self.RIGHT, self.FORWARD, self.BEHIND]:
                    return t, 'myself'

            # Not a special zone.
            if 'Location' in types:
                place = self.robot.knowledge["%s isAt *" % z]
                if not place:
                    logger.warning("No place found to look at!")
                    return None, None
                place = place[0]

                rel = self.robot.knowledge["%s * %s" % (z, place)]

                if 'isOn' in rel:
                    rel = self.ON
                elif 'isIn' in rel:
                    rel = self.IN
                elif 'isUnder' in rel:
                    rel = self.UNDER
                elif 'isAbove' in rel:
                    rel = self.ABOVE
                else:
                    rel = None # in the general case, we simply look at the place

                return rel, place

        return None, None

    def _process(self):

        robot = self._robot

        if "CLOSE" in self.qualification:
            robot.switch_active_stereo_pair("narrow_stereo")
        elif "NORMAL" in self.qualification or "WIDE" in self.qualification:
            robot.switch_active_stereo_pair("wide_stereo")


        if self.zone:
            logger.info(str(self.doer) + " wants to look '%s' relative to %s." % (self.zone, self.object))
            
            # First, special abstract zones
            if self.zone == self.UP:
                robot.look_at([1.0,0,0.5,'head_plate_frame'])
            elif self.zone == self.DOWN:
                robot.look_at([1.0,0,-0.5,'head_plate_frame'])
            elif self.zone == self.LEFT:
                robot.look_at([0.5,1.0,1.0,'base_link'])
            elif self.zone == self.RIGHT:
                robot.look_at([0.5,-1.0,1.0,'base_link'])
            elif self.zone == self.FORWARD:
                robot.look_at([1.0,0,1.0,'base_link'])
            elif self.zone == self.BEHIND:
                robot.look_at([-2.0,0,1.0,'base_link'])

            # Then, zone relative to an object
            elif self.zone in [self.ON, self.ABOVE]:
                pose = robot.poses[self.object]

                ok, bb = robot.execute([genom_request("spark", "GetBBPoints", [self.object])])

                if ok == 'OK': # we got the bounding box. Cool.
                    support_height = float(bb[2])
                    pose['z'] = support_height
                else:
                    logger.warning("No bounding-box for object %s. Can not reliably look 'on' it." % self.object)
                    pose['z'] += 0.2 #look 20 cm above the object.

                robot.look_at(pose)
                robot.sweep(45)

            else:
                logger.warning("Zone '%s' for look is not implemented. Looking directly at %s instead." % (self.zone, self.object))
                try:
                    robot.look_at(self.object)
                except UnknownFrameError:
                    robot.say("I do not know this object!")



        elif self.object:
            logger.info("%s wants to look at %s" % (self.doer, self.object))
            try:
                robot.look_at(self.object)
            except UnknownFrameError:
                robot.say("I do not know this object!")
        else:
            if robot.poses.human(self.to):
                robot.look_at(self.to)



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
    act = [x for x in act if x not in ["ActiveConcept"]] # Filter out some internal classes
    try:
        [action] = act
    except ValueError:
        raise NotExistingDesireTypeError(sit + " matches several actions ( " + \
                str(act) + ")! Did you forget to assert that atomic action " + \
                "are subclasses of PurposefulAction?")
    
    try:
        logger.info("Well well well! Someone wants a " + action + "!")
        goal = eval(action)(sit, robot)
    except NameError:
        raise NotExistingDesireTypeError("I don't know the action \"" + action + "\"!")
    
    return goal
