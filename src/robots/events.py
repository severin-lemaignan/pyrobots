import logging; logger = logging.getLogger("ranger.events")
import weakref

from robot_actions import PausableThread

from ranger.introspection import introspection

class Events:
    def __init__(self, robot):

        self.robot = robot
        self.eventmonitors = []

    def on(self, var, **kwargs):
        """
        Creates a new EventMonitor to watch a given event model.
        :returns: a new instance of EventMonitor for this event.
        """
        monitor = EventMonitor(self.robot, var, **kwargs)
        self.eventmonitors.append(weakref.ref(monitor))
        return monitor

    def close(self):

        # first, we tell *all* monitors not to trigger any events anymore
        # then we actually stop the monitor by interupting the callbacks
        # they may be processing.

        for m in self.eventmonitors:
            monitor = m() # weakref!
            if monitor:
                monitor.stop_monitoring()

        for m in self.eventmonitors:
            monitor = m() # weakref!
            if monitor:
                monitor.close()

class EventMonitor:

    VALUE = "="
    ABOVE = ">"
    BELOW = "<"
    INCREASE = "+="
    DECREASE = "-="

    def __init__(self, robot, var, 
                        value = None, 
                        above = None, 
                        below = None,
                        increase = None,
                        decrease = None,
                        oneshot = False):
        """

        :param oneshot: if true, the event is fired once and then discarded. 
        Otherwise, the event remains active.

        """

        self.cbs= [] # callbacks

        self.robot = robot

        if var not in robot.STATE:
            raise Exception("%s is not part of the robot state" % var)

        self.var = var

        self.oneshot = oneshot

        self.monitoring = False
        self.thread = None

        # store initial value, used by INCREASE/DECREASE modes
        if not self.robot.dummy:
            self.last_value = self.robot.state[self.var] 

        if value is not None:
            self.mode = EventMonitor.VALUE
            self.target = value
        elif above is not None:
            self.mode = EventMonitor.ABOVE
            self.target = above
        elif below is not None:
            self.mode = EventMonitor.BELOW
            self.target = below
        elif increase is not None:
            self.mode = EventMonitor.INCREASE
            self.target = increase
        elif decrease is not None:
            self.mode = EventMonitor.DECREASE
            self.target = decrease
        else:
            raise Exception("Event created without condition!")


    def do(self, cb):

        if introspection:
            introspection.action_subscribe_event("BROKEN TDB", str(self))

        # first add callback? start a thread to monitor the event!
        if not self.thread:
            self.monitoring = True
            self.thread = PausableThread(target=self._monitor)
            self.thread.start()
            self.thread.name = "Event monitor on %s" % self

        self.cbs.append(cb)
        return self # to allow for chaining

    def _monitor(self):

        while self.monitoring:
            self._wait_for_condition()

            if introspection:
                introspection.action_event_fired("BROKEN TDB", str(self))

            for cb in self.cbs:
                cb()
            if self.oneshot:
                return

    def stop_monitoring(self):
        self.monitoring = False

    def close(self):
         if self.thread and self.thread.is_alive:
            self.thread.cancel()
            self.thread.join()

    def _check_condition(self, val):

        if self.mode == EventMonitor.VALUE and val == self.target:
            return True
        elif self.mode == EventMonitor.ABOVE and val > self.target:
            return True
        elif self.mode == EventMonitor.BELOW and val < self.target:
            return True
        elif self.mode == EventMonitor.INCREASE and val > (self.last_value + self.target):
            self.last_value = val
            return True
        elif self.mode == EventMonitor.DECREASE and val < (self.last_value - self.target):
            self.last_value = val
            return True


    def _wait_for_condition(self, timeout = None):

        if not self.robot.dummy:
            if self.var not in self.robot.state:
                # value not yet read from the robot.
                logger.warning("Waiting for %s to be published by the robot..." % self.var)
                self.robot.update.acquire()
                while not self.var in self.robot.state:
                    self.robot.update.wait(1)
                self.robot.update.release()


            self.robot.update.acquire()
            while not self._check_condition(self.robot.state[self.var]):
                self.robot.update.wait(timeout)
            self.robot.update.release()

        logger.info("%s is true" % str(self) + " (dummy mode)" if self.robot.dummy else "")


    def wait(self, timeout = None):
        """ Blocks until an event occurs, or the timeout expires.
        """

        if introspection:
            introspection.action_waiting("BROKEN TDB", str(self))


        self._wait_for_condition(timeout)


        if introspection:
            introspection.action_waiting_over("BROKEN TDB")

    def __str__(self):
        return "condition <%s %s %s>"% (self.var, self.mode, self.target)
