import logging; logger = logging.getLogger("robots.events")
import time
import weakref

import threading # for current_thread()
from robot_actions import SignalingThread, ACTIVE_SLEEP_RESOLUTION

from robots.introspection import introspection

class Events:
    def __init__(self, robot):

        self.robot = robot
        self.eventmonitors = []

    def on(self, var, **kwargs):
        """
        Creates a new EventMonitor to watch a given event model (on shot).
        :returns: a new instance of EventMonitor for this event.
        """
        monitor = EventMonitor(self.robot, var, oneshot=True, **kwargs)
        self.eventmonitors.append(weakref.ref(monitor))
        return monitor


    def every(self, var, max_firing_freq = 10, blocking = True, **kwargs):
        return self.whenever(var, max_firing_freq, blocking, **kwargs)

    def whenever(self, var, max_firing_freq = 10, blocking = True, **kwargs):
        """
        Creates a new EventMonitor to watch continuously a given event model.

        :param max_firing_freq: set how many times pe second this event may be
        triggered (default to 10Hz. 0 means as many as possible)
        :param blocking: if True, the event callback is blocking, preventing
        new event to be triggered until the callback has completed (defaults to
        True).
        :returns: a new instance of EventMonitor for this event.
        """
        monitor = EventMonitor(self.robot, var, oneshot=False, max_firing_freq = max_firing_freq, blocking = blocking, **kwargs)
        self.eventmonitors.append(weakref.ref(monitor))
        return monitor

    def stop_all_monitoring(self):
        """ Stops all event monitoring, but do not interrupt event callbacks,
        if any is running.

        You may want to use Events.stop_all instead of Events.cancel_all when
        you need to prevent new events of being raised *from* an event callback
        (Events.cancel_all would interrupt this callback as well).

        """
        for m in self.eventmonitors:
            monitor = m() # weakref!
            if monitor:
                monitor.stop_monitoring()

    def cancel_all(self):
        # first, we tell *all* monitors not to trigger any events anymore
        # then we actually stop the monitor by interupting the callbacks
        # they may be processing.
        self.stop_all_monitoring()

        for m in self.eventmonitors:
            monitor = m() # weakref!
            if monitor:
                monitor.close()


    def close(self):
        self.cancel_all()

class EventMonitor:

    VALUE = "="
    BECOMES = "becomes"
    ABOVE = ">"
    BELOW = "<"
    INCREASE = "+="
    DECREASE = "-="

    def __init__(self, robot, var, 
                        value = None, 
                        becomes = None,
                        above = None, 
                        below = None,
                        increase = None,
                        decrease = None,
                        oneshot = False,
                        max_firing_freq = 10,
                        blocking = True):
        """

        :param oneshot: if true, the event is fired once and then discarded. 
        Otherwise, the event remains active.

        """

        self.cbs= [] # callbacks

        self.robot = robot

        if var not in robot.state and not callable(var):
            raise Exception("%s is neither a member of the robot's state or a predicate" % var)

        self.var = var

        self.oneshot = oneshot
        self.max_firing_freq= max_firing_freq
        self.blocking = blocking

        self.monitoring = False
        self.thread = None

        # store initial value, used by INCREASE/DECREASE modes
        # and last value, used by BECOMES modes
        if not self.robot.dummy:
            self.start_inc_value = self.robot.state[self.var] 
            self.start_dec_value = self.robot.state[self.var] 
            self.last_value = self.robot.state[self.var] 

        if not callable(self.var):
            if value is not None:
                self.mode = EventMonitor.VALUE
                self.target = value
            elif becomes is not None:
                self.mode = EventMonitor.BECOMES
                self.target = becomes
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

        logger.info("Added new event monitor: %s" % self)

    def do(self, cb):

        if introspection:
            introspection.action_subscribe_event("BROKEN TDB", str(self))

        # first add callback? start a thread to monitor the event!
        if not self.thread:
            self.monitoring = True
            self.thread = SignalingThread(target=self._monitor)
            self.thread.start()

        self.cbs.append(cb)
        return self # to allow for chaining

    def _monitor(self):

        threading.current_thread().name = "Event monitor on %s" % self
        while self.monitoring:
            ok = self._wait_for_condition()
            if not ok: # monitoring has been interrupted!
                return

            if introspection:
                introspection.action_event_fired("BROKEN TDB", str(self))

            for cb in self.cbs:
                if not self.blocking:
                    cb(self.robot)
                else:
                    cb(self.robot).wait()

                    # after a blocking event, reset the reference values for
                    # events INCREASE and DECREASE
                    self.start_inc_value = self.robot.state[self.var] 
                    self.start_dec_value = self.robot.state[self.var] 

            if self.oneshot:
                logger.info("Removing event on %s" % self)
                return
            else:
                if self.max_firing_freq > 0:
                    self.robot.sleep(1./self.max_firing_freq)

    def stop_monitoring(self):
        self.monitoring = False

    def close(self):
         if self.thread and self.thread.is_alive:
            self.thread.cancel()
            self.thread.join()

    def _check_condition(self, val):

        ok = False

        if self.mode == EventMonitor.VALUE and val == self.target:
            ok = True
        elif self.mode == EventMonitor.BECOMES and self.last_value != val and val == self.target:
            ok = True
        elif self.mode == EventMonitor.ABOVE and val > self.target:
            ok = True
        elif self.mode == EventMonitor.BELOW and val < self.target:
            ok = True
        elif self.mode == EventMonitor.INCREASE and val > (self.start_inc_value + self.target):
            self.start_inc_value = val
            ok = True
        elif self.mode == EventMonitor.DECREASE and val < (self.start_dec_value - self.target):
            self.start_dec_value = val
            ok = True

        # TODO: could be improved with a bit of hysteris filtering
        if val > self.start_dec_value:
            self.start_dec_value = val
        if val < self.start_inc_value:
            self.start_inc_value = val

        self.last_value = val
        return ok


    def _wait_for_condition(self):

        if not self.robot.dummy:

            # predicate-based event
            if callable(self.var):
                while not self.var(self.robot):
                    time.sleep(ACTIVE_SLEEP_RESOLUTION)

            # state-based event
            else:
                if self.var not in self.robot.state:
                    # value not yet read from the robot.
                    logger.warning("Waiting for %s to be published by the robot..." % self.var)
                    while not self.var in self.robot.state:
                        self.robot.wait_for_state_update(2)

                while not self._check_condition(self.robot.state[self.var]):
                    if not self.monitoring:
                        logger.info("<%s> not monitored anymore" % str(self))
                        return False
                    self.robot.wait_for_state_update(ACTIVE_SLEEP_RESOLUTION)

        else:
            #dummy mode. Wait a little bit, and assume the condition is true
            import time
            time.sleep(0.2)
        logger.info("%s is true" % str(self) + (" (dummy mode)" if self.robot.dummy else ""))
        return True


    def wait(self):
        """ Blocks until an event occurs.
        """

        if introspection:
            introspection.action_waiting("BROKEN TDB", str(self))


        self._wait_for_condition()


        if introspection:
            introspection.action_waiting_over("BROKEN TDB")

    def __str__(self):
        return "condition <%s %s %s>"% (self.var, self.mode, self.target)
