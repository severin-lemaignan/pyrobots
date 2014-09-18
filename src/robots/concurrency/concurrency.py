# coding=utf-8
"""
Concurrency support for pyRobot.

This module provides:

- an implementation of :class:`SignalingThread` (threads that explicitely
  handle signals like cancelation)
- heavily modified Python futures to support robot action management.
- A future executor that simply spawn one thread per future (action) instead of
  a thread pool.

These objects should not be directly used. Users should instead rely on the
:meth:`~robots.concurrency.action.action` decorator.

Helpful debugging commands::

    >>> sys._current_frames()
    >>> inspect.getouterframes(sys._current_frames()[<id>])[0][0].f_locals

"""
import logging; logger = logging.getLogger("robots.actions")

import sys

import uuid

MAX_FUTURES = 20
MAX_TIME_TO_COMPLETE = 1 # sec: time allowed to tasks to complete when cancelled. If they take more than that, force termination.
ACTIVE_SLEEP_RESOLUTION = 0.1 # sec

try:
    from concurrent.futures import Future, TimeoutError
except ImportError:
    import sys
    sys.stderr.write("[error] install python-concurrent.futures\n")
    sys.exit(1)

import os.path #for basename
import weakref
import threading 
import thread # for get_ident
from collections import deque

import traceback

from .signals import ActionCancelled, ActionPaused


class SignalingThread(threading.Thread):
    def __init__(self, *args, **kwargs):
        threading.Thread.__init__(self, *args, **kwargs)
        self.debugger_trace = None

    def cancel(self):
        self.__cancel = True
    def pause(self):
        self.__pause = True

    def _Thread__bootstrap(self):
        """ The name come from Python name mangling for 
        __double_leading_underscore_names

        Note that in Python3, __bootstrap becomes _bootstrap, thus
        making it easier to override.
        """
        if threading._trace_hook is not None:
            self.debugger_trace = threading._trace_hook
        else:
            self.debugger_trace = None

        self.__cancel = False
        self.__pause = False
        sys.settrace(self.__signal_emitter)

        self.name = "Ranger action thread (initialization)"
        super(SignalingThread, self)._Thread__bootstrap()

    def __signal_emitter(self, frame, event, arg):
        if self.__cancel:
            if frame.f_globals["__name__"] == "threading":
                # Raising exception at uncontrolled time is a dangerous sport,
                # especially if the thread is in the middle of locking/unlocking shared resources
                # like (in our case) setting result in futures and reading them.
                # After thinking about it for a day, I could not find any good solution except for
                # postponing raising the signals until out of the threading module. This is not
                # very nice, but seem to work out well.

                #logger.debug("Thread <%s> in threading module. Postponing cancelation" % self.name)
                pass
            else:
                self.__cancel = False
                desc = "Cancelling thread <%s>:\n" % self.name
                tb = traceback.extract_stack(frame, limit = 6)
                for f in tb:
                    file, line, fn, instruction = f
                    desc += " - in <%s> (l.%s of %s): %s\n" % (fn, line, os.path.basename(file), instruction)

                logger.debug(desc)
                raise ActionCancelled()
        if self.__pause:
            self.__pause = False
            logger.debug("Pausing thread <%s>" % self.name)
            raise ActionPaused()

        if self.debugger_trace:
            return self.debugger_trace
        else:
            return self.__signal_emitter

class RobotActionThread(SignalingThread):
    def __init__(self, future, initialized, fn, args, kwargs):
        SignalingThread.__init__(self)

        initialized.set()

        self.future = future
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    def run(self):

        if not self.future.set_running_or_notify_cancel():
            return

        try:
            result = self.fn(self.future, str(self.future),*self.args, **self.kwargs)
            self.future.set_result(result)
            logger.debug("Action <%s>: completed." % str(self.future))
        except BaseException:
            e = sys.exc_info()[1]
            logger.error("Exception in action <%s>: %s"%(str(self.future), e)) #self.fn.__name__
            logger.error(traceback.format_exc())
            self.future.set_exception(e)


class RobotAction(Future):
    def __init__(self, actionname):
        Future.__init__(self)

        self.actionname = actionname

        self.thread = None
        self.id = uuid.uuid4()

        self.subactions = []
        self.parent_action = None

        self.has_acquired_resource = False

    def add_subaction(self, action):
        self.subactions = [a for a in self.subactions if a() is not None and a().thread() is not None]
        self.subactions.append(action)
        logger.debug("Added sub-action %s to action %s" % (str(action()), str(self)))#.actionname))  1: action().actionname

    def set_parent(self, action):
        self.parent_action = action

    def childof(self, action):
        """ Returns true if this action is a child of the given action, ie, has
        been spawned from the given action or any of its descendants.
        """
        parent = self.parent_action() # weakref!
        if parent is None:
            return False
        if parent is action:
            return True
        return parent.childof(action)

    def set_thread(self, thread):
        self.thread = thread

    def cancel(self):
        # we do not call the 'standard' Future.cancel method since we do not have a thread pool (ie,
        # the future can not be 'pending for execution'), which is the only useful usecase (well, with callback on
        # cancellation, I imagine... so those are not supported for now)

        thread = self.thread() # weakref!
        if thread is None:
            logger.debug("Action <%s>: already done" % self)
            return

        # first, cancel myself (to make sure I won't restart subactions)
        logger.debug("Action <%s>: signaling cancelation to action's thread" % self)
        thread.cancel()

        # then, tell all the subactions that they should stop
        # (can not do that in the thread's cancel (_signal_emitter), because the
        # thread may hold locks that are not released until the exception is raised and
        # the context manager are left)
        logger.debug("Action <%s>: %s subactions to cancel" % (self, len(self.subactions))) #self.actionname

        for weak_subaction in self.subactions:
            subaction = weak_subaction()
            if subaction:
                logger.debug("Action <%s>: Cancelling subaction %s..." % (self, subaction))
                subaction.cancel()


        # then, make sure everybody actually terminates
        logger.debug("Action <%s>: now waiting for completion" % self)
        try:
            self.exception(timeout = MAX_TIME_TO_COMPLETE) # waits this amount of time for the task to effectively complete
        except TimeoutError:
            raise RuntimeError("Unable to cancel action %s (still running %s after cancellation)!" % (self, MAX_TIME_TO_COMPLETE))
        logger.debug("Action <%s>: successfully cancelled" % self)
        #t = 0
        #while t < MAX_TIME_TO_COMPLETE:
        #    time.sleep(ACTIVE_SLEEP_RESOLUTION)
        #    t+=ACTIVE_SLEEP_RESOLUTION
        #    if self.thread() is None:
        #        logger.debug("Action <%s>: successfully cancelled" % self.actionname)
        #        return
        #raise RuntimeError("Unable to cancel action %s (still running %s after cancellation)!" % (self.actionname, MAX_TIME_TO_COMPLETE))

    def result(self):
        if self.parent_action and self.parent_action():
            threading.current_thread().name = "Action %s (waiting for sub-action %s)" % (self.parent_action(), self)
        else:
            threading.current_thread().name = "Main thread (waiting for sub-action %s)" % self


        # active wait! Instead of blocking on the condition variable in super.result()
        # we do an active wait to make sure we can cancel/suspend the action via our
        # __signal_emitter trace function
        while True:
            try:
                return super(RobotAction, self).result(ACTIVE_SLEEP_RESOLUTION)
            except TimeoutError:
                pass

    def wait(self):
        """ alias for result()
        """
        return self.result()

    def __lt__(self, other):
        """ Overrides the comparision operator (used by ==, !=, <, >) to
        first wait for the result of the future.
        """
        return self.result().__lt__(other)

    def __le__(self, other):
        return self.result().__le__(other)

    def __eq__(self, other):
        return self.result().__eq__(other)

    def __ne__(self, other):
        return self.result().__ne__(other)

    def __gt__(self, other):
        return self.result().__gt__(other)

    def __ge__(self, other):
        return self.result().__ge__(other)

    def __repr__(self):
        return str(self.id)

    def __str__(self):
        return self.actionname  + "[" + self.__repr__() + "]"

class FakeFuture:
    """ Used in the 'immediate' mode.
    """

    def __init__(self, result):
        self._result = result
    def result(self):
        return self._result
    def wait(self):
        return self._result

class RobotActionExecutor():

    def __init__(self):

        # Attention, RobotActionExecutor must be thread-safe
        self.futures = []

        self.futures_lock = threading.Lock()

    def submit(self, fn, *args, **kwargs):

        with self.futures_lock:
            self.futures = [f for f in self.futures if not f.done()]


        name = fn.__name__
        if args and not kwargs:
            name += "(%s)" % ", ".join([str(a) for a in args[1:]]) # start at 1 because 0 is the robot instance
        elif kwargs and not args:
            name += "(%s)" % ", ".join(["%s=%s" % (str(k), str(v)) for k, v in kwargs.items()])
        elif args and kwargs:
            name += "(%s, " % ", ".join([str(a) for a in args[1:]])
            name += "%s)" % ", ".join(["%s=%s" % (str(k), str(v)) for k, v in kwargs.items()])

        if len([f for f in self.futures if f.has_acquired_resource]) > MAX_FUTURES:
            raise RuntimeError("You have more than %s actions running in parallel! Likely a bug in your application logic!" % MAX_FUTURES)

        f = RobotAction(name)

        initialized = threading.Event()


        t = RobotActionThread(f, initialized, fn, args, kwargs)
        f.set_thread(weakref.ref(t))


        current_action = self.get_current_action()
        if current_action:
            f.set_parent(weakref.ref(current_action))
            current_action.add_subaction(weakref.ref(f))


        t.start()

        while not initialized.is_set():
            # waits for the thread to actually start
            pass

        with self.futures_lock:
            self.futures.append(f)

        return f

    def get_current_action(self):
        """Returns the RobotAction linked to the current thread.
        """
        thread_id = threading.current_thread().ident

        with self.futures_lock:

            for f in self.futures:
                if not f.done():

                    thread = f.thread() # weak ref
                    if thread is not None and thread.ident == thread_id:
                        return f

        logger.debug("The current thread (<%s>) is not a robot action (main thread?)" % threading.current_thread().name)
        return None

    def cancel_all(self):
        """ Blocks until all the currently running actions are actually stopped.
        """

        with self.futures_lock:
            for f in self.futures:
                if not f.done():
                    f.cancel()

            self.futures = []

    def cancel_all_others(self):
        """ Blocks until all the currently running actions *except the calling
        one* are actually stopped.

        """

        thread_id = threading.current_thread().ident

        with self.futures_lock:
            for f in self.futures:
                if not f.done():
                    thread = f.thread() # weak ref
                    if thread is not None and thread.ident == thread_id:
                        myself = f
                        continue

                    f.cancel()

            self.futures = [myself]



    def actioninfo(self, future_id):

        with self.futures_lock:

            future = [f for f in self.futures if id(f) == future_id]

            if not future:
                return "No task with ID %s. Maybe the task is already done?" % future_id

            future = future[0]


            desc = "Task <%s>\n" % future

            thread = future.thread() # weak ref
            if thread:
                frame = sys._current_frames()[thread.ident]
                tb = traceback.extract_stack(frame, limit = 6)
                for f in tb:
                    file, line, fn, instruction = f
                    desc += " - in <%s> (l.%s of %s): %s\n" % (fn, line, os.path.basename(file), instruction)

                return desc
            else:
                return "Task ID %s is already done." % future_id

    def __str__(self):
        with self.futures_lock:
            return "Running tasks:\n" + \
                    "\n".join(["Task %s (id: %s, thread: <%s>)" % (f, id(f), str(f.thread())) for f in self.futures if not f.done()])

