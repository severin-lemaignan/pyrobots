"""
Extension of Python futures to support robot action management.

The main changes are:
 - the use of 'PausableThreads', ie threads in which cancellation and
   pause can be signaled (using custom exceptions as signals).
"""
import logging; logger = logging.getLogger("ranger.actions")

logger.setLevel(logging.DEBUG)

import sys

MAX_FUTURES = 20

try:
    from concurrent.futures import Future, TimeoutError
except ImportError:
    import sys
    sys.stderr.write("[error] install python-concurrent.futures\n")
    sys.exit(1)

import weakref
import threading 
import thread # for get_ident

import traceback

from ranger.signals import ActionCancelled, ActionPaused


class PausableThread(threading.Thread):
    """ Based on http://ideone.com/HBvezh
    """
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
            #logger.warning("Tracing function already registered (debugger?). Task cancellation/pause won't be available.")
        else:
            self.debugger_trace = None
        #else:
        self.__cancel = False
        self.__pause = False
        sys.settrace(self.__signal_emitter)

        self.name = "Ranger action thread (initialization)"
        super(PausableThread, self)._Thread__bootstrap()

    def __signal_emitter(self, frame, event, arg):
        if self.__cancel:
            self.__cancel = False
            logger.debug("Cancelling thread <%s>:" % self.name)
            logger.debug("".join(traceback.format_stack(frame, limit = 3)))
            raise ActionCancelled()
        if self.__pause:
            self.__pause = False
            logger.debug("Pausing thread <%s>" % self.name)
            raise ActionPaused()

        if self.debugger_trace:
            return self.debugger_trace
        else:
            return self.__signal_emitter

class RobotActionThread(PausableThread):
    def __init__(self, future, fn, args, kwargs):
        PausableThread.__init__(self)
        self.future = future
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    def run(self):

        if not self.future.set_running_or_notify_cancel():
            return

        try:
            result = self.fn(*self.args, **self.kwargs)
        except BaseException:
            e = sys.exc_info()[1]
            logger.error("Exception in action <%s>: %s"%(self.fn.__name__, e))
            self.future.set_exception(e)
        else:
            self.future.set_result(result)


class RobotAction(Future):
    def __init__(self, actionname):
        Future.__init__(self)

        self.actionname = actionname
        self.thread = None

    def set_thread(self, thread):
        self.thread = thread

    def cancel(self):
        thread = self.thread() # weakref!
        if self.done() or not thread:
            return

        cancelled = super(RobotAction, self).cancel()

        if not cancelled: # already running
            thread.cancel()
            try:
                self.exception(timeout = 1) # waits this amount of time for the task to effectively complete
            except TimeoutError:
                raise RuntimeError("Unable to cancel action %s (still running 1s after cancellation)!" % self.actionname)

    def result(self):
        threading.current_thread().name = "Action waiting for sub-action %s" % self.actionname


        # active wait! Instead of blocking on the condition variable in super.result()
        # we do an active wait to make sure we can cancel/suspend the action via our
        # __signal_emitter trace function
        while True:
            try:
                return super(RobotAction, self).result(0.1)
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
        """ Overrides the representation function to 
        first wait for the result of the future.
        """
        return self.result().__repr__()


class RobotActionExecutor():

    def __init__(self):

        self.futures = []


    def submit(self, fn, *args, **kwargs):
        name = fn.__name__
        if args and not kwargs:
            name += "(%s)" % ", ".join([str(a) for a in args[1:]]) # start at 1 because 0 is the robot instance
        elif kwargs and not args:
            name += "(%s)" % ", ".join(["%s=%s" % (str(k), str(v)) for k, v in kwargs.items()])
        elif args and kwargs:
            name += "(%s, " % ", ".join([str(a) for a in args[1:]])
            name += "%s)" % ", ".join(["%s=%s" % (str(k), str(v)) for k, v in kwargs.items()])

        if len(self.futures) > MAX_FUTURES:
            logger.error("You have more than %s actions running in parallel! Likely a bug in your application logic! I skip action <%s>." % (MAX_FUTURES, name))
            return


        f = RobotAction(name)

        t = RobotActionThread(f, fn, args, kwargs)
        f.set_thread(weakref.ref(t))
        t.start()

        self.futures.append(f)
        return f

    def cancel_all(self):
        """ Blocks until all the currently running actions are actually stopped.
        """
        for f in self.futures:
            f.cancel()

        self.futures = []
