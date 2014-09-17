import logging; logger = logging.getLogger("robots.actions")
import time

import threading

import robots
from resources import *
from robots.introspection import introspection
from robots.signals import ActionCancelled

def action(fn):
    """ When applied to a function, this decorator turns it into
    a asynchronous task, starts it in a different thread, and returns
    a 'future' object that can be used to query the result/cancel it/etc.

    Action implementation may want to handle the
    :class:`robots.signals.ActionCancelled` signal to properly process
    cancellation requests.

    Usage example:

    .. code-block:: python

        @action
        def safe_walk(robot):
        try:
            robot.walk()
        except ActionCancelled:
            robot.go_back_to_rest_pose()

        action = robot.safe_walk()
        time.sleep(1)
        action.cancel()

    In this example, after one second, the ``safe_walk`` action is cancelled.
    This sends the signal :class:`robots.signals.ActionCancelled` to the
    action, that can appropriately terminate.

    """

    # wrapper for the original function that locks/unlocks shared
    # resources
    def lockawarefn(future,actionname,*args, **kwargs):

        try:
            # we acquire resources *within the future thread* that
            # we want to *wait* for.
            if hasattr(fn, "_locked_res"):
                for res, wait in fn._locked_res:
                    if wait:
                        threading.current_thread().name = "Robot Action %s, waiting on resource %s" % (actionname, res) #fn.__name__
                        need_to_wait = False
                        if res.owner is not None:
                            need_to_wait = True
                            logger.info("Robot action <%s> is waiting on resource %s" % (actionname, res)) #fn.__name__
                        res.acquire(wait, acquirer = fn.__name__)
                        if need_to_wait:
                            logger.info("Robot action <%s> has acquired resource %s" % (actionname, res)) #fn.__name__
                        else:
                            logger.info("Robot action <%s> acquired free resource %s  " %(actionname, res))

        except ActionCancelled:
            # action cancelled while it was waiting for a resource to become
            # available
            threading.current_thread().name = "Idle Robot action thread"
            logger.debug("Action <%s> cancelled while it was waiting for a lock on a resource." % actionname) #fn.__name__
            return None
 
        try:
            future.has_acquired_resource = True
            threading.current_thread().name = "Robot Action %s (running)" % (actionname) #fn.__name__
            logger.debug("Starting action <%s> now." % actionname) #fn.__name__
            try:
                result = fn(*args, **kwargs)
            except TypeError:
                logger.error("Exception when invoking action <%s>. Did you forget to add the parameter 'robot'?" % actionname) #fn.__name__
                raise

            logger.debug("Action <%s> returned." % actionname) #fn.__name__
            return result
        except ActionCancelled:
            logger.warning("Action cancellation ignored by %s. Forced stop!" % actionname) #fn.__name__
        finally:
            if hasattr(fn, "_locked_res"):
                for res, wait in fn._locked_res:
                    res.release()

            threading.current_thread().name = "Idle Robot action thread"


    lockawarefn.__name__ = fn.__name__
    lockawarefn.__doc__ = fn.__doc__


    # wrapper that submits the function to the executor and returns
    # a future.
    def innerfunc(*args, **kwargs):

        if len(args) == 0 or not isinstance(args[0], robots.GenericRobot):
            raise Exception("No robot instance passed to the action!")
        
        robot = args[0]

        # we acquire resources *outside the future* (to fail fast)
        # for resources we do not want to wait for.
        if hasattr(fn, "_locked_res"):
            for res, wait in fn._locked_res:
                if not wait:
                    got_the_lock = res.acquire(False, acquirer = fn.__name__)

                    if not got_the_lock:
                        logger.info("Required resource <%s> locked while attempting to start %s. Cancelling it as required." % (res.name, fn.__name__))
                        return FakeFuture(None)

        if robot.immediate:
            res = FakeFuture(lockawarefn(*args, **kwargs))
            return res

        else:
            executor = robot.executor

            if args and kwargs:
                future = executor.submit(lockawarefn, *args, **kwargs)
            elif args:
                future = executor.submit(lockawarefn, *args)
            else:
                future = executor.submit(lockawarefn)

            if introspection:

                introspection.action_started(fn.__name__, 
                                            str("FUTURE ID BROKEN"),
                                            str("ACTION ID BROKEN TDB"), #id of the current action
                                            args[1:],
                                            kwargs)
                future.add_done_callback(lambda x : introspection.action_completed(fn.__name__, str("future.id BROKEN")))

            return future

    innerfunc.__name__ = fn.__name__
    innerfunc.__doc__ = fn.__doc__
    innerfunc._action = True

    return innerfunc

def lock(res, wait = True):
    """
    Used to define which resources are acquired (and locked)
    by the action.

    This decorator may be used as many times as required on the same function
    to lock several resources.

    Usage example:

    .. code-block:: python
    
        L_ARM = Resource()
        R_ARM = Resource()
        ARMS = CompoundResource(L_ARM, R_ARM)

        @action
        @lock(ARMS)
        def lift_box(robot):
            #...

        @action
        @lock(L_ARM)
        def wave_hand(robot):
            #...

        @action
        @lock(L_ARM, wait=False)
        def scratch_head(robot):
            #...

        robot.lift_box()
        robot.wave_hand() # waits until lift_box is over
        robot.scratch_head() # skipped if lift_box or
                            # wave_hand are still running



    :param res: an instance of Resource or CompoundResource
    :param wait: (default: true) if ``true``, the action will wait
                 until the resource is available, if ``false``, the action 
                 is skipped if the resource is not available.

    """
    def decorator(fn):
        if hasattr(fn, "_locked_res"):
            fn._locked_res.append((res, wait))
        else:
            fn._locked_res = [(res, wait)]

        return fn

    return decorator

class FakeFuture:
    """ Used in the 'immediate' mode.
    """

    def __init__(self, result):
        self._result = result
    def result(self):
        return self._result
    def wait(self):
        return self._result

