# coding=utf-8
from threading import Lock
import time

class Resource:
    def __init__(self, name = ""):
        self.lock = Lock()
        self.name = name
        self.owner = None

    def __str__(self):
        return self.name + ((" (currently owned by <%s>)" % self.owner) if self.owner else " (not currently owned)")

    def __enter__(self):
        """
        Entering a 'resource' block *release* the lock, which may seem counter-intuitive.

        It is meant to used inside an action that lock the resource, to temporarly transfer the
        lock ownership to a sub-action:

        For instance:
        
        .. code-block::python

            @action
            @lock(WHEELS)
            def move(...):
                ...

            @action
            @lock(WHEELS)
            def goto(...):

                with WHEELS:
                    move(...)

        Here, ``goto()`` calls ``move()`` by first releasing the lock on
        ``WHEELS``, executing ``move()`` and reacquiring the lock, also if
        ``move()`` raises an exception.
        """
        self.release()

    def __exit__(self, exc_type, exc_value, traceback):
        self.acquire()
        # here, the exception, if any, is automatically propagated

    def acquire(self, wait = True, acquirer = "unknown"):

        if not wait:
            if self.lock.acquire(False):
                self.owner = acquirer
                return True
            else:
                return False
        else:
            # we need an active wait to make sure we can properly cancel the actions
            # that are waiting for the resource
            while True:
                if self.lock.acquire(False):
                    self.owner = acquirer
                    return True
                time.sleep(0.1)

    def release(self):
        self.lock.release()
        self.owner = None


class CompoundResource:
    def __init__(self, *args, **kwargs):
        self.resources = args
        self.name = kwargs.get("name", "")
        self.owner = None

    def __str__(self):
        return self.name + ((" (currently owned by <%s>)" % self.owner) if self.owner else " (not currently owned)")


    def __enter__(self):
        """ cf doc of Resource.__enter__.
        """
        self.release()

    def __exit__(self, exc_type, exc_value, traceback):
        """ cf doc of Resource.__exit__.
        """
        self.acquire()
        # here, the exception, if any, is automatically propagated



    def acquire(self, wait = True, acquirer = "unknown"):
        ok = True
        for res in self.resources:
            ok = res.acquire(wait, acquirer) and ok

        if not ok:
            return False

        self.owner = acquirer

    def release(self):
        for res in self.resources:
            res.release()
        self.owner = None

