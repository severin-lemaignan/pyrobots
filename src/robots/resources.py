from threading import Lock
import time

class Resource:
    def __init__(self, name = ""):
        self.lock = Lock()
        self.name = name

    def __str__(self):
        return self.name

    def __enter__(self):
        """
        Entering a 'resource' block *release* the lock, which may seem counter-intuitive.

        It is meant to used inside an action that lock the resource, to temporarly transfer the
        lock ownership to a sub-action:

        For instance:

        @action
        @lock(WHEELS)
        def move(...):
            ...

        @action
        @lock(WHEELS)
        def goto(...):

            with WHEELS:
                move(...)

        Here, goto() calls move() by first releasing the lock on WHEELS, executing move() and reacquiring the lock,
        also if move() raises an exception.
        """
        self.release()

    def __exit__(self, exc_type, exc_value, traceback):
        self.acquire()
        # here, the exception, if any, is automatically propagated

    def acquire(self, wait = True):

        if not wait:
            return self.lock.acquire(False)
        else:
            # we need an active wait to make sure we can properly cancel the actions
            # that are waiting for the resource
            while True:
                if self.lock.acquire(False):
                    return True
                time.sleep(0.1)

    def release(self):
        self.lock.release()


class CompoundResource:
    def __init__(self, *args, **kwargs):
        self.resources = args
        self.name = kwargs.get("name", "")

    def __str__(self):
        return self.name


    def __enter__(self):
        """ cf doc of Resource.__enter__.
        """
        self.release()

    def __exit__(self, exc_type, exc_value, traceback):
        """ cf doc of Resource.__exit__.
        """
        self.acquire()
        # here, the exception, if any, is automatically propagated



    def acquire(self, wait = True):
        for res in self.resources:
            res.acquire(wait)

    def release(self):
        for res in self.resources:
            res.release()

class ResourceLockedError(RuntimeError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


LPUPIL = Resource("left pupil")
RPUPIL = Resource("right pupil")
LLID = Resource("left lid")
RLID = Resource("right lid")
LEYE = CompoundResource(LPUPIL, LLID, name = "left eye")
REYE = CompoundResource(RPUPIL, RLID, name = "right eye")
LIDS = CompoundResource(LLID, RLID, name = "eyelids")
EYES = CompoundResource(LEYE, REYE, name = "eyes")
WHEELS = Resource("wheels")
AUDIO = Resource("audio")
LEDS = Resource("LEDs")
