# coding=utf-8
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

