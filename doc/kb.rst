Interaction with a knowledge base
=================================

``pyRobots`` can be effectively used in combination with a KB-API knowledge base like `minimalKB <https://github.com/severin-lemaignan/minimalkb>`_ by using the `pykb <https://github.com/severin-lemaignan/pykb>`_ Python bindings.

Integration example
-------------------

.. code:: python

    from functools import partial
    from robots import GenericRobot

    import kb # pip install pykb
    kb = kb.KB() # by default, attempts to connect on localhost:6969

    def onDesire(robot, desire):
            print("The human wants something!")
            action = kb[desire + " rdf:type ?type"]
            object = kb[desire + " actsOnObject ?object"]

            if action == "Give":
                robot.give(object)

            if action == "Get":
                place = kb[object + " isAt ?place"] # isOn, isIn...
                robot.goto(place)
                robot.pick(object)


    with GenericRobot() as robot: # GenericRobot to be replaced by your own specialized class

        # the callback 'onDesire' will be invoked as soon as a
        # statement matching 'HUMAN desires *' is added to the 
        # knowledge base.
        kb.subscribe(["HUMAN desires ?action"], partial(onDesire, robot))

        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

