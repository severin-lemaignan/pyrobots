pyRobots overview
=================

Design principles
-----------------

- Control is based on *desires* (like *Move*, *Get*...) that are complex, conditional sets of *actions*.

Features
--------

- Supports ROS, pocolibs middlewares
- Foreground and background tasks
- Priorities between desires, possiblity to superseed a desire with another one, but no support for recovering supersed desires.
- Optionally interact with knowledge base (supporting the KB API -> oro server)
