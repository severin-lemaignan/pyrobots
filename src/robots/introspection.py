# coding=utf-8
import logging; logger = logging.getLogger("robots.introspection")
import threading
introspection = None

# disable introspection for now
if False:
    try:
        import Pyro4
        import Pyro4.errors
        uri = "PYRONAME:robots.introspection" # uses name server
        try:
            introspection = Pyro4.Proxy(uri)
            introspection.initiate(str(0)) # 0 is the action ID of the main process
            logger.info("Connection to the introspection server established.")
        except Pyro4.errors.CommunicationError:
            logger.warning("Introspection server not running. No introspection.")
            introspection = None
        except Pyro4.errors.NamingError:
            logger.warning("Introspection server not running (no name server). No introspection.")
            introspection = None
    
    except ImportError:
        pass
