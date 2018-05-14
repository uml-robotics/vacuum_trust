""" A nice logging function for me

import logging
from lagerlogger import LagerLogger 
logger = LagerLogger("mymodule")
logger.console(logging.INFO)

"""

import logging
import logging.handlers
import color
# import sys
import inspect

# http://code.activestate.com/recipes/474089-extending-the-logging-module/
# http://code.activestate.com/recipes/86900/
# http://spyhce.com/blog/understanding-new-and-init
# https://lists.gt.net/python/python/691963

class LagerLogger(logging.Logger):
    """ King of Loggers """
    FATAL = logging.FATAL
    ERROR = logging.ERROR
    WARNING = logging.WARNING
    WARN = logging.WARN
    INFO = logging.INFO
    DEBUG = logging.DEBUG
    def __new__(cls, name=None, time=None, level=None):
        tname = name if name else inspect.currentframe().f_back.f_globals['__name__']
        tmp = logging.getLoggerClass()
        logging.setLoggerClass(LagerLoggerImpl)
        this = logging.getLogger(tname)
        logging.setLoggerClass(tmp)
        return this

class LagerLoggerImpl(logging.getLoggerClass()):
    def __init__(self, name=None, time=None, level=None):
        if not name == inspect.currentframe().f_back.f_back.f_back.f_back.f_globals['__name__']:
            self.format_str = "[%(levelname)-8s][%(module)s/%(name)s] %(message)s"
        else:
            self.format_str = "[%(levelname)-8s][%(module)s] %(message)s"
        if time:
            self.format_str = "%(asctime)s " + self.format_str
        self._console = None
        self._logfile = None

        logging.Logger.__init__(self, name, self.__level(level))
        self.propagate = False # Don't pass up the chain

    def __level(self, lvl):
        return lvl if lvl is not None else logging.DEBUG

    def console(self, level):
        """ adds a console handler """
        if not self._console:
            ch = logging.StreamHandler()
            ch.setLevel(self.__level(level))
            ch.setFormatter(color.ColoredFormatter(self.format_str))
            self.addHandler(ch)
            self._console = ch
        else:
            self._console.setLevel(self.__level(level))

    def logfile(self, level, path=None):
        if not self._logfile:
            if path is None:
                path = "log.log"
            try:
                # Attempt to set up the logger with specified log target
                open(path, "a").close()
                hdlr = logging.handlers.RotatingFileHandler(path, maxBytes=500000, backupCount=5)
                hdlr.setLevel(self.__level(level))
                hdlr.setFormatter(color.ColoredFormatter(self.format_str, use_color=False))
            except IOError:
                logging.error('Failed to open file %s for logging' % logpath, exc_info=True)
                sys.exit(1)
            self.addHandler(hdlr)
            self._logfile = hdlr
        else:
            raise Exception("Opened logfile handler more than once!")

