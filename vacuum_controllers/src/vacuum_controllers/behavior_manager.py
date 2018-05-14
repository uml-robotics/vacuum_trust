from lagerlogger import LagerLogger
class RobotBehavior(object):
    def __init__(self, behavior_stack):
        self.logr = LagerLogger(self.__class__.__name__)
        self.logr.console(LagerLogger.DEBUG)
        self.behavior_stack = behavior_stack

    def start(self):
        if self in self.behavior_stack:
            self.logr.error("start() already started - stack position %d" % self.behavior_stack.index(self))
            return
        self.behavior_stack.append(self)
        self.logr.info("start() %s" % str([x.__class__.__name__ for x in self.behavior_stack]))

    def cancel(self):
        try:
            n = self.behavior_stack.index(self)
            self.behavior_stack.pop(n)
            self.logr.info("cancel() %s" % str([x.__class__.__name__ for x in self.behavior_stack]))
        except ValueError:
            self.logr.error("cancel() not started yet")

    def __nonzero__(self): 
        return self.__bool__()
    def __bool__(self):
        return self in self.behavior_stack
    def active(self):
        return self.__bool__()


class BehaviorStack(list):
    def __init__(self):
        super(BehaviorStack, self).__init__()
        self.logr = LagerLogger("BehaviorStack")
        self.logr.console(LagerLogger.DEBUG)

    def __getattr__(self, name):
        try:
            return super(BehaviorStack,self).__getattr__(name)
        except AttributeError:
            self.logr.debug("Trying to call '%s' from the stack" % name)
        if len(self) == 0:
            self.logr.error("Could not call '%s', no behaviors are on the stack. %s: %s" % (name, str(type(e)), e))
            return
        for i in range(1, len(self)+1):
            try:
                return self[-i].__getattribute__(name)
            except IndexError as e:
                self.logr.debug("Could not call '%s', no behaviors are on the stack. %s: %s" % (name, str(type(e)), e))
            except AttributeError as e:
                self.logr.debug("Active behavior: %s" % e)
            except Exception as e:
                self.logr.debug("Could not call '%s' on behavior. %s: %s" % (name, str(type(e)), e))
            self.logr.warn("Behavior '%s' undefined for %s, lowering the bar." % (name, self[-i].__class__.__name__))

        self.logr.error("Behavior '%s' undefined for all robot behaviors" % name)
        return BehaviorStack.__noop__

    @staticmethod
    def __noop__(*args, **kwargs):
        pass

