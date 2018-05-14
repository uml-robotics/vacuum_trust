from lagerlogger import LagerLogger 
import sys
from threading import Thread

class Response(dict):
    def __init__(self, val, disp_val):
        dict.__init__(self, {"id": val, "value": disp_val})
        self.logr = LagerLogger("%sResponse" % disp_val)
        self.logr.console(LagerLogger.DEBUG)
        self._cb_fun = None
    
    def _do_cb(self):
        self.logr.debug("Start callback")
        sys.stdout.flush()
        self._cb_fun()
        self.logr.debug("End callback")
        sys.stdout.flush()

    def __call__(self):
        """ Calls this responses callback """
        if self._cb_fun:
            self._do_cb()
            # [db] Removed threading, replaced with threading in payload pipe
#             t = Thread(target=self._cb_fun)
#             t = Thread(target=self._do_cb)
#             t.start()

    def set_callback(self, fun):
        """ sets a callback function to associate with this response """
        self._cb_fun = fun

    def cast(self):
        """ returns the simplified version of this python object - 
        in other words in gets rid of the extra functionality we got from subclassing"""
        return dict(self)


class Message(dict):
    def __init__(self, msgid, content):
        dict.__init__(self, {})
        self.logr = LagerLogger("Message")
        self.logr.console(LagerLogger.DEBUG)
        self.tag = None  # This is a free object you can set to identify your message with
        self["msgid"] = msgid
        self["content"] = content
        self["popup"] = False

    def set_content(self, message):
        self["content"] = message

    def set_selection(self, rspid):
        """ set the selected response id if a response has been given """
        if "responses" not in self.keys():
            self.logr.warn("tried to set response selection '%s' for message with no response '%s'" % (rspid, self["content"]))
            return

        if int(rspid) not in range(len(self["responses"])):
            self.logr.warn("tried to set response invalid selection '%s' for message with response '%s', %s" % (rspid, self["content"], str(self["responses"])))
            return

        self["selection"] = rspid


    def unset_selection(self):
        if "selection" in self.keys():
            self.pop("selection")

    def set_popup(self):
        self["popup"] = True

    def unset_popup(self):
        self["popup"] = False

    def add_response(self, disp_val, cb_fun):
        """ add a possible response"""
        if "responses" not in self.keys():
            self["responses"] = list()
        r = Response(str(len(self["responses"])), disp_val)
        r.set_callback(cb_fun)
        self["responses"].append(r)
        return r


class ConversationStack(list):
    def __init__(self):
        list.__init__(self)
        self.extra = "extrathing"

    def add_message(self, content):
        m = Message(str(len(self)), content) 
        self.append(m)
        return m

    def run_cb(self, msgid, rspid):
        self[msgid]["responses"][rspid]()

    def clear(self):
        """ empty the entire stack """
        self[:] = [] # Clear everything - http://stackoverflow.com/a/850831


# stack = ConverstationStack()
# m = stack.add_message("cleaning")
# m.content("Cleaning.")
# m.set_response("Pause", self._clean_cb)
# m.set_response("Dock", self._dock_cb)
# stack.pop_message()
def pause_cb():
    print "pause"

def dock_cb():
    print "dock"

if __name__ == "__main__":
    stack = ConversationStack()
    m = stack.add_message("Cleaning")
    m.add_response("pause", pause_cb)
    m.add_response("dock", dock_cb)
    print stack

#     import ujson as json
    import json
    print json.dumps(stack)

    stack.run_cb(0,0)


