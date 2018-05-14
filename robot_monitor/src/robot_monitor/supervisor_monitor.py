import xmlrpclib
from socket import error as socket_error

class SupervisorMonitor(object):
    def __init__(self, url):
        # We put the server on port 5000
        # http://supervisord.org/api.html
        self.server = xmlrpclib.Server("http://%s:5000/RPC2" % url)

    def controller_status(self):
        try:
            ret = self.server.supervisor.getProcessInfo("controller")
            return ret['statename']
        except socket_error:
            return "COMFAIL"

    def start_controller(self):
        try:
            return self.server.supervisor.startProcess("controller")
        except socket_error:
            return False

    def stop_controller(self):
        try:
            return self.server.supervisor.stopProcess("controller")
        except socket_error:
            return False


