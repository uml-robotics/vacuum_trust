#!/usr/bin/env python
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
import subprocess as sp
import threading
import time
import sys

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

class TelemetryServer(object):
    def __init__(self):
        self.cpu = 0
        self.cpu_lock = threading.Lock()
        self.mem = 0
        self.mem_lock = threading.Lock()
        self.uptime = 0
        self.uptime_lock = threading.Lock()
        self.wireless = 0
        self.wireless_lock = threading.Lock()

        self.running = True
        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.start()

    def update(self):
        while self.running:
            result = sp.Popen("mpstat 1 1".split(), stdout=sp.PIPE).stdout.readlines()
            with self.cpu_lock:
                self.cpu = dict(zip(result[2].split()[2:], result[3].split()[2:]))

            result = sp.Popen("free -m".split(), stdout=sp.PIPE).stdout.readlines()
            used, free = result[2].split()[-2:]
            used = int(used.strip())
            free = int(free.strip())
            total = used + free
            with self.mem_lock:
                self.mem = {"total": total, "used": used, "free": free}

            result = sp.Popen("uptime -p".split(), stdout=sp.PIPE).stdout.readlines()
            with self.uptime_lock:
                self.uptime = result[0]

#             result = sp.Popen("/sbin/iwconfig wlan0".split(), stdout=sp.PIPE).stdout.readlines()
#             bitrate = result[2].strip()
#             linkquality, siglevel, noiselevel = result[4].strip().split("  ")
#             with self.wireless_lock:
#                 self.wireless = {'bitrate': bitrate, 'linkquality': linkquality, 'siglevel': siglevel, 'noiselevel': noiselevel}
            time.sleep(1)
    
    def shutdown(self):
        self.running=False
        self.update_thread.join()

    def get_telemetry(self):
        telemetry = dict()
        telemetry["cpu"] = self.get_cpu()
        telemetry["mem"] = self.get_memory()
        telemetry["uptime"] = self.get_uptime()
        return telemetry

    def get_cpu(self):
        with self.cpu_lock:
            return self.cpu
       
    def get_memory(self):
        with self.mem_lock:
            return self.mem

    def get_uptime(self):
        with self.uptime_lock:
            return self.uptime

#     def get_wireless(self):
#         with self.wireless_lock:
#             return self.wireless

try:
    telemetry = TelemetryServer()
    server = SimpleXMLRPCServer(("0.0.0.0", 5050),
                                requestHandler=RequestHandler)
    server.register_introspection_functions()
    server.register_instance(telemetry)
    server.serve_forever()
except KeyboardInterrupt:
    telemetry.shutdown()

