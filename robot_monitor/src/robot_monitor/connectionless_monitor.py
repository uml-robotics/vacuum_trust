#!/usr/bin/env python
import xmlrpclib
from socket import error as socket_error
import os, platform
import socket  # for gethostbyname()

import subprocess as sp
from Queue import Queue
import threading
# import paramiko

import re # for PingParser
import sys  # for PingParser
import time

# Requires sysstat on remote machine, and remote machines to be linux

def ping_parse(ping_output):
    """ PingParser - parse ping output string into a dictionary
    Based on code from  https://github.com/ssteinerx/pingparser commit 80d7185
    Reorganized by [db] """
    host_matcher = re.compile(r'PING ([a-zA-Z0-9.\-]+) *\(')
    rslt_matcher = re.compile(r'(\d+) packets transmitted, (\d+) (?:packets )?received, (\d+\.?\d*)% packet loss')
    # Pull out round-trip min/avg/max/stddev = 49.042/49.042/49.042/0.000 ms
    minmax_matcher = re.compile(r'(\d+.\d+)/(\d+.\d+)/(\d+.\d+)/(\d+.\d+)')

    host = host_matcher.search(ping_output).groups()[0]
    sent, received, packet_loss = rslt_matcher.search(ping_output).groups()

    try:
        minping, avgping, maxping, jitter = minmax_matcher.search(ping_output).groups()
    except:
        minping = avgping = maxping = jitter = 'NaN'

    return {'host': host, 'sent': sent, 'received': received, 'packet_loss' : packet_loss,
            'minping': minping, 'avgping': avgping, 'maxping': maxping, 'jitter': jitter }



class RemoteMachine(object):
    def __init__(self, dns=None):
        self._dns = dns
        self._host = xmlrpclib.ServerProxy("http://%s:5050/RPC2" % dns)
        self._server = xmlrpclib.Server("http://%s:5000/RPC2" % dns)
        self._timeout = 3
        socket.setdefaulttimeout(self._timeout)

    def get_ip(self):
        socket.setdefaulttimeout(5)
        ip= socket.gethostbyname(self._dns)
        socket.setdefaulttimeout(self._timeout)
        return ip

    def ping(self, n=1):
        """ Pings the host n times, returns output """
        socket.setdefaulttimeout(5)
        p = sp.Popen(("ping -c %d -W 1 %s" % (n, self._dns)).split(), stdout=sp.PIPE, stderr=sp.PIPE, universal_newlines=True)        
#         rslt = sp.check_output(("ping -c %d -W 1 %s" % (n, self._dns)).split(), stderr=sp.PIPE, universal_newlines=True)
        for i in range(5*10):
            retcode = p.poll()
            if isinstance(retcode, int):
                break
            time.sleep(.1)
        if not isinstance(retcode, int):
            p.terminate()
            logger.error("ping did not terminate correctly")
        rslt = "".join(p.stdout.readlines())
        socket.setdefaulttimeout(self._timeout)
        return ping_parse(rslt)

    def check_server(self):
        # Throws an exception if the server is not ready
        socket.setdefaulttimeout(self._timeout)
        self._host.system.listMethods()
        return True

    def get_telemetry(self):
        """ Returns cpu, mem, and uptime all at once in a dictionary"""
        socket.setdefaulttimeout(self._timeout)
        return self._host.get_telemetry()

    def controller_status(self):
        try:
            ret = self._server.supervisor.getProcessInfo("controller")
            return ret['statename']
        except socket_error:
            return "COMFAIL"

    def start_controller(self):
        try:
            return self._server.supervisor.startProcess("controller")
        except socket_error:
            return False

    def stop_controller(self):
        try:
            return self._server.supervisor.stopProcess("controller")
        except socket_error:
            return False

    def reboot(self):
        try:
            return self._server.supervisor.startProcess("reboot")
        except socket_error:
            return False



if __name__ == "__main__":
    m = RemoteMachine("neato.lan", "pi", "test")
    m.connect()
    print m.ping()['avgping']
    print m.get_cpu()
    print m.get_mem()

