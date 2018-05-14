#!/usr/bin/env python
import os, platform
import subprocess as sp
import socket  # for gethostbyname()
from Queue import Queue
import threading
import paramiko

import re # for PingParser
import sys  # for PingParser
from supervisor_monitor import SupervisorMonitor

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


class AsyncSubprocess(threading.Thread):
    def __init__(self, command):
        threading.Thread.__init__(self)
        self._queue = Queue()
        self._command = command
        self._fd = None

    def run(self):
        """ Run the command, put any output into a queue """
        self._fd = sp.check_output(self._command, stdout=sp.PIPE, stderr=sp.PIPE)
        for line in iter(self._fd.readline, ''):
            self._queue.put(line)

    def eof(self):
        return not self.is_alive() and self._queue.empty()



class RemoteConnection(object):
    def __init__(self, dns=None, user=None, passwd=None):
        self._dns = dns
        self._user = user
        self._passwd = passwd
        self._ssh = paramiko.SSHClient()
        self._ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self._connected = False
        
    def get_ip(self):
        return socket.gethostbyname(self._dns)

    def ping(self, n=1):
        """ Pings the host n times, returns output """
#         print "ping", self._dns
        rslt = sp.check_output(("ping -c %d -W 1 %s" % (n, self._dns)).split(), stderr=sp.PIPE, universal_newlines=True)
        return ping_parse(rslt)

    def ping_test(self):
        try:
            rslt = sp.check_output(("ping -c %d -W 1 %s" % (n, self._dns)).split(), stderr=sp.PIPE, universal_newlines=True)
            if not float(ping_parse(rslt)['packet_loss']) == 0.0:
                return False
        except:
            return False
        return True



    def connected(self):
        """ Return True if ssh is connected, else False """
#         try:
#             if not float(self.ping()['packet_loss']) == 0.0:
#                 print "packet lost"
#                 return False
#         except Exception as e:
#             print e
#             return False
#         ssh_transport = self._ssh.get_transport()
#         if not ssh_transport:
#             print "no transport"
#             return False
#         return self._ssh.get_transport().is_alive()
        return self._connected

    def connect(self, timeout=None):
        """ Establish an SSH connection """
#         print "connecting"
        if not float(self.ping()['packet_loss']) == 0.0:
            print "robot not around"
            raise Exception("Robot not available")
        self._ssh.connect(self._dns, username=self._user, password=self._passwd, timeout=timeout)
        self._connected = True

    def exec_simple(self, cmd, timeout=None):
        """ Returns the output of a remotely run command  """
#         print "exec(%s)" % cmd
        if not self._connected:
            raise Exception("Not Connected")
        try:
            stdin, stdout, stderr = self._ssh.exec_command(cmd, timeout=timeout)
        except socket.error as e:
            # we are no longer connected
            print "WE HAVE DISCONNECTED"
            self._connected = False
            raise e
        errors = stderr.readlines()
        if errors:
            raise Exception(errors)
        return [x.encode('ascii') for x in stdout.readlines()]

    def reboot(self):
        try:
            stdin, stdout, stderr = self._ssh.exec_command("sudo reboot now")
        except socket.error as e:
            self._connected = False
            raise e
        stdin.write(self._passwd)
        stdin.flush()
        errors = stderr.readlines()
        if errors:
            raise Exception(errors)
        return [x.encode('ascii') for x in stdout.readlines()]




class RemoteMachine(RemoteConnection):
    def __init__(self, dns=None, user=None, passwd=None):
        RemoteConnection.__init__(self, dns=dns, user=user, passwd=passwd)
        self._supervisor = SupervisorMonitor(dns)

    def get_cpu(self):
        """ Returns the remote cpu info %usr %nice %sys %iowait %irq %soft %steal %guest %gnice %idle """
        result = self.exec_simple("mpstat 1 1", timeout=3)
        return dict(zip(result[2].split()[2:], result[3].split()[2:]))
        
    def get_mem(self):
        """ Returns total, used, free as strings, in M"""
        #http://askubuntu.com/a/155771
        result = self.exec_simple("free -m", timeout=3)
        used, free = result[2].split()[-2:]
        used = int(used.strip())
        free = int(free.strip())
        total = used + free
        return {"total": total, "used": used, "free": free}

    def get_uptime(self):
        """ Returns the system uptime """
        result = self.exec_simple("uptime -p", timeout=3)
        result = result[0]
        return result

    def controller_status(self):
        return self._supervisor.controller_status()

    def start_controller(self):
        return self._supervisor.start_controller()

    def stop_controller(self):
        return self._supervisor.stop_controller()
        


if __name__ == "__main__":
    m = RemoteMachine("neato.lan", "pi", "test")
    m.connect()
    print m.ping()['avgping']
    print m.get_cpu()
    print m.get_mem()

