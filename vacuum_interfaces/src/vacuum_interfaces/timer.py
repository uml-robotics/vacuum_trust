import threading

class Timer(object):
        """ Timer fires on an interval, but next cb does not fire until the 
        callback has completed. Can also be restarted without reinstantiating """
        def __init__(self, time, cb):
            self._time = time
            self._cb = cb
            self._timer = None
            self._running = True
            self._spin = True  # This is for the spin() function
            self._lock = threading.RLock()

        def start(self):
            self._running = True
            self._start_timer()

        def cancel(self):
            with self._lock:
                self._running = False
                self._stop_timer()
	
	def fire_and_restart(self):
            print("resetting")
	    self._cbfun()
            print("reset")
                
        def _start_timer(self):
            if not self._running:
                return
            if self._timer:
                raise Exception("Timer already started")
            self._timer = threading.Timer(self._time, self._cbfun)
            self._timer.start()

        def _stop_timer(self):
            if self._timer:
                self._timer.cancel()
            self._timer = None

        def _cbfun(self):
            with self._lock: 
                self._stop_timer()
                self._cb()
                self._start_timer()

