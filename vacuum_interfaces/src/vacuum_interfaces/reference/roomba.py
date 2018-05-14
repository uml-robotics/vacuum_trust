from generic_vacuum import VacuumInterface
import Queue
import serial
import RPi.GPIO as GPIO
import struct
import time
import threading
from timer import Timer
#import numpy 

DEVICE_DETECT = 7
debug = False

#class RoombaHWState(gen_enum_class("RoombaHWState","unknown","poweroff","passivemode","safemode","fullmode")):
#    """ Roomba Hardware States"""
#    pass
# is_unknown() is_poweroff() is_passivemode() ...
# set_unknown() set_poweroff() set_passivemode() ...
# set("unknown") set("passivemode")
# get() -> "unknown"
# RoombaHWState.UNKOWN == "unknown"

class RoombaSharedInterface(VacuumInterface):
    def __init__(self, device, baud):
	VacuumInterface.__init__(self)
	self.dev = device
        self.robot = serial.Serial(self.dev, baud)
       
        # Keep Alive Code
        #setup GPIO to reference pins based the board
        GPIO.setmode(GPIO.BOARD) 
        #device_detect pin is the pin we set low to turn on the robot, we must 
        #set it to out mode for rasing high and low
        GPIO.setup(DEVICE_DETECT, GPIO.OUT) 
       
        self._buttons = {}
        self._old_button_state=0 
        self._sensor_data = []
        self._sensor_data_lock = threading.Lock()
        self._opcode_lock = threading.Lock()

        #A queue for running jobs
        self._job_queue = Queue.Queue()  
        #timer for pulling jobs off the job queue
        self._job_runner = Timer(0.01, self.job_runner_cb)
        self._job_runner.start()

        #sensor thread
        self._poll_sensor_timer = Timer(0.05,self._poll_sensors) 
        self._poll_sensor_timer.start()

        #keep alive thread
        self._keep_alive_timer = Timer(1, self.keep_alive)       
        self._keep_alive_timer.start()
         
    def job_runner_cb(self):
        if not self._job_queue.empty():
            function = self._job_queue.get()
           # print("FUNCTION: " )
            function()
        #print("Jobs: " + str(self._job_queue.qsize())) 


    def keep_alive(self):
       # Keep alive timer callback. Throws a gpio pin up and down 
        GPIO.output(DEVICE_DETECT, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(DEVICE_DETECT, GPIO.HIGH)


    def _poll_sensors(self):
        raise NotImplementedError()

    def set_buttons(self, **kwargs):
        self.buttons=kwargs
        #here code will start a thread for checking the data packet for button 
        #presses, I would like to have some kind of event thread that
        #sleeps until a value change is detected. Not sure how to best implement this. 


    def dock_passed(self):
        """ Instruct the robot to go back to its dock """
        with self._opcode_lock:
            self.robot.write(struct.pack('i', 143))

    def dock(self):
        self._job_queue.put(self.dock_passed)



    def is_docked(self): 
        print("is_cleaning() not implemented.")  
        pass

    def clean_passed(self):
        """ Instruct the robot to begin cleaning """
        with self._opcode_lock:
            print("Cleaning")
            self.robot.write(struct.pack('i', 135))

    def clean(self):
        #print("Engqued clean")
        self._job_queue.put(self.clean_passed)        



    def is_cleaning(self):
        print("is_cleaning() not implemented.")  

 

    def pause_passed(self):
        """ Instruct the robot to stop cleaning, but not return to dock """
        with self._opcode_lock:
            self.robot.write(struct.pack('i', 130))    
 
    def pause(self):
        self._job_queue.put(self.pause_passed)


    def get_current(self):
        """ returns an integer related to the current draw """
        with self._sensor_data_lock:
             if len(self._sensor_data) < 26:
                 return
             # bytes 19 and 20 make up current draw which is a 16 bit signed int
             # between -32768 and 32767. Positive currents indicate charging while 
             # negative indicated being off the charger. Current draw exceeping
             # -1000 should imply the robot is cleaning. idle current draw is around -170
             byte1 = struct.unpack('B', self._sensor_data[19]) 
             byte2 =  struct.unpack('B', self._sensor_data[20])
             # would like to convert math to numpy or use a more straight foreward 
             # method of translating 2 bytes into a 16 bit signed int
             #val = numpy.sum(numpy.prod(byte1[0], 256), byte2[0])
             val = byte1[0] * 256 + byte2[0]
             if val > 32767:
                 val -= 65536
             print "val: " + str(val)
             return val 
 
    def terminate(self):
        #print "ROOMBA INTERFACE: KILLING KEEPALIVE TIMER"
        #self._job_queue= Queue.Queue() #empty queue
        self._keep_alive_timer.cancel()
        print("keep alive killed")
        self._poll_sensor_timer.cancel()
        print("sensor_timer_killed")
        self._job_runner.cancel()
        print("job runner killed")
        time.sleep(1)
        GPIO.cleanup()
        self.robot.close()
        print "Clean shutdown achieved - cheers!" 
