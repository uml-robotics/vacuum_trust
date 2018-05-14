import RPi.GPIO as GPIO
def gpio_sensor(pin, cb_fun):
    """ Register a callback whenever gpio pin changes state """
    def _cb(channel):
        if not channel == pin:
            print "heard back strange channel %d for cb pin %d" % (channel, pin)
        cb_fun(not bool(GPIO.input(pin)))
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(pin, GPIO.BOTH, callback=_cb)
    return
