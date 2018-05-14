#ping_monitor

=====


###**Project Synopsis**
Graphs ping times in real-time.  Also highlights a given system's name
in the legend if the ping fails.


###**Introduction**
Each "robot" (e.g. computer) is pinged... (TODO: write this)


###**Install & Run**

- Open config.cfg and add the robot names
 - *(add an ip address for each, though they don't matter--still working on that!)*
 
 
````
git clone https://csrobot@bitbucket.org/umlroboticslab/ping_monitor.git
cd ping_monitor
python ping_monitor.py
```


###**TODO**

-Need to make the ping call asyncronous/in separate threads for each
system.  (right now it does it sequentially on the main thread :P )