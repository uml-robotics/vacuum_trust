###**Chris Code**

###**Install and source these required ros-packages**###
git@bitbucket.org:umlroboticslab/robot_monitor.git  
git@bitbucket.org:umlroboticslab/mjpeg_network_camera2.git

###**Run**
```
roscd vacuum_wizard_controls
./scripts/interface.bash
```


=====


###**James Code**


###**Install and source these required ros-packages**###
git@bitbucket.org:umlroboticslab/vacuum_wizard_controls.git
git@bitbucket.org:umlroboticslab/vacuum_experiment_msgs.git


###**Run**



- Modify the ```config.cfg``` file to specify the robots
- Make sure you are using the same version of vacuum_experiment_msgs with this node and on the robots

```
cd [your-vacuum-workspace]
roscore
rosrun vacuum_wizard_controls telemetry_chatter.py [robot-name]      #optional: of no real robots are used) 
rosrun vacuum_wizard_controls phonereply_chatter.py [robot-name]      #optional: of no real robots are used) 
#...  run as many of these chatter-nodes as you'd like; just make sure they match your config.cfg file!
rosrun vacuum_wizard_controls wizard_controls.py
```
