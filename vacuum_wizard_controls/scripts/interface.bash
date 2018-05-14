#!/bin/bash
roscd vacuum_wizard_controls
rqt -l --perspective-file=resources/experiment.perspective &
roslaunch mjpeg_network_camera2 3camera.launch
