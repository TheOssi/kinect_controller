# kinect_controller
ROS-Node that uses the messages from openni_tracker on /tf to control a ARDrone2.0 by using FuzzyLogic

## Requirements
* Ubuntu 14.04
* ROS Indigo
* Tum Simulator Package for ROS Indigo: https://github.com/dougvk/tum_simulator
* OpenNI + openni_tracker: https://github.com/jstnhuang/cse481c_tutorials/wiki/How-to-run-openni_tracker
* FuzzyLite: https://github.com/fuzzylite/fuzzylite
* If you intend using the real Drone: http://wiki.ros.org/ardrone_autonomy

## Running
* Start the Simulator, then run
```
roslaunch kinect_controller controller.launch
```

## Troubleshooting
* If Kinect Device is not found: try unplugging the kinect and restarting everything
* If this doesn't help install OpenNI freshly
* If the program crashes with, unplug kinect and and restart
