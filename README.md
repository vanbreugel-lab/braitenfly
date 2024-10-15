# Braitenfly

This ROS package provides an interface for controlling the bitcraze crazyflie in the spirit of Valentino Braitenberg's vehicles. 

A variety of sensor-action modules are implemented, and each one can be turned on or off in the braitenfly_config.yaml object by commenting out modules that are not intended to be active.

### Installation

Requires the `rospy_crazyflie` package. 

### Usage

1. Edit the braitenfly_config.yaml module to turn on or off selected modules.
2. Run the crazyflie: `roslaunch ~/catkin_ws/src/rospy_crazyflie/launch/default.launch`
3. Navigate to `~/catkin_ws/src/braitenfly/src`
4. Run `rosrun braitenfly braitenfly.py --takeoff=1` to run the braitenfly code and have the crazyflie take off. Set takeoff=0 if you don't want the crazyflie to fly, but you want to test sensor-action module triggering. 