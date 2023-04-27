#!/bin/bash
PWD=`pwd`
cd ~/catkin_ws
catkin_make trucky_arduino_firmware_actuators
catkin_make trucky_arduino_firmware_sensors
catkin_make trucky_arduino_firmware_actuators-upload
catkin_make trucky_arduino_firmware_sensors-upload
cd $PWD