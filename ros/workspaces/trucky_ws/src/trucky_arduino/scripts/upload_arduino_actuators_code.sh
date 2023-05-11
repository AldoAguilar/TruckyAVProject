#!/bin/bash
PWD=`pwd`
roscd trucky_arduino
cd ../..
catkin_make trucky_arduino_firmware_actuators
catkin_make trucky_arduino_firmware_actuators-upload
cd $PWD