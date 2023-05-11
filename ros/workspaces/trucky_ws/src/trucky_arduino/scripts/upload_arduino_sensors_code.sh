#!/bin/bash
PWD=`pwd`
roscd trucky_arduino
cd ../..
catkin_make trucky_arduino_firmware_sensors
catkin_make trucky_arduino_firmware_sensors-upload
cd $PWD