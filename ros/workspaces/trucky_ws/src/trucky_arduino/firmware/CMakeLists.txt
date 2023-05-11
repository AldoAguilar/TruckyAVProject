cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(sensors
  SRCS arduino_sensors.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyUSB0
)

generate_arduino_firmware(actuators
  SRCS arduino_actuators.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyUSB0
)