# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aldo/catkin_ws/src/trucky_arduino/firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aldo/catkin_ws/build/trucky_arduino/firmware

# Include any dependencies generated for this target.
include CMakeFiles/sensors.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sensors.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sensors.dir/flags.make

CMakeFiles/sensors.dir/arduino_sensors.cpp.obj: CMakeFiles/sensors.dir/flags.make
CMakeFiles/sensors.dir/arduino_sensors.cpp.obj: /home/aldo/catkin_ws/src/trucky_arduino/firmware/arduino_sensors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldo/catkin_ws/build/trucky_arduino/firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sensors.dir/arduino_sensors.cpp.obj"
	/usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensors.dir/arduino_sensors.cpp.obj -c /home/aldo/catkin_ws/src/trucky_arduino/firmware/arduino_sensors.cpp

CMakeFiles/sensors.dir/arduino_sensors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensors.dir/arduino_sensors.cpp.i"
	/usr/bin/avr-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldo/catkin_ws/src/trucky_arduino/firmware/arduino_sensors.cpp > CMakeFiles/sensors.dir/arduino_sensors.cpp.i

CMakeFiles/sensors.dir/arduino_sensors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensors.dir/arduino_sensors.cpp.s"
	/usr/bin/avr-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldo/catkin_ws/src/trucky_arduino/firmware/arduino_sensors.cpp -o CMakeFiles/sensors.dir/arduino_sensors.cpp.s

CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.requires:

.PHONY : CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.requires

CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.provides: CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.requires
	$(MAKE) -f CMakeFiles/sensors.dir/build.make CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.provides.build
.PHONY : CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.provides

CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.provides.build: CMakeFiles/sensors.dir/arduino_sensors.cpp.obj


CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj: CMakeFiles/sensors.dir/flags.make
CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj: /home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aldo/catkin_ws/build/trucky_arduino/firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj"
	/usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj -c /home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp

CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.i"
	/usr/bin/avr-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp > CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.i

CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.s"
	/usr/bin/avr-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp -o CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.s

CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.requires:

.PHONY : CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.requires

CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.provides: CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.requires
	$(MAKE) -f CMakeFiles/sensors.dir/build.make CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.provides.build
.PHONY : CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.provides

CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.provides.build: CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj


# Object files for target sensors
sensors_OBJECTS = \
"CMakeFiles/sensors.dir/arduino_sensors.cpp.obj" \
"CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj"

# External object files for target sensors
sensors_EXTERNAL_OBJECTS =

/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf: CMakeFiles/sensors.dir/arduino_sensors.cpp.obj
/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf: CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj
/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf: CMakeFiles/sensors.dir/build.make
/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf: libuno_Wire.a
/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf: libuno_CORE.a
/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf: CMakeFiles/sensors.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aldo/catkin_ws/build/trucky_arduino/firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensors.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating EEP image"
	/usr/bin/avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 /home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf /home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.eep
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating HEX image"
	/usr/bin/avr-objcopy -O ihex -R .eeprom /home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf /home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.hex
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Calculating image size"
	/usr/local/bin/cmake -DFIRMWARE_IMAGE=/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf -DMCU=atmega328p -DEEPROM_IMAGE=/home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.eep -P /home/aldo/catkin_ws/build/trucky_arduino/firmware/CMakeFiles/FirmwareSize.cmake

# Rule to build all files generated by this target.
CMakeFiles/sensors.dir/build: /home/aldo/catkin_ws/devel/share/trucky_arduino/sensors.elf

.PHONY : CMakeFiles/sensors.dir/build

CMakeFiles/sensors.dir/requires: CMakeFiles/sensors.dir/arduino_sensors.cpp.obj.requires
CMakeFiles/sensors.dir/requires: CMakeFiles/sensors.dir/home/aldo/catkin_ws/build/trucky_arduino/ros_lib/time.cpp.obj.requires

.PHONY : CMakeFiles/sensors.dir/requires

CMakeFiles/sensors.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensors.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensors.dir/clean

CMakeFiles/sensors.dir/depend:
	cd /home/aldo/catkin_ws/build/trucky_arduino/firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aldo/catkin_ws/src/trucky_arduino/firmware /home/aldo/catkin_ws/src/trucky_arduino/firmware /home/aldo/catkin_ws/build/trucky_arduino/firmware /home/aldo/catkin_ws/build/trucky_arduino/firmware /home/aldo/catkin_ws/build/trucky_arduino/firmware/CMakeFiles/sensors.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensors.dir/depend

