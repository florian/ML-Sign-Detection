# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dorian/traffic_sign_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dorian/traffic_sign_ws/build

# Include any dependencies generated for this target.
include detection/CMakeFiles/detection.dir/depend.make

# Include the progress variables for this target.
include detection/CMakeFiles/detection.dir/progress.make

# Include the compile flags for this target's objects.
include detection/CMakeFiles/detection.dir/flags.make

detection/CMakeFiles/detection.dir/src/detection.cpp.o: detection/CMakeFiles/detection.dir/flags.make
detection/CMakeFiles/detection.dir/src/detection.cpp.o: /home/dorian/traffic_sign_ws/src/detection/src/detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dorian/traffic_sign_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object detection/CMakeFiles/detection.dir/src/detection.cpp.o"
	cd /home/dorian/traffic_sign_ws/build/detection && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/detection.dir/src/detection.cpp.o -c /home/dorian/traffic_sign_ws/src/detection/src/detection.cpp

detection/CMakeFiles/detection.dir/src/detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detection.dir/src/detection.cpp.i"
	cd /home/dorian/traffic_sign_ws/build/detection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/dorian/traffic_sign_ws/src/detection/src/detection.cpp > CMakeFiles/detection.dir/src/detection.cpp.i

detection/CMakeFiles/detection.dir/src/detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detection.dir/src/detection.cpp.s"
	cd /home/dorian/traffic_sign_ws/build/detection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/dorian/traffic_sign_ws/src/detection/src/detection.cpp -o CMakeFiles/detection.dir/src/detection.cpp.s

detection/CMakeFiles/detection.dir/src/detection.cpp.o.requires:
.PHONY : detection/CMakeFiles/detection.dir/src/detection.cpp.o.requires

detection/CMakeFiles/detection.dir/src/detection.cpp.o.provides: detection/CMakeFiles/detection.dir/src/detection.cpp.o.requires
	$(MAKE) -f detection/CMakeFiles/detection.dir/build.make detection/CMakeFiles/detection.dir/src/detection.cpp.o.provides.build
.PHONY : detection/CMakeFiles/detection.dir/src/detection.cpp.o.provides

detection/CMakeFiles/detection.dir/src/detection.cpp.o.provides.build: detection/CMakeFiles/detection.dir/src/detection.cpp.o

# Object files for target detection
detection_OBJECTS = \
"CMakeFiles/detection.dir/src/detection.cpp.o"

# External object files for target detection
detection_EXTERNAL_OBJECTS =

/home/dorian/traffic_sign_ws/devel/lib/detection/detection: detection/CMakeFiles/detection.dir/src/detection.cpp.o
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: detection/CMakeFiles/detection.dir/build.make
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /home/dorian/traffic_sign_ws/devel/lib/libdlib.a
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libcv_bridge.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libimage_transport.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libmessage_filters.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libclass_loader.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/libPocoFoundation.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libdl.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libroscpp.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/librosconsole.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/liblog4cxx.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libroslib.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/librospack.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/librostime.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /opt/ros/indigo/lib/libcpp_common.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libnsl.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libSM.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libICE.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libX11.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libXext.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libpng.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libjpeg.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/libcblas.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/liblapack.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: /usr/lib/x86_64-linux-gnu/libsqlite3.so
/home/dorian/traffic_sign_ws/devel/lib/detection/detection: detection/CMakeFiles/detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/dorian/traffic_sign_ws/devel/lib/detection/detection"
	cd /home/dorian/traffic_sign_ws/build/detection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
detection/CMakeFiles/detection.dir/build: /home/dorian/traffic_sign_ws/devel/lib/detection/detection
.PHONY : detection/CMakeFiles/detection.dir/build

detection/CMakeFiles/detection.dir/requires: detection/CMakeFiles/detection.dir/src/detection.cpp.o.requires
.PHONY : detection/CMakeFiles/detection.dir/requires

detection/CMakeFiles/detection.dir/clean:
	cd /home/dorian/traffic_sign_ws/build/detection && $(CMAKE_COMMAND) -P CMakeFiles/detection.dir/cmake_clean.cmake
.PHONY : detection/CMakeFiles/detection.dir/clean

detection/CMakeFiles/detection.dir/depend:
	cd /home/dorian/traffic_sign_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dorian/traffic_sign_ws/src /home/dorian/traffic_sign_ws/src/detection /home/dorian/traffic_sign_ws/build /home/dorian/traffic_sign_ws/build/detection /home/dorian/traffic_sign_ws/build/detection/CMakeFiles/detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : detection/CMakeFiles/detection.dir/depend

