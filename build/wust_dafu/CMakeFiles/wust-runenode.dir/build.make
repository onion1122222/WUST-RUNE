# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zhang/dafu_final/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhang/dafu_final/build

# Include any dependencies generated for this target.
include wust_dafu/CMakeFiles/wust-runenode.dir/depend.make

# Include the progress variables for this target.
include wust_dafu/CMakeFiles/wust-runenode.dir/progress.make

# Include the compile flags for this target's objects.
include wust_dafu/CMakeFiles/wust-runenode.dir/flags.make

wust_dafu/CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.o: wust_dafu/CMakeFiles/wust-runenode.dir/flags.make
wust_dafu/CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.o: /home/zhang/dafu_final/src/wust_dafu/src/wust-runenode.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhang/dafu_final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object wust_dafu/CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.o"
	cd /home/zhang/dafu_final/build/wust_dafu && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.o -c /home/zhang/dafu_final/src/wust_dafu/src/wust-runenode.cpp

wust_dafu/CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.i"
	cd /home/zhang/dafu_final/build/wust_dafu && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhang/dafu_final/src/wust_dafu/src/wust-runenode.cpp > CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.i

wust_dafu/CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.s"
	cd /home/zhang/dafu_final/build/wust_dafu && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhang/dafu_final/src/wust_dafu/src/wust-runenode.cpp -o CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.s

# Object files for target wust-runenode
wust__runenode_OBJECTS = \
"CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.o"

# External object files for target wust-runenode
wust__runenode_EXTERNAL_OBJECTS =

/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: wust_dafu/CMakeFiles/wust-runenode.dir/src/wust-runenode.cpp.o
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: wust_dafu/CMakeFiles/wust-runenode.dir/build.make
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/libroscpp.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/librosconsole.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/librostime.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /opt/ros/noetic/lib/libcpp_common.so
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_highgui.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_ml.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_objdetect.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_photo.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_stitching.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_video.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_videoio.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_calib3d.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_dnn.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_features2d.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_flann.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_imgproc.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: /usr/local/lib/libopencv_core.so.4.8.0
/home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode: wust_dafu/CMakeFiles/wust-runenode.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhang/dafu_final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode"
	cd /home/zhang/dafu_final/build/wust_dafu && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/wust-runenode.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
wust_dafu/CMakeFiles/wust-runenode.dir/build: /home/zhang/dafu_final/devel/lib/wust_dafu/wust-runenode

.PHONY : wust_dafu/CMakeFiles/wust-runenode.dir/build

wust_dafu/CMakeFiles/wust-runenode.dir/clean:
	cd /home/zhang/dafu_final/build/wust_dafu && $(CMAKE_COMMAND) -P CMakeFiles/wust-runenode.dir/cmake_clean.cmake
.PHONY : wust_dafu/CMakeFiles/wust-runenode.dir/clean

wust_dafu/CMakeFiles/wust-runenode.dir/depend:
	cd /home/zhang/dafu_final/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhang/dafu_final/src /home/zhang/dafu_final/src/wust_dafu /home/zhang/dafu_final/build /home/zhang/dafu_final/build/wust_dafu /home/zhang/dafu_final/build/wust_dafu/CMakeFiles/wust-runenode.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : wust_dafu/CMakeFiles/wust-runenode.dir/depend

