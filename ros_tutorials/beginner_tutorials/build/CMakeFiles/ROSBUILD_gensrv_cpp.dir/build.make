# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jamest/ros_tutorials/beginner_tutorials

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jamest/ros_tutorials/beginner_tutorials/build

# Utility rule file for ROSBUILD_gensrv_cpp.

CMakeFiles/ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h

../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: ../srv/AddTwoInts.srv
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/scripts/gensrv_cpp.py
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/ros/core/roslib/scripts/gendeps
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: ../manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/ros/tools/rospack/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/ros/core/roslib/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/messages/std_msgs/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/messages/rosgraph_msgs/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/ros/core/rosbuild/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/ros/core/roslang/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/rospy/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_traits/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/utilities/rostime/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/manifest.xml
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/messages/std_msgs/msg_gen/generated
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/generated
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/msg_gen/generated
../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h: /opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jamest/ros_tutorials/beginner_tutorials/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h"
	/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/scripts/gensrv_cpp.py /home/jamest/ros_tutorials/beginner_tutorials/srv/AddTwoInts.srv

ROSBUILD_gensrv_cpp: CMakeFiles/ROSBUILD_gensrv_cpp
ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/beginner_tutorials/AddTwoInts.h
ROSBUILD_gensrv_cpp: CMakeFiles/ROSBUILD_gensrv_cpp.dir/build.make
.PHONY : ROSBUILD_gensrv_cpp

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_gensrv_cpp.dir/build: ROSBUILD_gensrv_cpp
.PHONY : CMakeFiles/ROSBUILD_gensrv_cpp.dir/build

CMakeFiles/ROSBUILD_gensrv_cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_gensrv_cpp.dir/clean

CMakeFiles/ROSBUILD_gensrv_cpp.dir/depend:
	cd /home/jamest/ros_tutorials/beginner_tutorials/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jamest/ros_tutorials/beginner_tutorials /home/jamest/ros_tutorials/beginner_tutorials /home/jamest/ros_tutorials/beginner_tutorials/build /home/jamest/ros_tutorials/beginner_tutorials/build /home/jamest/ros_tutorials/beginner_tutorials/build/CMakeFiles/ROSBUILD_gensrv_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_gensrv_cpp.dir/depend

