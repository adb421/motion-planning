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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/adam/Dropbox/Manipulation/MotionPlanning/AStar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adam/Dropbox/Manipulation/MotionPlanning/AStar

# Include any dependencies generated for this target.
include CMakeFiles/AStarTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/AStarTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/AStarTest.dir/flags.make

CMakeFiles/AStarTest.dir/AStarNode.cpp.o: CMakeFiles/AStarTest.dir/flags.make
CMakeFiles/AStarTest.dir/AStarNode.cpp.o: AStarNode.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/AStarTest.dir/AStarNode.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AStarTest.dir/AStarNode.cpp.o -c /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/AStarNode.cpp

CMakeFiles/AStarTest.dir/AStarNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AStarTest.dir/AStarNode.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/AStarNode.cpp > CMakeFiles/AStarTest.dir/AStarNode.cpp.i

CMakeFiles/AStarTest.dir/AStarNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AStarTest.dir/AStarNode.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/AStarNode.cpp -o CMakeFiles/AStarTest.dir/AStarNode.cpp.s

CMakeFiles/AStarTest.dir/AStarNode.cpp.o.requires:
.PHONY : CMakeFiles/AStarTest.dir/AStarNode.cpp.o.requires

CMakeFiles/AStarTest.dir/AStarNode.cpp.o.provides: CMakeFiles/AStarTest.dir/AStarNode.cpp.o.requires
	$(MAKE) -f CMakeFiles/AStarTest.dir/build.make CMakeFiles/AStarTest.dir/AStarNode.cpp.o.provides.build
.PHONY : CMakeFiles/AStarTest.dir/AStarNode.cpp.o.provides

CMakeFiles/AStarTest.dir/AStarNode.cpp.o.provides.build: CMakeFiles/AStarTest.dir/AStarNode.cpp.o

CMakeFiles/AStarTest.dir/AStarTest.cpp.o: CMakeFiles/AStarTest.dir/flags.make
CMakeFiles/AStarTest.dir/AStarTest.cpp.o: AStarTest.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/AStarTest.dir/AStarTest.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AStarTest.dir/AStarTest.cpp.o -c /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/AStarTest.cpp

CMakeFiles/AStarTest.dir/AStarTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AStarTest.dir/AStarTest.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/AStarTest.cpp > CMakeFiles/AStarTest.dir/AStarTest.cpp.i

CMakeFiles/AStarTest.dir/AStarTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AStarTest.dir/AStarTest.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/AStarTest.cpp -o CMakeFiles/AStarTest.dir/AStarTest.cpp.s

CMakeFiles/AStarTest.dir/AStarTest.cpp.o.requires:
.PHONY : CMakeFiles/AStarTest.dir/AStarTest.cpp.o.requires

CMakeFiles/AStarTest.dir/AStarTest.cpp.o.provides: CMakeFiles/AStarTest.dir/AStarTest.cpp.o.requires
	$(MAKE) -f CMakeFiles/AStarTest.dir/build.make CMakeFiles/AStarTest.dir/AStarTest.cpp.o.provides.build
.PHONY : CMakeFiles/AStarTest.dir/AStarTest.cpp.o.provides

CMakeFiles/AStarTest.dir/AStarTest.cpp.o.provides.build: CMakeFiles/AStarTest.dir/AStarTest.cpp.o

# Object files for target AStarTest
AStarTest_OBJECTS = \
"CMakeFiles/AStarTest.dir/AStarNode.cpp.o" \
"CMakeFiles/AStarTest.dir/AStarTest.cpp.o"

# External object files for target AStarTest
AStarTest_EXTERNAL_OBJECTS =

AStarTest: CMakeFiles/AStarTest.dir/AStarNode.cpp.o
AStarTest: CMakeFiles/AStarTest.dir/AStarTest.cpp.o
AStarTest: /usr/local/lib/libmpfr.so
AStarTest: /usr/local/lib/libgmp.so
AStarTest: /usr/local/lib/libCGAL.so
AStarTest: /usr/lib/libboost_thread-mt.so
AStarTest: /usr/lib/libboost_system-mt.so
AStarTest: /usr/local/lib/libCGAL.so
AStarTest: /usr/lib/libboost_thread-mt.so
AStarTest: /usr/lib/libboost_system-mt.so
AStarTest: CMakeFiles/AStarTest.dir/build.make
AStarTest: CMakeFiles/AStarTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable AStarTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AStarTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/AStarTest.dir/build: AStarTest
.PHONY : CMakeFiles/AStarTest.dir/build

CMakeFiles/AStarTest.dir/requires: CMakeFiles/AStarTest.dir/AStarNode.cpp.o.requires
CMakeFiles/AStarTest.dir/requires: CMakeFiles/AStarTest.dir/AStarTest.cpp.o.requires
.PHONY : CMakeFiles/AStarTest.dir/requires

CMakeFiles/AStarTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/AStarTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/AStarTest.dir/clean

CMakeFiles/AStarTest.dir/depend:
	cd /home/adam/Dropbox/Manipulation/MotionPlanning/AStar && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adam/Dropbox/Manipulation/MotionPlanning/AStar /home/adam/Dropbox/Manipulation/MotionPlanning/AStar /home/adam/Dropbox/Manipulation/MotionPlanning/AStar /home/adam/Dropbox/Manipulation/MotionPlanning/AStar /home/adam/Dropbox/Manipulation/MotionPlanning/AStar/CMakeFiles/AStarTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/AStarTest.dir/depend

