# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Brushless_Motor_Controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Brushless_Motor_Controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Brushless_Motor_Controller.dir/flags.make

CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o: CMakeFiles/Brushless_Motor_Controller.dir/flags.make
CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o -c /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/main.cpp

CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/main.cpp > CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.i

CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/main.cpp -o CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.s

CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.requires

CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.provides: CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Brushless_Motor_Controller.dir/build.make CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.provides

CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.provides.build: CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o


# Object files for target Brushless_Motor_Controller
Brushless_Motor_Controller_OBJECTS = \
"CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o"

# External object files for target Brushless_Motor_Controller
Brushless_Motor_Controller_EXTERNAL_OBJECTS =

Brushless_Motor_Controller: CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o
Brushless_Motor_Controller: CMakeFiles/Brushless_Motor_Controller.dir/build.make
Brushless_Motor_Controller: CMakeFiles/Brushless_Motor_Controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Brushless_Motor_Controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Brushless_Motor_Controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Brushless_Motor_Controller.dir/build: Brushless_Motor_Controller

.PHONY : CMakeFiles/Brushless_Motor_Controller.dir/build

CMakeFiles/Brushless_Motor_Controller.dir/requires: CMakeFiles/Brushless_Motor_Controller.dir/main.cpp.o.requires

.PHONY : CMakeFiles/Brushless_Motor_Controller.dir/requires

CMakeFiles/Brushless_Motor_Controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Brushless_Motor_Controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Brushless_Motor_Controller.dir/clean

CMakeFiles/Brushless_Motor_Controller.dir/depend:
	cd /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/cmake-build-debug /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/cmake-build-debug /Users/aktersnurra/Documents/Embedded_Systems/Brushless-Motor-Controller/cmake-build-debug/CMakeFiles/Brushless_Motor_Controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Brushless_Motor_Controller.dir/depend
