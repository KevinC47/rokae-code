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
CMAKE_SOURCE_DIR = /home/kevin/rci

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/rci/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/track_move_test.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/track_move_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/track_move_test.dir/flags.make

examples/CMakeFiles/track_move_test.dir/track_move_test.cpp.o: examples/CMakeFiles/track_move_test.dir/flags.make
examples/CMakeFiles/track_move_test.dir/track_move_test.cpp.o: ../examples/track_move_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/rci/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/track_move_test.dir/track_move_test.cpp.o"
	cd /home/kevin/rci/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/track_move_test.dir/track_move_test.cpp.o -c /home/kevin/rci/examples/track_move_test.cpp

examples/CMakeFiles/track_move_test.dir/track_move_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_move_test.dir/track_move_test.cpp.i"
	cd /home/kevin/rci/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/rci/examples/track_move_test.cpp > CMakeFiles/track_move_test.dir/track_move_test.cpp.i

examples/CMakeFiles/track_move_test.dir/track_move_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_move_test.dir/track_move_test.cpp.s"
	cd /home/kevin/rci/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/rci/examples/track_move_test.cpp -o CMakeFiles/track_move_test.dir/track_move_test.cpp.s

# Object files for target track_move_test
track_move_test_OBJECTS = \
"CMakeFiles/track_move_test.dir/track_move_test.cpp.o"

# External object files for target track_move_test
track_move_test_EXTERNAL_OBJECTS =

examples/track_move_test: examples/CMakeFiles/track_move_test.dir/track_move_test.cpp.o
examples/track_move_test: examples/CMakeFiles/track_move_test.dir/build.make
examples/track_move_test: examples/CMakeFiles/track_move_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/rci/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable track_move_test"
	cd /home/kevin/rci/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/track_move_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/track_move_test.dir/build: examples/track_move_test

.PHONY : examples/CMakeFiles/track_move_test.dir/build

examples/CMakeFiles/track_move_test.dir/clean:
	cd /home/kevin/rci/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/track_move_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/track_move_test.dir/clean

examples/CMakeFiles/track_move_test.dir/depend:
	cd /home/kevin/rci/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/rci /home/kevin/rci/examples /home/kevin/rci/build /home/kevin/rci/build/examples /home/kevin/rci/build/examples/CMakeFiles/track_move_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/track_move_test.dir/depend
