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
include examples/CMakeFiles/joint_s_mu.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/joint_s_mu.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/joint_s_mu.dir/flags.make

examples/CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.o: examples/CMakeFiles/joint_s_mu.dir/flags.make
examples/CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.o: ../examples/joint_s_mu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/rci/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.o"
	cd /home/kevin/rci/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.o -c /home/kevin/rci/examples/joint_s_mu.cpp

examples/CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.i"
	cd /home/kevin/rci/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/rci/examples/joint_s_mu.cpp > CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.i

examples/CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.s"
	cd /home/kevin/rci/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/rci/examples/joint_s_mu.cpp -o CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.s

# Object files for target joint_s_mu
joint_s_mu_OBJECTS = \
"CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.o"

# External object files for target joint_s_mu
joint_s_mu_EXTERNAL_OBJECTS =

examples/joint_s_mu: examples/CMakeFiles/joint_s_mu.dir/joint_s_mu.cpp.o
examples/joint_s_mu: examples/CMakeFiles/joint_s_mu.dir/build.make
examples/joint_s_mu: examples/CMakeFiles/joint_s_mu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/rci/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable joint_s_mu"
	cd /home/kevin/rci/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joint_s_mu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/joint_s_mu.dir/build: examples/joint_s_mu

.PHONY : examples/CMakeFiles/joint_s_mu.dir/build

examples/CMakeFiles/joint_s_mu.dir/clean:
	cd /home/kevin/rci/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/joint_s_mu.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/joint_s_mu.dir/clean

examples/CMakeFiles/joint_s_mu.dir/depend:
	cd /home/kevin/rci/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/rci /home/kevin/rci/examples /home/kevin/rci/build /home/kevin/rci/build/examples /home/kevin/rci/build/examples/CMakeFiles/joint_s_mu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/joint_s_mu.dir/depend

