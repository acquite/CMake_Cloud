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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/work/Documents/eigen_tutorials

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/work/Documents/eigen_tutorials/build

# Include any dependencies generated for this target.
include CMakeFiles/eigrnTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/eigrnTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/eigrnTest.dir/flags.make

CMakeFiles/eigrnTest.dir/main.cpp.o: CMakeFiles/eigrnTest.dir/flags.make
CMakeFiles/eigrnTest.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/work/Documents/eigen_tutorials/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/eigrnTest.dir/main.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/eigrnTest.dir/main.cpp.o -c /home/work/Documents/eigen_tutorials/main.cpp

CMakeFiles/eigrnTest.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/eigrnTest.dir/main.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/work/Documents/eigen_tutorials/main.cpp > CMakeFiles/eigrnTest.dir/main.cpp.i

CMakeFiles/eigrnTest.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/eigrnTest.dir/main.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/work/Documents/eigen_tutorials/main.cpp -o CMakeFiles/eigrnTest.dir/main.cpp.s

CMakeFiles/eigrnTest.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/eigrnTest.dir/main.cpp.o.requires

CMakeFiles/eigrnTest.dir/main.cpp.o.provides: CMakeFiles/eigrnTest.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/eigrnTest.dir/build.make CMakeFiles/eigrnTest.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/eigrnTest.dir/main.cpp.o.provides

CMakeFiles/eigrnTest.dir/main.cpp.o.provides.build: CMakeFiles/eigrnTest.dir/main.cpp.o


# Object files for target eigrnTest
eigrnTest_OBJECTS = \
"CMakeFiles/eigrnTest.dir/main.cpp.o"

# External object files for target eigrnTest
eigrnTest_EXTERNAL_OBJECTS =

eigrnTest: CMakeFiles/eigrnTest.dir/main.cpp.o
eigrnTest: CMakeFiles/eigrnTest.dir/build.make
eigrnTest: CMakeFiles/eigrnTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/work/Documents/eigen_tutorials/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable eigrnTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/eigrnTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/eigrnTest.dir/build: eigrnTest

.PHONY : CMakeFiles/eigrnTest.dir/build

CMakeFiles/eigrnTest.dir/requires: CMakeFiles/eigrnTest.dir/main.cpp.o.requires

.PHONY : CMakeFiles/eigrnTest.dir/requires

CMakeFiles/eigrnTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/eigrnTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/eigrnTest.dir/clean

CMakeFiles/eigrnTest.dir/depend:
	cd /home/work/Documents/eigen_tutorials/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/work/Documents/eigen_tutorials /home/work/Documents/eigen_tutorials /home/work/Documents/eigen_tutorials/build /home/work/Documents/eigen_tutorials/build /home/work/Documents/eigen_tutorials/build/CMakeFiles/eigrnTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/eigrnTest.dir/depend

