# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/sdb2/Progetti/Extruder/proj-8.1.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build

# Include any dependencies generated for this target.
include test/unit/CMakeFiles/proj_angular_io_test.dir/depend.make

# Include the progress variables for this target.
include test/unit/CMakeFiles/proj_angular_io_test.dir/progress.make

# Include the compile flags for this target's objects.
include test/unit/CMakeFiles/proj_angular_io_test.dir/flags.make

test/unit/CMakeFiles/proj_angular_io_test.dir/main.cpp.o: test/unit/CMakeFiles/proj_angular_io_test.dir/flags.make
test/unit/CMakeFiles/proj_angular_io_test.dir/main.cpp.o: ../test/unit/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/unit/CMakeFiles/proj_angular_io_test.dir/main.cpp.o"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/proj_angular_io_test.dir/main.cpp.o -c /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/main.cpp

test/unit/CMakeFiles/proj_angular_io_test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/proj_angular_io_test.dir/main.cpp.i"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/main.cpp > CMakeFiles/proj_angular_io_test.dir/main.cpp.i

test/unit/CMakeFiles/proj_angular_io_test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/proj_angular_io_test.dir/main.cpp.s"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/main.cpp -o CMakeFiles/proj_angular_io_test.dir/main.cpp.s

test/unit/CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.o: test/unit/CMakeFiles/proj_angular_io_test.dir/flags.make
test/unit/CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.o: ../test/unit/proj_angular_io_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/unit/CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.o"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.o -c /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/proj_angular_io_test.cpp

test/unit/CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.i"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/proj_angular_io_test.cpp > CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.i

test/unit/CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.s"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/proj_angular_io_test.cpp -o CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.s

# Object files for target proj_angular_io_test
proj_angular_io_test_OBJECTS = \
"CMakeFiles/proj_angular_io_test.dir/main.cpp.o" \
"CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.o"

# External object files for target proj_angular_io_test
proj_angular_io_test_EXTERNAL_OBJECTS =

bin/proj_angular_io_test: test/unit/CMakeFiles/proj_angular_io_test.dir/main.cpp.o
bin/proj_angular_io_test: test/unit/CMakeFiles/proj_angular_io_test.dir/proj_angular_io_test.cpp.o
bin/proj_angular_io_test: test/unit/CMakeFiles/proj_angular_io_test.dir/build.make
bin/proj_angular_io_test: lib/libgtest.so
bin/proj_angular_io_test: lib/libproj.so.22.1.1
bin/proj_angular_io_test: test/unit/CMakeFiles/proj_angular_io_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/proj_angular_io_test"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/proj_angular_io_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/unit/CMakeFiles/proj_angular_io_test.dir/build: bin/proj_angular_io_test

.PHONY : test/unit/CMakeFiles/proj_angular_io_test.dir/build

test/unit/CMakeFiles/proj_angular_io_test.dir/clean:
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && $(CMAKE_COMMAND) -P CMakeFiles/proj_angular_io_test.dir/cmake_clean.cmake
.PHONY : test/unit/CMakeFiles/proj_angular_io_test.dir/clean

test/unit/CMakeFiles/proj_angular_io_test.dir/depend:
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/sdb2/Progetti/Extruder/proj-8.1.1 /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit/CMakeFiles/proj_angular_io_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/unit/CMakeFiles/proj_angular_io_test.dir/depend

