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
include test/unit/CMakeFiles/test_misc.dir/depend.make

# Include the progress variables for this target.
include test/unit/CMakeFiles/test_misc.dir/progress.make

# Include the compile flags for this target's objects.
include test/unit/CMakeFiles/test_misc.dir/flags.make

test/unit/CMakeFiles/test_misc.dir/main.cpp.o: test/unit/CMakeFiles/test_misc.dir/flags.make
test/unit/CMakeFiles/test_misc.dir/main.cpp.o: ../test/unit/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/unit/CMakeFiles/test_misc.dir/main.cpp.o"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_misc.dir/main.cpp.o -c /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/main.cpp

test/unit/CMakeFiles/test_misc.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_misc.dir/main.cpp.i"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/main.cpp > CMakeFiles/test_misc.dir/main.cpp.i

test/unit/CMakeFiles/test_misc.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_misc.dir/main.cpp.s"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/main.cpp -o CMakeFiles/test_misc.dir/main.cpp.s

test/unit/CMakeFiles/test_misc.dir/test_misc.cpp.o: test/unit/CMakeFiles/test_misc.dir/flags.make
test/unit/CMakeFiles/test_misc.dir/test_misc.cpp.o: ../test/unit/test_misc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/unit/CMakeFiles/test_misc.dir/test_misc.cpp.o"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_misc.dir/test_misc.cpp.o -c /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/test_misc.cpp

test/unit/CMakeFiles/test_misc.dir/test_misc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_misc.dir/test_misc.cpp.i"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/test_misc.cpp > CMakeFiles/test_misc.dir/test_misc.cpp.i

test/unit/CMakeFiles/test_misc.dir/test_misc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_misc.dir/test_misc.cpp.s"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit/test_misc.cpp -o CMakeFiles/test_misc.dir/test_misc.cpp.s

# Object files for target test_misc
test_misc_OBJECTS = \
"CMakeFiles/test_misc.dir/main.cpp.o" \
"CMakeFiles/test_misc.dir/test_misc.cpp.o"

# External object files for target test_misc
test_misc_EXTERNAL_OBJECTS =

bin/test_misc: test/unit/CMakeFiles/test_misc.dir/main.cpp.o
bin/test_misc: test/unit/CMakeFiles/test_misc.dir/test_misc.cpp.o
bin/test_misc: test/unit/CMakeFiles/test_misc.dir/build.make
bin/test_misc: lib/libgtest.so
bin/test_misc: lib/libproj.so.22.1.1
bin/test_misc: test/unit/CMakeFiles/test_misc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/test_misc"
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_misc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/unit/CMakeFiles/test_misc.dir/build: bin/test_misc

.PHONY : test/unit/CMakeFiles/test_misc.dir/build

test/unit/CMakeFiles/test_misc.dir/clean:
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit && $(CMAKE_COMMAND) -P CMakeFiles/test_misc.dir/cmake_clean.cmake
.PHONY : test/unit/CMakeFiles/test_misc.dir/clean

test/unit/CMakeFiles/test_misc.dir/depend:
	cd /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/sdb2/Progetti/Extruder/proj-8.1.1 /mnt/sdb2/Progetti/Extruder/proj-8.1.1/test/unit /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit /mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/test/unit/CMakeFiles/test_misc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/unit/CMakeFiles/test_misc.dir/depend

