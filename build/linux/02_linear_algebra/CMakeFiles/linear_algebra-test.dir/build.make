# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/02_linear_algebra"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra"

# Include any dependencies generated for this target.
include CMakeFiles/linear_algebra-test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/linear_algebra-test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/linear_algebra-test.dir/flags.make

CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o: CMakeFiles/linear_algebra-test.dir/flags.make
CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o: /home/aakash/Desktop/Bonn/Humanoid\ Robotics/group-25/src/02_linear_algebra/test/test_linear_algebra.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o"
	/home/aakash/anaconda3/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o -c "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/02_linear_algebra/test/test_linear_algebra.cpp"

CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.i"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/02_linear_algebra/test/test_linear_algebra.cpp" > CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.i

CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.s"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/02_linear_algebra/test/test_linear_algebra.cpp" -o CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.s

CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.requires:

.PHONY : CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.requires

CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.provides: CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.requires
	$(MAKE) -f CMakeFiles/linear_algebra-test.dir/build.make CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.provides.build
.PHONY : CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.provides

CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.provides.build: CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o


CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o: CMakeFiles/linear_algebra-test.dir/flags.make
CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o: /home/aakash/Desktop/Bonn/Humanoid\ Robotics/group-25/src/gtest/src/gtest-all.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o"
	/home/aakash/anaconda3/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o -c "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/gtest/src/gtest-all.cc"

CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.i"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/gtest/src/gtest-all.cc" > CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.i

CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.s"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/gtest/src/gtest-all.cc" -o CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.s

CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires:

.PHONY : CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires

CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides: CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires
	$(MAKE) -f CMakeFiles/linear_algebra-test.dir/build.make CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides.build
.PHONY : CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides

CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides.build: CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o


# Object files for target linear_algebra-test
linear_algebra__test_OBJECTS = \
"CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o" \
"CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o"

# External object files for target linear_algebra-test
linear_algebra__test_EXTERNAL_OBJECTS =

linear_algebra-test: CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o
linear_algebra-test: CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o
linear_algebra-test: CMakeFiles/linear_algebra-test.dir/build.make
linear_algebra-test: liblinear_algebra.so
linear_algebra-test: CMakeFiles/linear_algebra-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable linear_algebra-test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linear_algebra-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/linear_algebra-test.dir/build: linear_algebra-test

.PHONY : CMakeFiles/linear_algebra-test.dir/build

CMakeFiles/linear_algebra-test.dir/requires: CMakeFiles/linear_algebra-test.dir/test/test_linear_algebra.cpp.o.requires
CMakeFiles/linear_algebra-test.dir/requires: CMakeFiles/linear_algebra-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires

.PHONY : CMakeFiles/linear_algebra-test.dir/requires

CMakeFiles/linear_algebra-test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/linear_algebra-test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/linear_algebra-test.dir/clean

CMakeFiles/linear_algebra-test.dir/depend:
	cd "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/02_linear_algebra" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/02_linear_algebra" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/02_linear_algebra/CMakeFiles/linear_algebra-test.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/linear_algebra-test.dir/depend

