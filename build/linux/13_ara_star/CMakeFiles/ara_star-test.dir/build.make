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
CMAKE_SOURCE_DIR = "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/13_ara_star"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star"

# Include any dependencies generated for this target.
include CMakeFiles/ara_star-test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ara_star-test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ara_star-test.dir/flags.make

CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o: CMakeFiles/ara_star-test.dir/flags.make
CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o: /home/aakash/Desktop/Bonn/Humanoid\ Robotics/group-25/src/13_ara_star/test/test_ara_star.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o"
	/home/aakash/anaconda3/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o -c "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/13_ara_star/test/test_ara_star.cpp"

CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.i"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/13_ara_star/test/test_ara_star.cpp" > CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.i

CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.s"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/13_ara_star/test/test_ara_star.cpp" -o CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.s

CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.requires:

.PHONY : CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.requires

CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.provides: CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.requires
	$(MAKE) -f CMakeFiles/ara_star-test.dir/build.make CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.provides.build
.PHONY : CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.provides

CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.provides.build: CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o


CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o: CMakeFiles/ara_star-test.dir/flags.make
CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o: /home/aakash/Desktop/Bonn/Humanoid\ Robotics/group-25/src/gtest/src/gtest-all.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o"
	/home/aakash/anaconda3/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o -c "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/gtest/src/gtest-all.cc"

CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.i"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/gtest/src/gtest-all.cc" > CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.i

CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.s"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/gtest/src/gtest-all.cc" -o CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.s

CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires:

.PHONY : CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires

CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides: CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires
	$(MAKE) -f CMakeFiles/ara_star-test.dir/build.make CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides.build
.PHONY : CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides

CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.provides.build: CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o


# Object files for target ara_star-test
ara_star__test_OBJECTS = \
"CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o" \
"CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o"

# External object files for target ara_star-test
ara_star__test_EXTERNAL_OBJECTS =

ara_star-test: CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o
ara_star-test: CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o
ara_star-test: CMakeFiles/ara_star-test.dir/build.make
ara_star-test: libara_star.a
ara_star-test: CMakeFiles/ara_star-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ara_star-test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ara_star-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ara_star-test.dir/build: ara_star-test

.PHONY : CMakeFiles/ara_star-test.dir/build

CMakeFiles/ara_star-test.dir/requires: CMakeFiles/ara_star-test.dir/test/test_ara_star.cpp.o.requires
CMakeFiles/ara_star-test.dir/requires: CMakeFiles/ara_star-test.dir/home/aakash/Desktop/Bonn/Humanoid_Robotics/group-25/src/gtest/src/gtest-all.cc.o.requires

.PHONY : CMakeFiles/ara_star-test.dir/requires

CMakeFiles/ara_star-test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ara_star-test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ara_star-test.dir/clean

CMakeFiles/ara_star-test.dir/depend:
	cd "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/13_ara_star" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/13_ara_star" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/13_ara_star/CMakeFiles/ara_star-test.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/ara_star-test.dir/depend

