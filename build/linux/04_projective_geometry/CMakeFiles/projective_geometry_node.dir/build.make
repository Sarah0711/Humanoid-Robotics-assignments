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
CMAKE_SOURCE_DIR = "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/04_projective_geometry"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/04_projective_geometry"

# Include any dependencies generated for this target.
include CMakeFiles/projective_geometry_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/projective_geometry_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/projective_geometry_node.dir/flags.make

CMakeFiles/projective_geometry_node.dir/src/main.cpp.o: CMakeFiles/projective_geometry_node.dir/flags.make
CMakeFiles/projective_geometry_node.dir/src/main.cpp.o: /home/aakash/Desktop/Bonn/Humanoid\ Robotics/group-25/src/04_projective_geometry/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/04_projective_geometry/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/projective_geometry_node.dir/src/main.cpp.o"
	/home/aakash/anaconda3/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projective_geometry_node.dir/src/main.cpp.o -c "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/04_projective_geometry/src/main.cpp"

CMakeFiles/projective_geometry_node.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projective_geometry_node.dir/src/main.cpp.i"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/04_projective_geometry/src/main.cpp" > CMakeFiles/projective_geometry_node.dir/src/main.cpp.i

CMakeFiles/projective_geometry_node.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projective_geometry_node.dir/src/main.cpp.s"
	/home/aakash/anaconda3/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/04_projective_geometry/src/main.cpp" -o CMakeFiles/projective_geometry_node.dir/src/main.cpp.s

CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.requires

CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.provides: CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/projective_geometry_node.dir/build.make CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.provides

CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.provides.build: CMakeFiles/projective_geometry_node.dir/src/main.cpp.o


# Object files for target projective_geometry_node
projective_geometry_node_OBJECTS = \
"CMakeFiles/projective_geometry_node.dir/src/main.cpp.o"

# External object files for target projective_geometry_node
projective_geometry_node_EXTERNAL_OBJECTS =

projective_geometry_node: CMakeFiles/projective_geometry_node.dir/src/main.cpp.o
projective_geometry_node: CMakeFiles/projective_geometry_node.dir/build.make
projective_geometry_node: libprojective_geometry.a
projective_geometry_node: CMakeFiles/projective_geometry_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/04_projective_geometry/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable projective_geometry_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/projective_geometry_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/projective_geometry_node.dir/build: projective_geometry_node

.PHONY : CMakeFiles/projective_geometry_node.dir/build

CMakeFiles/projective_geometry_node.dir/requires: CMakeFiles/projective_geometry_node.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/projective_geometry_node.dir/requires

CMakeFiles/projective_geometry_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/projective_geometry_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/projective_geometry_node.dir/clean

CMakeFiles/projective_geometry_node.dir/depend:
	cd "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/04_projective_geometry" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/04_projective_geometry" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/src/04_projective_geometry" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/04_projective_geometry" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/04_projective_geometry" "/home/aakash/Desktop/Bonn/Humanoid Robotics/group-25/build/linux/04_projective_geometry/CMakeFiles/projective_geometry_node.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/projective_geometry_node.dir/depend

