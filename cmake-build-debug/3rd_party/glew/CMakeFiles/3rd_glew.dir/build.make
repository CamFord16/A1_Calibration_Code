# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug"

# Include any dependencies generated for this target.
include 3rd_party/glew/CMakeFiles/3rd_glew.dir/depend.make

# Include the progress variables for this target.
include 3rd_party/glew/CMakeFiles/3rd_glew.dir/progress.make

# Include the compile flags for this target's objects.
include 3rd_party/glew/CMakeFiles/3rd_glew.dir/flags.make

3rd_party/glew/CMakeFiles/3rd_glew.dir/src/glew.c.o: 3rd_party/glew/CMakeFiles/3rd_glew.dir/flags.make
3rd_party/glew/CMakeFiles/3rd_glew.dir/src/glew.c.o: ../3rd_party/glew/src/glew.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object 3rd_party/glew/CMakeFiles/3rd_glew.dir/src/glew.c.o"
	cd "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew" && /Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/3rd_glew.dir/src/glew.c.o   -c "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/3rd_party/glew/src/glew.c"

3rd_party/glew/CMakeFiles/3rd_glew.dir/src/glew.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/3rd_glew.dir/src/glew.c.i"
	cd "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew" && /Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/3rd_party/glew/src/glew.c" > CMakeFiles/3rd_glew.dir/src/glew.c.i

3rd_party/glew/CMakeFiles/3rd_glew.dir/src/glew.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/3rd_glew.dir/src/glew.c.s"
	cd "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew" && /Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/3rd_party/glew/src/glew.c" -o CMakeFiles/3rd_glew.dir/src/glew.c.s

# Object files for target 3rd_glew
3rd_glew_OBJECTS = \
"CMakeFiles/3rd_glew.dir/src/glew.c.o"

# External object files for target 3rd_glew
3rd_glew_EXTERNAL_OBJECTS =

lib/lib3rd_glew.a: 3rd_party/glew/CMakeFiles/3rd_glew.dir/src/glew.c.o
lib/lib3rd_glew.a: 3rd_party/glew/CMakeFiles/3rd_glew.dir/build.make
lib/lib3rd_glew.a: 3rd_party/glew/CMakeFiles/3rd_glew.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library ../../lib/lib3rd_glew.a"
	cd "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew" && $(CMAKE_COMMAND) -P CMakeFiles/3rd_glew.dir/cmake_clean_target.cmake
	cd "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/3rd_glew.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
3rd_party/glew/CMakeFiles/3rd_glew.dir/build: lib/lib3rd_glew.a

.PHONY : 3rd_party/glew/CMakeFiles/3rd_glew.dir/build

3rd_party/glew/CMakeFiles/3rd_glew.dir/clean:
	cd "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew" && $(CMAKE_COMMAND) -P CMakeFiles/3rd_glew.dir/cmake_clean.cmake
.PHONY : 3rd_party/glew/CMakeFiles/3rd_glew.dir/clean

3rd_party/glew/CMakeFiles/3rd_glew.dir/depend:
	cd "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code" "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/3rd_party/glew" "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug" "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew" "/Users/cameraford/Documents/NL LIFE/TU Delft/Year 2/Q3/3D Vision/A1/A1_Calibration/A1_Calibration_Code/cmake-build-debug/3rd_party/glew/CMakeFiles/3rd_glew.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : 3rd_party/glew/CMakeFiles/3rd_glew.dir/depend

