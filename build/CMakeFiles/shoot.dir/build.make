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
CMAKE_SOURCE_DIR = /home/pxx/Documents/code/gimbal_shoot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pxx/Documents/code/gimbal_shoot/build

# Include any dependencies generated for this target.
include CMakeFiles/shoot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/shoot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/shoot.dir/flags.make

CMakeFiles/shoot.dir/src/shoot.cpp.o: CMakeFiles/shoot.dir/flags.make
CMakeFiles/shoot.dir/src/shoot.cpp.o: /home/pxx/Documents/code/gimbal_shoot/src/src/shoot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pxx/Documents/code/gimbal_shoot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/shoot.dir/src/shoot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shoot.dir/src/shoot.cpp.o -c /home/pxx/Documents/code/gimbal_shoot/src/src/shoot.cpp

CMakeFiles/shoot.dir/src/shoot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shoot.dir/src/shoot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pxx/Documents/code/gimbal_shoot/src/src/shoot.cpp > CMakeFiles/shoot.dir/src/shoot.cpp.i

CMakeFiles/shoot.dir/src/shoot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shoot.dir/src/shoot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pxx/Documents/code/gimbal_shoot/src/src/shoot.cpp -o CMakeFiles/shoot.dir/src/shoot.cpp.s

# Object files for target shoot
shoot_OBJECTS = \
"CMakeFiles/shoot.dir/src/shoot.cpp.o"

# External object files for target shoot
shoot_EXTERNAL_OBJECTS =

shoot: CMakeFiles/shoot.dir/src/shoot.cpp.o
shoot: CMakeFiles/shoot.dir/build.make
shoot: /usr/local/lib/libopencv_dnn.so.3.4.14
shoot: /usr/local/lib/libopencv_highgui.so.3.4.14
shoot: /usr/local/lib/libopencv_ml.so.3.4.14
shoot: /usr/local/lib/libopencv_objdetect.so.3.4.14
shoot: /usr/local/lib/libopencv_shape.so.3.4.14
shoot: /usr/local/lib/libopencv_stitching.so.3.4.14
shoot: /usr/local/lib/libopencv_superres.so.3.4.14
shoot: /usr/local/lib/libopencv_videostab.so.3.4.14
shoot: /usr/local/lib/libopencv_viz.so.3.4.14
shoot: /usr/local/lib/libopencv_calib3d.so.3.4.14
shoot: /usr/local/lib/libopencv_features2d.so.3.4.14
shoot: /usr/local/lib/libopencv_flann.so.3.4.14
shoot: /usr/local/lib/libopencv_photo.so.3.4.14
shoot: /usr/local/lib/libopencv_video.so.3.4.14
shoot: /usr/local/lib/libopencv_videoio.so.3.4.14
shoot: /usr/local/lib/libopencv_imgcodecs.so.3.4.14
shoot: /usr/local/lib/libopencv_imgproc.so.3.4.14
shoot: /usr/local/lib/libopencv_core.so.3.4.14
shoot: CMakeFiles/shoot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pxx/Documents/code/gimbal_shoot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable shoot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/shoot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/shoot.dir/build: shoot

.PHONY : CMakeFiles/shoot.dir/build

CMakeFiles/shoot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/shoot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/shoot.dir/clean

CMakeFiles/shoot.dir/depend:
	cd /home/pxx/Documents/code/gimbal_shoot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pxx/Documents/code/gimbal_shoot/src /home/pxx/Documents/code/gimbal_shoot/src /home/pxx/Documents/code/gimbal_shoot/build /home/pxx/Documents/code/gimbal_shoot/build /home/pxx/Documents/code/gimbal_shoot/build/CMakeFiles/shoot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/shoot.dir/depend

