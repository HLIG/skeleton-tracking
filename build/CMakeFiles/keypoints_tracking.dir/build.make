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
CMAKE_SOURCE_DIR = /home/hlg/cpp_code/keypoints_tracking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hlg/cpp_code/keypoints_tracking/build

# Include any dependencies generated for this target.
include CMakeFiles/keypoints_tracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/keypoints_tracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/keypoints_tracking.dir/flags.make

CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o: CMakeFiles/keypoints_tracking.dir/flags.make
CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o: ../keypoints_tracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hlg/cpp_code/keypoints_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o -c /home/hlg/cpp_code/keypoints_tracking/keypoints_tracking.cpp

CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hlg/cpp_code/keypoints_tracking/keypoints_tracking.cpp > CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.i

CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hlg/cpp_code/keypoints_tracking/keypoints_tracking.cpp -o CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.s

CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.requires:

.PHONY : CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.requires

CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.provides: CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.requires
	$(MAKE) -f CMakeFiles/keypoints_tracking.dir/build.make CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.provides.build
.PHONY : CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.provides

CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.provides.build: CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o


CMakeFiles/keypoints_tracking.dir/main.cpp.o: CMakeFiles/keypoints_tracking.dir/flags.make
CMakeFiles/keypoints_tracking.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hlg/cpp_code/keypoints_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/keypoints_tracking.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keypoints_tracking.dir/main.cpp.o -c /home/hlg/cpp_code/keypoints_tracking/main.cpp

CMakeFiles/keypoints_tracking.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keypoints_tracking.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hlg/cpp_code/keypoints_tracking/main.cpp > CMakeFiles/keypoints_tracking.dir/main.cpp.i

CMakeFiles/keypoints_tracking.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keypoints_tracking.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hlg/cpp_code/keypoints_tracking/main.cpp -o CMakeFiles/keypoints_tracking.dir/main.cpp.s

CMakeFiles/keypoints_tracking.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/keypoints_tracking.dir/main.cpp.o.requires

CMakeFiles/keypoints_tracking.dir/main.cpp.o.provides: CMakeFiles/keypoints_tracking.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/keypoints_tracking.dir/build.make CMakeFiles/keypoints_tracking.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/keypoints_tracking.dir/main.cpp.o.provides

CMakeFiles/keypoints_tracking.dir/main.cpp.o.provides.build: CMakeFiles/keypoints_tracking.dir/main.cpp.o


# Object files for target keypoints_tracking
keypoints_tracking_OBJECTS = \
"CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o" \
"CMakeFiles/keypoints_tracking.dir/main.cpp.o"

# External object files for target keypoints_tracking
keypoints_tracking_EXTERNAL_OBJECTS =

keypoints_tracking: CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o
keypoints_tracking: CMakeFiles/keypoints_tracking.dir/main.cpp.o
keypoints_tracking: CMakeFiles/keypoints_tracking.dir/build.make
keypoints_tracking: munkres/libmunkres.a
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_dnn.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_highgui.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_stitching.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_photo.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_video.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_gapi.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_ml.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_videoio.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_imgcodecs.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_objdetect.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_calib3d.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_features2d.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_imgproc.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_flann.so.4.2.0
keypoints_tracking: /usr/local/opencv4.2/lib/libopencv_core.so.4.2.0
keypoints_tracking: CMakeFiles/keypoints_tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hlg/cpp_code/keypoints_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable keypoints_tracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keypoints_tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/keypoints_tracking.dir/build: keypoints_tracking

.PHONY : CMakeFiles/keypoints_tracking.dir/build

CMakeFiles/keypoints_tracking.dir/requires: CMakeFiles/keypoints_tracking.dir/keypoints_tracking.cpp.o.requires
CMakeFiles/keypoints_tracking.dir/requires: CMakeFiles/keypoints_tracking.dir/main.cpp.o.requires

.PHONY : CMakeFiles/keypoints_tracking.dir/requires

CMakeFiles/keypoints_tracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/keypoints_tracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/keypoints_tracking.dir/clean

CMakeFiles/keypoints_tracking.dir/depend:
	cd /home/hlg/cpp_code/keypoints_tracking/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hlg/cpp_code/keypoints_tracking /home/hlg/cpp_code/keypoints_tracking /home/hlg/cpp_code/keypoints_tracking/build /home/hlg/cpp_code/keypoints_tracking/build /home/hlg/cpp_code/keypoints_tracking/build/CMakeFiles/keypoints_tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/keypoints_tracking.dir/depend

