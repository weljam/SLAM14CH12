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
CMAKE_SOURCE_DIR = /home/cwj/slamebook2/ch12/dense_mono

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cwj/slamebook2/ch12/dense_mono/build

# Include any dependencies generated for this target.
include CMakeFiles/dense_mapping.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dense_mapping.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dense_mapping.dir/flags.make

CMakeFiles/dense_mapping.dir/dense_mapping.cpp.o: CMakeFiles/dense_mapping.dir/flags.make
CMakeFiles/dense_mapping.dir/dense_mapping.cpp.o: ../dense_mapping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cwj/slamebook2/ch12/dense_mono/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dense_mapping.dir/dense_mapping.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dense_mapping.dir/dense_mapping.cpp.o -c /home/cwj/slamebook2/ch12/dense_mono/dense_mapping.cpp

CMakeFiles/dense_mapping.dir/dense_mapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dense_mapping.dir/dense_mapping.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cwj/slamebook2/ch12/dense_mono/dense_mapping.cpp > CMakeFiles/dense_mapping.dir/dense_mapping.cpp.i

CMakeFiles/dense_mapping.dir/dense_mapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dense_mapping.dir/dense_mapping.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cwj/slamebook2/ch12/dense_mono/dense_mapping.cpp -o CMakeFiles/dense_mapping.dir/dense_mapping.cpp.s

# Object files for target dense_mapping
dense_mapping_OBJECTS = \
"CMakeFiles/dense_mapping.dir/dense_mapping.cpp.o"

# External object files for target dense_mapping
dense_mapping_EXTERNAL_OBJECTS =

dense_mapping: CMakeFiles/dense_mapping.dir/dense_mapping.cpp.o
dense_mapping: CMakeFiles/dense_mapping.dir/build.make
dense_mapping: /usr/local/lib/libopencv_gapi.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_highgui.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_ml.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_objdetect.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_photo.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_stitching.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_video.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_videoio.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_dnn.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_calib3d.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_features2d.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_flann.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_imgproc.so.4.8.0
dense_mapping: /usr/local/lib/libopencv_core.so.4.8.0
dense_mapping: CMakeFiles/dense_mapping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cwj/slamebook2/ch12/dense_mono/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable dense_mapping"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dense_mapping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dense_mapping.dir/build: dense_mapping

.PHONY : CMakeFiles/dense_mapping.dir/build

CMakeFiles/dense_mapping.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dense_mapping.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dense_mapping.dir/clean

CMakeFiles/dense_mapping.dir/depend:
	cd /home/cwj/slamebook2/ch12/dense_mono/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cwj/slamebook2/ch12/dense_mono /home/cwj/slamebook2/ch12/dense_mono /home/cwj/slamebook2/ch12/dense_mono/build /home/cwj/slamebook2/ch12/dense_mono/build /home/cwj/slamebook2/ch12/dense_mono/build/CMakeFiles/dense_mapping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dense_mapping.dir/depend
