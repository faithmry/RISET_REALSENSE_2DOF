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
CMAKE_SOURCE_DIR = /home/faith/Documents/RISET_REALSENSE_2DoF/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/faith/Documents/RISET_REALSENSE_2DoF/build

# Include any dependencies generated for this target.
include vision/CMakeFiles/capture_rs.dir/depend.make

# Include the progress variables for this target.
include vision/CMakeFiles/capture_rs.dir/progress.make

# Include the compile flags for this target's objects.
include vision/CMakeFiles/capture_rs.dir/flags.make

vision/CMakeFiles/capture_rs.dir/src/capture_rs.cpp.o: vision/CMakeFiles/capture_rs.dir/flags.make
vision/CMakeFiles/capture_rs.dir/src/capture_rs.cpp.o: /home/faith/Documents/RISET_REALSENSE_2DoF/src/vision/src/capture_rs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/faith/Documents/RISET_REALSENSE_2DoF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object vision/CMakeFiles/capture_rs.dir/src/capture_rs.cpp.o"
	cd /home/faith/Documents/RISET_REALSENSE_2DoF/build/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/capture_rs.dir/src/capture_rs.cpp.o -c /home/faith/Documents/RISET_REALSENSE_2DoF/src/vision/src/capture_rs.cpp

vision/CMakeFiles/capture_rs.dir/src/capture_rs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/capture_rs.dir/src/capture_rs.cpp.i"
	cd /home/faith/Documents/RISET_REALSENSE_2DoF/build/vision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/faith/Documents/RISET_REALSENSE_2DoF/src/vision/src/capture_rs.cpp > CMakeFiles/capture_rs.dir/src/capture_rs.cpp.i

vision/CMakeFiles/capture_rs.dir/src/capture_rs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/capture_rs.dir/src/capture_rs.cpp.s"
	cd /home/faith/Documents/RISET_REALSENSE_2DoF/build/vision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/faith/Documents/RISET_REALSENSE_2DoF/src/vision/src/capture_rs.cpp -o CMakeFiles/capture_rs.dir/src/capture_rs.cpp.s

# Object files for target capture_rs
capture_rs_OBJECTS = \
"CMakeFiles/capture_rs.dir/src/capture_rs.cpp.o"

# External object files for target capture_rs
capture_rs_EXTERNAL_OBJECTS =

/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: vision/CMakeFiles/capture_rs.dir/src/capture_rs.cpp.o
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: vision/CMakeFiles/capture_rs.dir/build.make
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libimage_transport.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libmessage_filters.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libclass_loader.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libdl.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libroscpp.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libroslib.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/librospack.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libcv_bridge.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/librosconsole.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/librostime.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /opt/ros/noetic/lib/libcpp_common.so
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_gapi.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_highgui.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_ml.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_objdetect.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_photo.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_stitching.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_video.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_videoio.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_imgcodecs.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_dnn.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_calib3d.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_features2d.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_flann.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_imgproc.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: /usr/local/lib/libopencv_core.so.4.9.0
/home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs: vision/CMakeFiles/capture_rs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/faith/Documents/RISET_REALSENSE_2DoF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs"
	cd /home/faith/Documents/RISET_REALSENSE_2DoF/build/vision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/capture_rs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision/CMakeFiles/capture_rs.dir/build: /home/faith/Documents/RISET_REALSENSE_2DoF/devel/lib/vision/capture_rs

.PHONY : vision/CMakeFiles/capture_rs.dir/build

vision/CMakeFiles/capture_rs.dir/clean:
	cd /home/faith/Documents/RISET_REALSENSE_2DoF/build/vision && $(CMAKE_COMMAND) -P CMakeFiles/capture_rs.dir/cmake_clean.cmake
.PHONY : vision/CMakeFiles/capture_rs.dir/clean

vision/CMakeFiles/capture_rs.dir/depend:
	cd /home/faith/Documents/RISET_REALSENSE_2DoF/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/faith/Documents/RISET_REALSENSE_2DoF/src /home/faith/Documents/RISET_REALSENSE_2DoF/src/vision /home/faith/Documents/RISET_REALSENSE_2DoF/build /home/faith/Documents/RISET_REALSENSE_2DoF/build/vision /home/faith/Documents/RISET_REALSENSE_2DoF/build/vision/CMakeFiles/capture_rs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision/CMakeFiles/capture_rs.dir/depend

