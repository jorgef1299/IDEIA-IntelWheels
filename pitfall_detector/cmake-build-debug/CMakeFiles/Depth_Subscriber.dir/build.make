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
CMAKE_COMMAND = /opt/clion-2020.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2020.3.2/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jorge/ros2_ws/src/pitfall_detector

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Depth_Subscriber.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Depth_Subscriber.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Depth_Subscriber.dir/flags.make

CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.o: CMakeFiles/Depth_Subscriber.dir/flags.make
CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.o: ../src/depth_subscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.o -c /home/jorge/ros2_ws/src/pitfall_detector/src/depth_subscriber.cpp

CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/ros2_ws/src/pitfall_detector/src/depth_subscriber.cpp > CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.i

CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/ros2_ws/src/pitfall_detector/src/depth_subscriber.cpp -o CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.s

CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.o: CMakeFiles/Depth_Subscriber.dir/flags.make
CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.o: ../src/ground_segmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.o -c /home/jorge/ros2_ws/src/pitfall_detector/src/ground_segmentation.cpp

CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/ros2_ws/src/pitfall_detector/src/ground_segmentation.cpp > CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.i

CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/ros2_ws/src/pitfall_detector/src/ground_segmentation.cpp -o CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.s

# Object files for target Depth_Subscriber
Depth_Subscriber_OBJECTS = \
"CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.o" \
"CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.o"

# External object files for target Depth_Subscriber
Depth_Subscriber_EXTERNAL_OBJECTS =

Depth_Subscriber: CMakeFiles/Depth_Subscriber.dir/src/depth_subscriber.cpp.o
Depth_Subscriber: CMakeFiles/Depth_Subscriber.dir/src/ground_segmentation.cpp.o
Depth_Subscriber: CMakeFiles/Depth_Subscriber.dir/build.make
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libimage_transport.so
Depth_Subscriber: /opt/ros/foxy/lib/libmessage_filters.so
Depth_Subscriber: /opt/ros/foxy/lib/librclcpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librclcpp.so
Depth_Subscriber: /opt/ros/foxy/lib/liblibstatistics_collector.so
Depth_Subscriber: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
Depth_Subscriber: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libtracetools.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_runtime_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
Depth_Subscriber: /opt/ros/foxy/lib/libclass_loader.so
Depth_Subscriber: /opt/ros/foxy/lib/librcutils.so
Depth_Subscriber: /opt/ros/foxy/lib/librcpputils.so
Depth_Subscriber: /opt/ros/foxy/lib/libament_index_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libclass_loader.so
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
Depth_Subscriber: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librmw_implementation.so
Depth_Subscriber: /opt/ros/foxy/lib/librmw.so
Depth_Subscriber: /opt/ros/foxy/lib/librcl_logging_spdlog.so
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libspdlog.so.1.5.0
Depth_Subscriber: /opt/ros/foxy/lib/libyaml.so
Depth_Subscriber: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
Depth_Subscriber: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_typesupport_c.so
Depth_Subscriber: /opt/ros/foxy/lib/librosidl_runtime_c.so
Depth_Subscriber: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
Depth_Subscriber: /opt/ros/foxy/lib/librcpputils.so
Depth_Subscriber: /opt/ros/foxy/lib/librcutils.so
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
Depth_Subscriber: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
Depth_Subscriber: CMakeFiles/Depth_Subscriber.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Depth_Subscriber"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Depth_Subscriber.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Depth_Subscriber.dir/build: Depth_Subscriber

.PHONY : CMakeFiles/Depth_Subscriber.dir/build

CMakeFiles/Depth_Subscriber.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Depth_Subscriber.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Depth_Subscriber.dir/clean

CMakeFiles/Depth_Subscriber.dir/depend:
	cd /home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jorge/ros2_ws/src/pitfall_detector /home/jorge/ros2_ws/src/pitfall_detector /home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug /home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug /home/jorge/ros2_ws/src/pitfall_detector/cmake-build-debug/CMakeFiles/Depth_Subscriber.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Depth_Subscriber.dir/depend

