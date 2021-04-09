CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(pitfall_detector)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

SET (ZED2_SOURCE_FILES src/zed_publisher.cpp src/zed2_camera.cpp)
SET (DEPTH_SUB_SOURCE_FILES src/depth_subscriber.cpp src/ground_segmentation.cpp)


add_executable(zed2_pub ${ZED2_SOURCE_FILES})
ament_target_dependencies(zed2_pub rclcpp image_transport)
add_definitions(-std=c++14 -O3)
if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()
TARGET_LINK_LIBRARIES(zed2_pub ${ZED_LIBS} ${OpenCV_LIBRARIES})

add_executable(Depth_Subscriber ${DEPTH_SUB_SOURCE_FILES})
ament_target_dependencies(Depth_Subscriber rclcpp image_transport sensor_msgs)
add_definitions(-std=c++14 -O3)
TARGET_LINK_LIBRARIES(Depth_Subscriber ${OpenCV_LIBRARIES})

ament_package()