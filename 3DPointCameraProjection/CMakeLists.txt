cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executables for exercises

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)
add_executable (lidar_camera src/lidar_camera.cpp src/utils.cpp)
target_link_libraries (lidar_camera ${OpenCV_LIBRARIES})
