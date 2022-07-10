cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(3d_point_camera_projection)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executables for exercises

add_executable (3d_point_camera_projection src/main.cpp src/utils.cpp)
target_link_libraries (3d_point_camera_projection ${OpenCV_LIBRARIES})
