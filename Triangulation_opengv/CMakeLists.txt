cmake_minimum_required(VERSION 2.8)
project(triangulatin)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS}")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(opengv REQUIRED)

include_directories(
        "/usr/include/eigen3/"
        "./include"
)

add_library( random_generators src/random_generators.cpp 
                               src/random_generators.hpp 
                               src/experiment_helpers.cpp 
                               src/experiment_helpers.hpp 
                               src/time_measurement.cpp 
                               src/time_measurement.hpp )
target_link_libraries( random_generators opengv)

add_executable( test_triangulation src/test_triangulation.cpp )
target_link_libraries( test_triangulation opengv random_generators )
  