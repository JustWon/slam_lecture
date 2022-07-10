cmake_minimum_required(VERSION 2.8)
project(3d_point_cloud_registration)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS}")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(closed_form_registration src/closed_form_registration.cpp)
target_link_libraries(closed_form_registration
        g2o_core g2o_stuff
        ${OpenCV_LIBS}
        )

add_executable(iterative_form_registration src/iterative_form_registration.cpp)
target_link_libraries(iterative_form_registration
        ${PCL_LIBRARIES}
        )
