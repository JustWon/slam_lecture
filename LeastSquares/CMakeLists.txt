cmake_minimum_required(VERSION 2.8)
project(ch6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

add_executable(gauss_newton src/gauss_newton.cpp)
target_link_libraries(gauss_newton ${OpenCV_LIBS})

add_executable(ceres_curve_fitting src/ceres_curve_fitting.cpp)
target_link_libraries(ceres_curve_fitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(g2o_curve_fitting src/g2o_curve_fitting.cpp)
target_link_libraries(g2o_curve_fitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
