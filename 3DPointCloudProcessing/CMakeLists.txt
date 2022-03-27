cmake_minimum_required(VERSION 2.8)
project(3d_point_cloud_processing)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS}")

find_package(PCL REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(pass_through_filter src/PassThroughFilter.cpp)
target_link_libraries(pass_through_filter ${PCL_LIBRARIES})

add_executable(voxel_grid_filter src/VoxelGridFilter.cpp)
target_link_libraries(voxel_grid_filter ${PCL_LIBRARIES})

add_executable(statistical_outlier_filter src/StatisticalOutlierFilter.cpp)
target_link_libraries(statistical_outlier_filter ${PCL_LIBRARIES})

add_executable(normal_estimation src/NormalEstimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES})

add_executable(clustering src/Clustering.cpp)
target_link_libraries(clustering ${PCL_LIBRARIES})