cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(region_growing_ply)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(region_growing region_growing_ply.cpp)
target_link_libraries(region_growing ${PCL_LIBRARIES})
