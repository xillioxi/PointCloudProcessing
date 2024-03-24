cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(test_project)

find_package(PCL 1.8 REQUIRED)

set(BOOST_INCLUDE_DIR "/usr/include/boost")
set(BOOST_LIBRARY_DIR "/usr/lib/aarch64-linux-gnu/")


# Find Boost package - specify components if needed
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Include directories from PCL and Boost
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test test.cpp)
target_link_libraries(test ${PCL_LIBRARIES})
