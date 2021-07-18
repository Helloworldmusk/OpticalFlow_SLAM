cmake_minimum_required(VERSION 2.8)
project(opticalflow_slam)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#include local .h files
include_directories(${PROJECT_SOURCE_DIR})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
message(" current  source dir : ${PROJECT_SOURCE_DIR}" )

# Algorithm
include_directories(${PROJECT_SOURCE_DIR}/algorithm)



add_subdirectory(algorithm)
add_subdirectory(app)
