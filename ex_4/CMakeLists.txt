cmake_minimum_required(VERSION 2.8)
project(undistorted)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE Release)
SET( CMAKE_CXX_FLAGS "-O3 -std=c++11" )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PROJECT_SOURCE_DIR}/lib)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
