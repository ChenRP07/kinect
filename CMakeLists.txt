cmake_minimum_required(VERSION 3.21.0)
project(kinect_reconstruction)

set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

include_directories(${CMAKE_SOURCE_DIR}/include/)
link_directories(${CMAKE_SOURCE_DIR}/lib/)
link_directories(${CMAKE_SOURCE_DIR}/bin/)

add_subdirectory(./src/)

add_executable(kinect ${CMAKE_SOURCE_DIR}/kinect.cpp)
target_link_libraries(kinect k4a k4arecord depthengine_2_0 turbojpeg kinect-dev)
