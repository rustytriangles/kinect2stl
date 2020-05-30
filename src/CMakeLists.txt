cmake_minimum_required(VERSION 3.16)

project(kinect2stl)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(kinect2stl kinect2stl.cpp)

target_include_directories(kinect2stl PUBLIC "/usr/local/include/libfreenect")

target_link_libraries(kinect2stl -L/usr/local/lib)
target_link_libraries(kinect2stl libfreenect.a libfreenect_sync.a)

