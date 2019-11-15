cmake_minimum_required(VERSION 3.5)
project(cleanbot)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(OpenCV_INCLUDE_DIRS)
include_directories(/usr/include/eigen3)
add_executable(cleanbot main.cpp)
target_link_libraries(cleanbot ${OpenCV_LIBS})

