cmake_minimum_required(VERSION 3.15)
project(RandomizedHoughEllipse)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)


add_executable(RandomizedHoughEllipse main.cpp utils.cpp utils.h RandomizedHough.cpp RandomizedHough.h HoughException.cpp HoughException.h)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RandomizedHoughEllipse ${OpenCV_LIBS})
