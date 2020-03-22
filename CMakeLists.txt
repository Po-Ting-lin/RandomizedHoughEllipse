cmake_minimum_required(VERSION 3.15)
project(RandomizedHoughEllipse)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}")
# EIGEN3_INCLUDE_DIRS = /usr/include/eigen3

set(CMAKE_CXX_STANDARD 14)


add_executable(RandomizedHoughEllipse main.cpp utils.cpp utils.h RandomizedHough.cpp RandomizedHough.h HoughException.cpp HoughException.h)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})


target_link_libraries(RandomizedHoughEllipse ${OpenCV_LIBS})
