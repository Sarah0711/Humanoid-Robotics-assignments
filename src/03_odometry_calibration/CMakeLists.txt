cmake_minimum_required(VERSION 2.8.3)
project(odometry_calibration)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include_directories(
  include
  ../includes
)

add_library(odometry_calibration SHARED
  src/FileIO.cpp src/OdometryCalibration.cpp
)

add_executable(odometry_calibration_node src/main.cpp)

target_link_libraries(odometry_calibration_node
  odometry_calibration
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)

