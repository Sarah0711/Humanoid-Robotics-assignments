cmake_minimum_required(VERSION 2.8.3)
project(forward_kinematics)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include_directories(
  include
  ../includes
)

add_library(forward_kinematics
  src/ForwardKinematics.cpp src/FileIO.cpp
)

add_executable(forward_kinematics_node src/main.cpp)

target_link_libraries(forward_kinematics_node
  forward_kinematics
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)

