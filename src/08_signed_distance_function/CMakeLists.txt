cmake_minimum_required(VERSION 2.8.3)
project(signed_distance_function)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include_directories(
  include
  ../includes
)

add_library(signed_distance_function
  src/SignedDistanceFunction.cpp src/FileIO.cpp
)

add_executable(signed_distance_function_node src/main.cpp)

target_link_libraries(signed_distance_function_node
  signed_distance_function
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)

