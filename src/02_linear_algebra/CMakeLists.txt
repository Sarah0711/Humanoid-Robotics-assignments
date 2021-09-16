cmake_minimum_required(VERSION 2.8.3)
project(linear_algebra)

add_definitions(-DPROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR})

include_directories(
  include
  ../includes  
)

add_library(linear_algebra SHARED
  src/LinearAlgebra.cpp
)

add_executable(linear_algebra_node src/main.cpp)

target_link_libraries(linear_algebra_node
  linear_algebra
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)

