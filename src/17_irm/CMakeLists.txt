cmake_minimum_required(VERSION 2.8.3)
project(irm)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include_directories(
  include
  ../includes  
)

add_library(irm
  src/IRM.cpp src/AbstractIRM.cpp
)

add_executable(irm_node src/main.cpp src/FileIO.cpp)

target_link_libraries(irm_node
  irm
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)


