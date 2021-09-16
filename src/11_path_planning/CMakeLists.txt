cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include_directories(
  include
  ../includes
)

add_library(path_planning
  src/PathPlanning.cpp src/GridNode.cpp src/FileIO.cpp src/ClosedList.cpp src/OpenList.cpp
)

add_executable(path_planning_node src/main.cpp)

target_link_libraries(path_planning_node
  path_planning
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)

