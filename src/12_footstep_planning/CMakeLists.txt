cmake_minimum_required(VERSION 2.8.3)
project(footstep_planning)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include_directories(
  include
  ../includes
)

add_library(footstep_planning
    src/ClosedList.cpp
    src/FileIO.cpp
    src/FootstepMap.cpp
    src/FootstepNode.cpp
    src/FootstepPlanning.cpp
    src/OpenList.cpp
    src/PathPlanning.cpp
)

add_executable(footstep_planning_node src/main.cpp)

target_link_libraries(footstep_planning_node
  footstep_planning
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)

