cmake_minimum_required(VERSION 2.8.3)
project(projective_geometry)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include_directories(
  include
  ../includes
)

add_library(projective_geometry
  src/ProjectiveGeometry.cpp
)

add_executable(projective_geometry_node src/main.cpp)

target_link_libraries(projective_geometry_node
  projective_geometry
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)


