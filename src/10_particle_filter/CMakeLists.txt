cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)

find_package(Boost 1.53 QUIET COMPONENTS random)

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
if(${Boost_RANDOM_FOUND})
  add_definitions(-DBoost_RANDOM_FOUND=1)
endif()

include_directories(
  include
  ../includes
  ${Boost_INCLUDE_DIRS}
)
add_library(particle_filter
  src/ParticleFilter.cpp src/FileIO.cpp
)

add_executable(particle_filter_node src/main.cpp)

target_link_libraries(particle_filter_node
  ${Boost_LIBRARIES} particle_filter
)

enable_testing()
include_directories(../gtest/include ../gtest)
add_executable(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp ../gtest/src/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread ${Boost_LIBRARIES})
add_test(NAME run-${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)

