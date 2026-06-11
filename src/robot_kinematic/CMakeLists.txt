cmake_minimum_required(VERSION 3.8)
project(robot_kinematic)

add_library(robot_kinematic
  src/kinematic.cpp
)

target_include_directories(robot_kinematic
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
