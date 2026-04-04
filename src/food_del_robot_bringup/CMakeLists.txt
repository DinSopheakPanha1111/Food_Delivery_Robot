cmake_minimum_required(VERSION 3.8)
project(food_del_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
