cmake_minimum_required(VERSION 3.8)
project(food_del_robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(my_controller REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
