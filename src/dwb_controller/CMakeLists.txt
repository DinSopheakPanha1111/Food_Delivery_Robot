cmake_minimum_required(VERSION 3.8)
project(dwb_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_dwb_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(food_del_robot REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# ================= DWB CONTROLLER =================
add_executable(dwb_controller
  src/dwb_controller.cpp
)

ament_target_dependencies(dwb_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  food_del_robot
)

install(TARGETS
  dwb_controller
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
