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
find_package(map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(food_del_robot REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# ================= OBSTACLE DETECTOR LIBRARY =================
add_library(obstacle_detector
  src/obstacle_detector.cpp
)

target_include_directories(obstacle_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ================= DWB CONTROLLER LIBRARY =================
add_library(dwb_controller
  src/dwb_controller.cpp
)

target_include_directories(dwb_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dwb_controller
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ================= MAIN =================
add_executable(main
  src/main.cpp
)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(main
  obstacle_detector
  dwb_controller
)

ament_target_dependencies(main
  rclcpp
  geometry_msgs
  nav_msgs
  map_msgs
  sensor_msgs
  tf2_ros
  tf2                   # add this
  nav2_costmap_2d       # add this
  food_del_robot
)

# ================= INSTALL =================
install(TARGETS
  main
  obstacle_detector
  dwb_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()