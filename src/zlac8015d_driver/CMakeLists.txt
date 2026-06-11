cmake_minimum_required(VERSION 3.8)
project(zlac8015d_driver)

# ================= Compiler Options =================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= ROS Dependencies =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(food_del_robot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)


# ================= External Kinematic Library =================
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../robot_kinematic
  ${CMAKE_CURRENT_BINARY_DIR}/robot_kinematic
)

# ================= CAN Library =================
add_library(can
  src/can.cpp
)

target_include_directories(can
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/zlac8015d_driver>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(can
  rclcpp
)

# ================= zlac8015d Driver Library =================
add_library(zlac8015d_driver
  src/zlac8015d_driver.cpp
)

target_link_libraries(zlac8015d_driver
  can
)

ament_target_dependencies(zlac8015d_driver
  rclcpp
)

# ================= Drive and Odom Node =================
add_executable(drive_and_odom
  src/drive_and_odom.cpp
)

target_link_libraries(drive_and_odom
  can
  zlac8015d_driver
  robot_kinematic   
)

ament_target_dependencies(drive_and_odom
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  food_del_robot_interfaces
)

# ================= Drive and Odom Node V2 =================
add_executable(drive_and_odom_v2
  src/drive_and_odom_v2.cpp
)

target_link_libraries(drive_and_odom_v2
  can
  zlac8015d_driver
  robot_kinematic   
)

ament_target_dependencies(drive_and_odom_v2
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  food_del_robot_interfaces
)

# ================= EKF Node =================
add_executable(ekf_node
  src/ekf_node.cpp
)

target_link_libraries(ekf_node
  Eigen3::Eigen
)

ament_target_dependencies(ekf_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)
# ================= Install =================
install(
  TARGETS can zlac8015d_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS drive_and_odom drive_and_odom_v2 ekf_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_package()
