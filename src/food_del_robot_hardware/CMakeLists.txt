cmake_minimum_required(VERSION 3.8)
project(food_del_robot_hardware)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= DEPENDENCIES =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)  # ADDED

# ================= LIBRARY (the actual hardware interface) =================
add_library(${PROJECT_NAME} 
  SHARED
  src/can.cpp
  src/zlac8015d_driver.cpp
  src/food_del_robot_hardware_interface.cpp
  src/wheels_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  lifecycle_msgs  # ADDED
)

# ================= INSTALL =================
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the plugin XML file (UNCOMMENT THIS!)
install(
  FILES food_del_robot_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# ================= EXPORT =================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  lifecycle_msgs  
)

pluginlib_export_plugin_description_file(hardware_interface food_del_robot_hw_interface.xml)

ament_package()