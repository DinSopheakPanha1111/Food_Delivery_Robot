cmake_minimum_required(VERSION 3.8)
project(my_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create executable
add_executable(my_controller src/my_controller.cpp)

# Link dependencies
ament_target_dependencies(my_controller
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install executable
install(TARGETS
  my_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()