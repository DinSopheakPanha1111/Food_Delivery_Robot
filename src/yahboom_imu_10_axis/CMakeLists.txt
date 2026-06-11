cmake_minimum_required(VERSION 3.8)
project(yahboom_imu_10_axis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ================= IMU Serial Library =================
add_library(imu_serial
  src/imu_serial.cpp
)

target_include_directories(imu_serial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_serial
  rclcpp
)

# ================= Driver Node =================
add_executable(imu_driver
  src/imu_driver.cpp
)

target_link_libraries(imu_driver
  imu_serial
)

ament_target_dependencies(imu_driver
  rclcpp
  sensor_msgs
)

# ================= Install =================
install(
  TARGETS imu_serial imu_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)
ament_package()
