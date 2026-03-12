cmake_minimum_required(VERSION 3.8)
project(food_del_astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ────────────────────────────────────────────────
find_package(ament_cmake     REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(nav_msgs        REQUIRED)
find_package(tf2             REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── Executable ──────────────────────────────────────────────────
add_executable(astar_planner_node
  src/astar_planner.cpp
)

target_include_directories(astar_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(astar_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(astar_planner_node
  tf2_geometry_msgs::tf2_geometry_msgs
)

# ── Install ─────────────────────────────────────────────────────
install(TARGETS astar_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Testing ─────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()