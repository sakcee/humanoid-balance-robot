cmake_minimum_required(VERSION 3.8)
project(humanoid_balance_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# ── Fall Detection Node ──────────────────────────────────────────────────────
add_executable(fall_detection_node src/fall_detection_node.cpp)
ament_target_dependencies(fall_detection_node ${DEPENDENCIES})

# ── Anti-Disturbance Controller Node ────────────────────────────────────────
add_executable(anti_disturbance_controller src/anti_disturbance_controller.cpp)
ament_target_dependencies(anti_disturbance_controller ${DEPENDENCIES})

# ── Balance Monitor Node ─────────────────────────────────────────────────────
add_executable(balance_monitor src/balance_monitor.cpp)
ament_target_dependencies(balance_monitor ${DEPENDENCIES})

install(TARGETS
  fall_detection_node
  anti_disturbance_controller
  balance_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  config
  rviz
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
