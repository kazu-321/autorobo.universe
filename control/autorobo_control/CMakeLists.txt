cmake_minimum_required(VERSION 3.5)
project(autorobo_control)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autorobo_control SHARED
  DIRECTORY src
)

target_include_directories(autorobo_control
  SYSTEM PUBLIC
)

rclcpp_components_register_node(autorobo_control
  PLUGIN "pure_pursuit_node::PurePursuitNode"
  EXECUTABLE pure_pursuit_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)