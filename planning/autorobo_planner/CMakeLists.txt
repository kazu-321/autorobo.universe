cmake_minimum_required(VERSION 3.8)
project(autorobo_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autorobo_planner SHARED
  DIRECTORY src
)

target_include_directories(autorobo_planner
  SYSTEM PUBLIC
)


rclcpp_components_register_node(autorobo_planner
  PLUGIN "planner_node::PlannerNode"
  EXECUTABLE planner_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)