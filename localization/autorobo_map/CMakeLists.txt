cmake_minimum_required(VERSION 3.5)
project(autorobo_map)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autorobo_map SHARED
  DIRECTORY src
)

target_include_directories(autorobo_map
  SYSTEM PUBLIC
)

rclcpp_components_register_node(autorobo_map
  PLUGIN "map_node::MapNode"
  EXECUTABLE map_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)