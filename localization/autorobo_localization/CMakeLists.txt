cmake_minimum_required(VERSION 3.5)
project(autorobo_localization)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autorobo_localization SHARED
  DIRECTORY src
)

target_include_directories(autorobo_localization
  SYSTEM PUBLIC
)

rclcpp_components_register_node(autorobo_localization
  PLUGIN "ransac_node::RansacNode"
  EXECUTABLE ransac_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)