cmake_minimum_required(VERSION 3.5)
project(autorobo_visualization)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autorobo_visualization SHARED
  DIRECTORY src
)

target_include_directories(autorobo_visualization
  SYSTEM PUBLIC
)

rclcpp_components_register_node(autorobo_visualization
  PLUGIN "visualize_lines_node::VisualizeLinesNode"
  EXECUTABLE visualize_lines_node
)

rclcpp_components_register_node(autorobo_visualization
  PLUGIN "visualize_stl_node::VisualizeStlNode"
  EXECUTABLE visualize_stl_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)