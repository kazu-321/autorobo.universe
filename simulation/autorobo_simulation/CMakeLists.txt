cmake_minimum_required(VERSION 3.8)
project(autorobo_simulation)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autorobo_simulation SHARED
  DIRECTORY src
)

target_include_directories(autorobo_simulation
  SYSTEM PUBLIC
)


rclcpp_components_register_node(autorobo_simulation
  PLUGIN "simulation_node::OmniSim"
  EXECUTABLE simulation_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)