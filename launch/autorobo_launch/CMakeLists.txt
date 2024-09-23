cmake_minimum_required(VERSION 3.8)
project(autorobo_launch)
find_package(ament_cmake_auto REQUIRED)
find_package(domain_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  model3d
  params
  rviz
  map
)
