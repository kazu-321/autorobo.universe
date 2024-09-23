cmake_minimum_required(VERSION 3.8)
project(autorobo_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(twistring REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(simulation_node src/simulation_node.cpp)
target_compile_features(simulation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(simulation_node
  rclcpp
  sensor_msgs
  twistring
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS
  simulation_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
