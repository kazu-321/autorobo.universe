cmake_minimum_required(VERSION 3.8)
project(autorobo_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(visualize_stl_node src/visualize_stl_node.cpp)
add_executable(visualize_lines_node src/visualize_lines_node.cpp)
target_compile_features(visualize_stl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(visualize_lines_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(visualize_stl_node
  rclcpp
  visualization_msgs)

ament_target_dependencies(visualize_lines_node
  rclcpp
  visualization_msgs)

install(TARGETS visualize_stl_node visualize_lines_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
