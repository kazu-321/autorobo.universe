cmake_minimum_required(VERSION 3.8)
project(autorobo_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
  
add_executable(ransac src/ransac_node.cpp)
target_compile_features(ransac PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ransac
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros)

install(TARGETS
  ransac
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
