cmake_minimum_required(VERSION 3.8)
project(autorobo_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Twistring.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

ament_package()
