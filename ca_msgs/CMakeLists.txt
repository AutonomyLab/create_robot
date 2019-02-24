cmake_minimum_required(VERSION 3.5)
project(ca_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bumper.msg"
  "msg/ChargingState.msg"
  "msg/DefineSong.msg"
  "msg/Mode.msg"
  "msg/PlaySong.msg"
  DEPENDENCIES std_msgs
  )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
