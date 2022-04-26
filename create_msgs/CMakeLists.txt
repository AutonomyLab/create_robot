cmake_minimum_required(VERSION 3.5)
project(create_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  msg/Bumper.msg
  msg/ChargingState.msg
  msg/DefineSong.msg
  msg/Mode.msg
  msg/PlaySong.msg
  msg/MotorSetpoint.msg
  msg/Cliff.msg
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
