cmake_minimum_required(VERSION 3.16.3)
project(create_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Bumper.msg
  ChargingState.msg
  DefineSong.msg
  Mode.msg
  PlaySong.msg
  MotorSetpoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
