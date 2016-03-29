cmake_minimum_required(VERSION 2.8.3)
project(ca_tools)

find_package(catkin REQUIRED
)

catkin_package(
  CATKIN_DEPENDS joy_teleop
)
