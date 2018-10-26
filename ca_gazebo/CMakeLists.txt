cmake_minimum_required(VERSION 2.8.3)
project(ca_gazebo)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
