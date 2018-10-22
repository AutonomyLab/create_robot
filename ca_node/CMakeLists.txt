cmake_minimum_required(VERSION 2.8.3)
project(ca_node)

find_package(catkin REQUIRED COMPONENTS
  libcreate
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  ca_msgs
  diagnostic_msgs
  diagnostic_updater
  nodelet
  pluginlib
  ecl_threads
  ca_driver
)

find_package(Boost REQUIRED system thread)

catkin_package(
  LIBRARIES ca_nodelet ca_driver
  CATKIN_DEPENDS libcreate roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf ca_msgs ca_description diagnostic_msgs diagnostic_updater nodelet pluginlib
ecl_threads
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/kinect_power.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


