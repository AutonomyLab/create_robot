cmake_minimum_required(VERSION 2.8.3)
project(ca_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  ca_msgs
)

find_package(Boost REQUIRED system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libcreate
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf ca_msgs ca_description
)

include(ExternalProject)
ExternalProject_Add(libcreate
  GIT_REPOSITORY https://github.com/AutonomyLab/libcreate.git
  GIT_TAG 1.2.0
  PREFIX ${CATKIN_DEVEL_PREFIX}
  CONFIGURE_COMMAND cmake .
  BUILD_COMMAND make
  INSTALL_COMMAND echo "No install"
  BUILD_IN_SOURCE 1
)

set(libcreate_PATH "${CATKIN_DEVEL_PREFIX}/src/libcreate/")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${libcreate_PATH}/include
  include
)

link_directories(${libcreate_PATH})

add_executable(ca_driver src/create_driver.cpp)

target_link_libraries(ca_driver
  create
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(ca_driver libcreate)
