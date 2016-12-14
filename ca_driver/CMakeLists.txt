cmake_minimum_required(VERSION 2.8.3)
project(ca_driver)

find_package(catkin REQUIRED COMPONENTS
  ca_msgs
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libcreate
  CATKIN_DEPENDS
    ca_description
    ca_msgs
    diagnostic_msgs
    diagnostic_updater
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
)

include(ExternalProject)
ExternalProject_Add(libcreate
  GIT_REPOSITORY https://github.com/AutonomyLab/libcreate.git
  GIT_TAG 1.3.0
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

add_executable(${PROJECT_NAME} src/create_driver.cpp)

target_link_libraries(${PROJECT_NAME}
  create
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} libcreate)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
